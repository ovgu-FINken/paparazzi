#include "nps_fdm.h"

#include <unistd.h>
#include <iostream>
#include <string>
#include <cstdlib>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/asio.hpp>

#include <fstream>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <thread>
#include <chrono>
#include <dataPacket.h>

#pragma GCC diagnostic ignored "-Wint-in-bool-context"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#pragma GCC diagnostic pop


using std::endl;
using std::ostream;
using boost::filesystem::ofstream;
using boost::filesystem::current_path;
using boost::asio::ip::tcp;

ofstream csvdata;
extern uint8_t nav_block;
uint8_t curBlock;
/*initlializing ecef, lla and ltpref bases*/
LlaCoor_d lla_base;
EcefCoor_d ecef_base;

struct LtpDef_d ltpRef;
struct NpsFdm fdm;
paparazziPacket outPacket;
vrepPacket inPacket;
using Clock = std::chrono::high_resolution_clock;
Clock::time_point lastUpdate;
Clock::time_point runEnd;
const std::chrono::milliseconds timeout(34);

Eigen::Matrix3d rotMatrix;

std::string homepath = getenv("HOME");
std::string pprzHome=std::getenv("PAPARAZZI_HOME");

class LogLine {
  private:
    std::ostream& o;

  public:
    LogLine(std::ostream& o) : o(o) {}
    ~LogLine(){
      o <<  ";";
    }
    LogLine& operator<<(std::ostream&(*pf)(std::ostream&)) {
      o << pf;
      return *this;
    }
    template<typename T>
    LogLine& operator<<(T t) {
      o << t;
      return *this;
    }
  friend class VrepLog;
};
class VrepLog {
  private:
    std::ofstream log;
  public:
    VrepLog() {
        log.open((pprzHome + "/paparazzi.log").c_str());
    }
    template<typename T>
    LogLine operator<<(T& t) {
      return log << Clock::now().time_since_epoch().count() << ", " << t;
    }
} vrepLog;

int iTest = 1;
class VRepClient {
  private:
    tcp::iostream s;
    bool connected=false;
    void connect() {
        while(!connected){
          try
          { 
            s.connect("localhost", "50013");
            connected=true;
            return;
          }
          catch (std::exception& e)
          {
            vrepLog << "[PPRZ] Exception: " << e.what() << "\n";
            std::this_thread::sleep_for(std::chrono::seconds(60));
          }
          std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
  public:
      double update(double *commands, const int& commands_nb) {
        outPacket.ac_id = 1;
        outPacket.pitch = commands[0];
	    outPacket.roll = commands[1];
	    outPacket.yaw = commands[2];
        outPacket.thrust = commands[3];
        connect();
        try {
            {
                auto then = std::chrono::high_resolution_clock::now();
                boost::archive::binary_oarchive out(s);
                out << outPacket;
                auto now = std::chrono::high_resolution_clock::now();
                vrepLog << "[PPRZ] sending data computation time: " << std::chrono::nanoseconds(now-then).count()/1000000 << "ms" << std::endl;
            }

            vrepLog << "[PPRZ] Commands sent: " << outPacket.pitch << " | " << outPacket.roll << " | " << outPacket.yaw << " | " << outPacket.thrust << std::endl;


            auto then = std::chrono::high_resolution_clock::now();
            double dt;
            {
                boost::archive::binary_iarchive in(s);
                in >> inPacket;
                dt = inPacket.dt;
                EnuCoor_d enu;
                EnuCoor_d enu_vel;
                EnuCoor_d enu_accel;
                EnuCoor_d enu_rotVel;
                EnuCoor_d enu_rotAccel;
                enu.x = inPacket.pos[0];
                enu.y = inPacket.pos[1];
                enu.z = inPacket.pos[2];
                enu_vel.x = inPacket.vel[0];
                enu_vel.y = inPacket.vel[1];
                enu_vel.z = inPacket.vel[2];
                enu_accel.x = inPacket.accel[0];
                enu_accel.y = inPacket.accel[1];
                enu_accel.z = inPacket.accel[2];
                enu_rotVel.x = inPacket.rotVel[0];
                enu_rotVel.y = inPacket.rotVel[1];
                enu_rotVel.z = inPacket.rotVel[2];
                enu_rotAccel.x = inPacket.rotAccel[0];
                enu_rotAccel.y = inPacket.rotAccel[1];
                enu_rotAccel.z = inPacket.rotAccel[2];
                Eigen::Quaterniond quat(inPacket.quat[3], inPacket.quat[0], -inPacket.quat[1], -inPacket.quat[2]);
		csvdata << std::to_string(inPacket.quat[0]) << "," << std::to_string(inPacket.quat[1]) << "," << std::to_string(-inPacket.quat[2]) << "," << std::to_string(inPacket.quat[3]) << std::endl;
		
		//set simTime
		fdm.time = inPacket.simTime;
		

                //set copter Position:
                ecef_of_enu_point_d(&fdm.ecef_pos, &ltpRef, &enu);
                lla_of_ecef_d(&fdm.lla_pos, &fdm.ecef_pos);
                ned_of_ecef_point_d(&fdm.ltpprz_pos, &ltpRef, &fdm.ecef_pos);
                fdm.hmsl = fdm.lla_pos.alt;
                vrepLog << "[pprz] copter position: " << std::endl
			<< " ecef: " << fdm.ecef_pos.x << " | " << fdm.ecef_pos.y << " | " << fdm.ecef_pos.z << std::endl
			<< " LLA:  " << fdm.lla_pos.lat << " | " << fdm.lla_pos.lon << " | " << fdm.lla_pos.alt << std::endl
			<< " ENU:  " << enu.x << " | " << enu.y << " | " << enu.z << std::endl
			<< " NED:  " << fdm.ltpprz_pos.x << " | " << fdm.ltpprz_pos.y << " | " << fdm.ltpprz_pos.z << std::endl;
		       
                //convert velocity & acceleration from enu to body:
                Eigen::Vector3d body_vel(enu_vel.x, enu_vel.y, enu_vel.z);
                Eigen::Vector3d body_accel(enu_accel.x, enu_accel.y, enu_accel.z);
		Eigen::Vector3d body_rotVel(enu_rotVel.x, enu_rotVel.y, enu_rotVel.z);
		Eigen::Vector3d body_rotAccel(enu_rotAccel.x, enu_rotAccel.y, enu_rotAccel.z);
                body_vel = quat *  body_vel;
                body_accel = quat * body_accel;
		body_rotVel = quat * body_rotVel;
		body_rotAccel = quat * body_rotVel;

                //convert velocties and accelerations from enu to ecef:
                
                ecef_of_enu_vect_d(&fdm.ecef_ecef_vel, &ltpRef, &enu_vel);
                ecef_of_enu_vect_d(&fdm.ecef_ecef_accel, &ltpRef, &enu_accel);
                fdm.body_ecef_vel.x = body_vel[0];
                fdm.body_ecef_vel.y = body_vel[1];
                fdm.body_ecef_vel.z = body_vel[2];
                fdm.body_ecef_accel.x = body_accel[0];
                fdm.body_ecef_accel.y = body_accel[1];
                fdm.body_ecef_accel.z = body_accel[2];
                
                

                // velocity in LTP frame, wrt ECEF frame 
                //struct NedCoor_d ltp_ecef_vel;
                ned_of_ecef_vect_d(&fdm.ltp_ecef_vel, &ltpRef, &fdm.ecef_ecef_vel);

                // acceleration in LTP frame, wrt ECEF frame 
                //struct NedCoor_d ltp_ecef_accel;
                ned_of_ecef_vect_d(&fdm.ltp_ecef_accel, &ltpRef, &fdm.ecef_ecef_accel);

                // velocity in ltppprz frame, wrt ECEF frame 
                //struct NedCoor_d ltpprz_ecef_vel;
                ned_of_ecef_vect_d(&fdm.ltpprz_ecef_vel, &ltpRef, &fdm.ecef_ecef_vel);


                //accel in ltppprz frame, wrt ECEF frame 
                //struct NedCoor_d ltpprz_ecef_accel;
                ned_of_ecef_vect_d(&fdm.ltpprz_ecef_accel, &ltpRef, &fdm.ecef_ecef_accel);




                // acceleration in body frame, wrt ECI inertial frame 
                //struct DoubleVect3 body_inertial_accel;
                
                // acceleration in body frame as measured by an accelerometer (incl. gravity)                
                
                Eigen::Vector3d gravity(0,0,-9.81);
                body_accel = body_accel + quat.inverse()*gravity;
                //body_accel = quat*gravity;
                fdm.body_accel.x = body_accel[0];
                fdm.body_accel.y = body_accel[1];
                fdm.body_accel.z= body_accel[2];
                
                
                //attitude
                Eigen::Quaterniond ecef_to_enu_quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(fdm.ecef_pos.x, fdm.ecef_pos.y, fdm.ecef_pos.z), Eigen::Vector3d(enu.x, enu.y, enu.z));
                Eigen::Quaterniond ecef_to_body_quat = ecef_to_enu_quat * quat;
                fdm.ecef_to_body_quat.qi = ecef_to_body_quat.w();
                fdm.ecef_to_body_quat.qx = ecef_to_body_quat.x();
                fdm.ecef_to_body_quat.qy = ecef_to_body_quat.y();
                fdm.ecef_to_body_quat.qz = ecef_to_body_quat.z();
                
                
                Eigen::Quaterniond ltp_to_enu_quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(fdm.ltpprz_pos.x, fdm.ltpprz_pos.y, fdm.ltpprz_pos.z), Eigen::Vector3d(enu.x, enu.y, enu.z));
                Eigen::Quaterniond ltp_to_body_quat = ltp_to_enu_quat * quat;
                fdm.ltp_to_body_quat.qi = ltp_to_body_quat.w();
                fdm.ltp_to_body_quat.qx = ltp_to_body_quat.x();
                fdm.ltp_to_body_quat.qy = ltp_to_body_quat.y();
                fdm.ltp_to_body_quat.qz = ltp_to_body_quat.z();
                double_eulers_of_quat(&fdm.ltp_to_body_eulers, &fdm.ltp_to_body_quat);

                
                Eigen::Quaterniond ltpprz_to_enu_quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(fdm.ltpprz_pos.x, fdm.ltpprz_pos.y, fdm.ltpprz_pos.z), Eigen::Vector3d(enu.x, enu.y, enu.z));
                Eigen::Quaterniond ltpprz_to_body_quat = ltp_to_enu_quat * quat;
                fdm.ltpprz_to_body_quat.qi = ltpprz_to_body_quat.w();
                fdm.ltpprz_to_body_quat.qx = ltpprz_to_body_quat.x();
                fdm.ltpprz_to_body_quat.qy = ltpprz_to_body_quat.y();
                fdm.ltpprz_to_body_quat.qz = ltpprz_to_body_quat.z();
                double_eulers_of_quat(&fdm.ltpprz_to_body_eulers, &fdm.ltpprz_to_body_quat);
                
		        
		        //angular rates in body frame wrt ECEF:
		        RATES_ASSIGN(fdm.body_ecef_rotvel, body_rotVel[0], body_rotVel[1], body_rotVel[2]);
		        RATES_ASSIGN(fdm.body_ecef_rotaccel, body_rotAccel[0], body_rotAccel[1], body_rotAccel[2]);
                
                
                }

            auto now = std::chrono::high_resolution_clock::now();
            vrepLog << "[PPRZ] reading data  coomputation time: " << std::chrono::nanoseconds(now-then).count()/1000000 << "ms" << std::endl;
            vrepLog << "[PPRZ] Position received " << inPacket.pos[0] << " | " << inPacket.pos[1] << " | " << inPacket.pos[3] << " | "  << std::endl;
            return dt;

        }
        catch(const std::exception& e) {
		vrepLog << "PPRZ] Exception: " << e.what() << "\n";
            vrepLog << "[PPRZ] Error: " << s.error().message() << std::endl;
            return 0;
        }
    }
};

VRepClient client;

void nps_fdm_init(double dt) {
  
  std::string pprzHome=std::getenv("PAPARAZZI_HOME");
  csvdata.open((pprzHome + "/log.csv").c_str());
  curBlock = nav_block;
  csvdata << "TIME,NE,SE,SW,NW,Quat.x,Quat.y,Quat.z,Quat.w" << "\n";
  bzero(&fdm, sizeof(&fdm));
  lla_base.lat = 0.901;
  lla_base.lon = 0.192;
  lla_base.alt = 50;
  ecef_of_lla_d(&ecef_base, &lla_base);
  ltp_def_from_ecef_d(&ltpRef, &ecef_base);
  ltpRef.hmsl = 44;
  fdm.on_ground=1;
  fdm.ecef_pos.x=0.0;
  fdm.ecef_pos.y=0.0;
  fdm.ecef_pos.z=0.0;
  fdm.ecef_ecef_vel.x=0.0f;
  fdm.ecef_ecef_vel.y=0.0f;
  fdm.ecef_ecef_vel.z=0.0f;
  fdm.ltp_ecef_vel.x=0.0f;
  fdm.ltp_ecef_vel.y=0.0f;
  fdm.ltp_ecef_vel.z=0.0f;
  fdm.ecef_ecef_accel.x=0.0f;
  fdm.ecef_ecef_accel.y=0.0f;
  fdm.ecef_ecef_accel.z=0.0f;
  fdm.time=0;
  fdm.init_dt=dt;
  vrepLog << "[PPRZ] [" << fdm.time << "] vrep fdm init: dt=" << dt << endl;
  lastUpdate = Clock::now();
  runEnd = Clock::now();
}

double nps_fdm_run_step(bool_t launch, double *commands, int commands_nb) {
  	
  Clock::time_point now = Clock::now();
  
  auto runStart = Clock::now();
  vrepLog << "[PPRZ] time betweeen 2 consecutive client updates: " << std::chrono::nanoseconds(runStart-runEnd).count()/1000000 << "ms" << std::endl;


  lastUpdate = now;
  //fdm.time+=fdm.init_dt;
  vrepLog << "[PPRZ] [" << fdm.time << "] vrep fdm step: launch=" << (launch?"yes":"no") << " commands=[";
  /*
  if (curBlock != nav_block) {
    curBlock = nav_block;
    csvdata.close();
    csvdata.open((pprzHome + "/navBlock" + std::to_string(nav_block) + ".csv").c_str());
    csvdata << "TIME,NE,SE,SW,NW,Quat.x,Quat.y,Quat.z,Quat.w" << "\n";
  }
  */
  csvdata << fdm.time << ",";
  for(int i=0;i<commands_nb;i++) {
    vrepLog << commands[i] << ((i==commands_nb-1)?"":", ");
    csvdata << commands[i] << ((i==commands_nb-1)?",":",");
  }
  vrepLog << "]" << endl;
  auto then = std::chrono::high_resolution_clock::now();
  double dt = client.update(commands, commands_nb);
  auto after = std::chrono::high_resolution_clock::now();
  vrepLog << "[PPRZ] Client computation time: " << std::chrono::nanoseconds(after-then).count()/1000000 << "ms" << std::endl;
  runEnd = Clock::now();
  return dt;
}

void nps_fdm_set_wind(double speed, double dir) {
  //vrepLog << "[" << fdm.time << "] vrep fdm set wind: speed=" << speed << " dir=" << dir << endl;
}

void nps_fdm_set_wind_ned(double wind_north, double wind_east, double wind_down) {
  //vrepLog << "[" << fdm.time << "] vrep fdm set ned wind_north=" << wind_north << " wind_east=" <<  wind_east << " wind_down=" <<  wind_down << endl;
}

void nps_fdm_set_turbulence(double wind_speed, int turbulence_severity) {
  //vrepLog << "[" << fdm.time << "] vrep fdm set turbulance: wind_speed=" << wind_speed << " severity=" << turbulence_severity << endl;
}
