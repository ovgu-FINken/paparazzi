#include "nps_fdm.h"

#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/asio.hpp>

#include <fstream>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <thread>
#include <chrono>
#include <dataPacket.h>

using std::endl;
using std::ostream;
using boost::filesystem::ofstream;
using boost::filesystem::current_path;
using boost::asio::ip::tcp;

/*initlializing ecef, lla and ltpref bases*/
LlaCoor_d lla_base;
EcefCoor_d ecef_base;

struct LtpDef_d ltpRef;
struct NpsFdm fdm;
paparazziPacket outPacket;
vrepPacket inPacket;
using Clock = std::chrono::high_resolution_clock;
Clock::time_point lastUpdate;
const std::chrono::milliseconds timeout(10);

ofstream vrepLog(current_path()/"vrep.log");
int iTest = 1;
class VRepClient {
  private:
    tcp::iostream s;
    bool connected=false;
    void connect() {
        while(!connected)
          try
          {
            s.connect("localhost", "50013");
            connected=true;
          }
          catch (std::exception& e)
          {
            vrepLog << "Exception: " << e.what() << "\n";
            std::this_thread::sleep_for(std::chrono::seconds(60));
          }
    }
  public:
      void update(double *commands, const int& commands_nb) {
        outPacket.ac_id = 1;
        outPacket.pitch = commands[0];
	    outPacket.roll = commands[1];
	    outPacket.yaw = commands[3];
        outPacket.thrust = commands[2];
        connect();
        try {
            { 
                auto then = std::chrono::high_resolution_clock::now();
                boost::archive::binary_oarchive out(s);
                out << outPacket;
                auto now = std::chrono::high_resolution_clock::now();
                vrepLog << "sending data computation time: " << std::chrono::nanoseconds(now-then).count()/1000000 << "ms" << std::endl;                      
            }
            
            vrepLog << "Commands sent: " << outPacket.pitch << " | " << outPacket.roll << " | " << outPacket.yaw << " | " << outPacket.thrust << std::endl;
            
            
            auto then = std::chrono::high_resolution_clock::now();
            {
                boost::archive::binary_iarchive in(s);
                in >> inPacket;
                EnuCoor_d enu;
                enu.x = inPacket.x;
                enu.y = inPacket.y;
                enu.z = inPacket.z;
                ecef_of_enu_point_d(&fdm.ecef_pos, &ltpRef, &enu);
                /*
                fdm.ecef_pos.x = inPacket.x;
                fdm.ecef_pos.y = inPacket.y;
                fdm.ecef_pos.z = inPacket.z;
                */
                lla_of_ecef_d(&fdm.lla_pos, &fdm.ecef_pos);
                ned_of_ecef_point_d(&fdm.ltpprz_pos, &ltpRef, &fdm.ecef_pos);
                fdm.hmsl = fdm.lla_pos.alt - 6;
            }

            auto now = std::chrono::high_resolution_clock::now();
            vrepLog << "reading data  coomputation time: " << std::chrono::nanoseconds(now-then).count()/1000000 << "ms" << std::endl;
            vrepLog << "Position received " << inPacket.x << " | " << inPacket.y << " | " << inPacket.z << " | "  << std::endl;

        }
        catch(const std::exception& e) {
            vrepLog << "Exception: " << e.what() << "\n";
            vrepLog << "Error: " << s.error().message() << std::endl;
        }
    }
};

VRepClient client;

void nps_fdm_init(double dt) {
  lla_base.lat = 52;
  lla_base.lon = 11;
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
  vrepLog << "[" << fdm.time << "] vrep fdm init: dt=" << dt << endl;
  lastUpdate = Clock::now();
} 

void nps_fdm_run_step(bool_t launch, double *commands, int commands_nb) {
  Clock::time_point now = Clock::now();
  if(now-lastUpdate < timeout) return;

  lastUpdate = now;
  fdm.time+=fdm.init_dt;
  vrepLog << "[" << fdm.time << "] vrep fdm step: launch=" << (launch?"yes":"no") << " commands=[";
  for(int i=0;i<commands_nb;i++)
    vrepLog << commands[i] << ((i==commands_nb-1)?"":", ");
  vrepLog << "]" << endl;
  auto then = std::chrono::high_resolution_clock::now();
  client.update(commands, commands_nb);
  auto after = std::chrono::high_resolution_clock::now();
  vrepLog << "Client coomputation time: " << std::chrono::nanoseconds(after-then).count()/1000000 << "ms" << std::endl;
}

void nps_fdm_set_wind(double speed, double dir) {
  vrepLog << "[" << fdm.time << "] vrep fdm set wind: speed=" << speed << " dir=" << dir << endl;
}

void nps_fdm_set_wind_ned(double wind_north, double wind_east, double wind_down) {
  vrepLog << "[" << fdm.time << "] vrep fdm init ned wind_north=" << wind_north << " wind_east=" <<  wind_east << " wind_down=" <<  wind_down << endl;
}

void nps_fdm_set_turbulence(double wind_speed, int turbulence_severity) {
  vrepLog << "[" << fdm.time << "] vrep fdm set turbulance: wind_speed=" << wind_speed << " severity=" << turbulence_severity << endl;
}
