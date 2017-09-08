#include "nps_fdm.h"

#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/asio.hpp>

#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include <thread>
#include <chrono>

using std::endl;
using std::ostream;
using boost::filesystem::ofstream;
using boost::filesystem::current_path;
using boost::asio::ip::tcp;

struct NpsFdm fdm;
struct LtpDef_d ltpRef;

ofstream vrepLog(current_path()/"vrep.log");

class VRepClient {
  private:
    tcp::iostream s;
    std::unique_ptr<boost::archive::text_oarchive> outPtr;
    std::unique_ptr<boost::archive::text_iarchive> inPtr;
    bool connected=false;
    void connect() {
        while(!connected)
          try
          {
            s.connect("localhost", "50013");
            connected=true;
            outPtr.reset(new boost::archive::text_oarchive(s, boost::archive::no_header));
            inPtr.reset(new boost::archive::text_iarchive(s, boost::archive::no_header));
          }
          catch (std::exception& e)
          {
            vrepLog << "Exception: " << e.what() << "\n";
            std::this_thread::sleep_for(std::chrono::seconds(60));
          }
    }
  public:
      void update(double *commands, const int& commands_nb) {
        connect();
        boost::archive::text_oarchive& out=*outPtr;
        boost::archive::text_iarchive& in=*inPtr;
        try {
            out << commands_nb;
            for(int i=0;i<commands_nb;i++) {
	        	  out << commands[i];
            }
            double bogus=0;
            out << bogus;
          
            vrepLog << "Query is: " << commands[0] << std::endl;
            double d;
            in >> d;
            vrepLog << "Reply is: " << d << std::endl;

        }catch(const std::exception& e) {
            vrepLog << "Exception: " << e.what() << "\n";
            vrepLog << "Error: " << s.error().message() << std::endl;
        }
      }
};

VRepClient client;

void nps_fdm_init(double dt) {

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
  ltpRef.ecef.x=0.0;
  ltpRef.ecef.y=0.0;
  ltpRef.ecef.z=0.0;
  ltpRef.lla.lat=0.0;
  ltpRef.lla.lon=0.0;
  ltpRef.lla.alt=0.0;
  ltp_def_from_ecef_d(&ltpRef, &ltpRef.ecef);
  fdm.time=0;
  fdm.init_dt=dt;
  vrepLog << "[" << fdm.time << "] vrep fdm init: dt=" << dt << endl;
}

void nps_fdm_run_step(bool_t launch, double *commands, int commands_nb) {
  fdm.time+=fdm.init_dt;
  vrepLog << "[" << fdm.time << "] vrep fdm step: launch=" << (launch?"yes":"no") << " commands=[";
  for(int i=0;i<commands_nb;i++)
    vrepLog << commands[i] << ((i==commands_nb-1)?"":", ");
  vrepLog << "]" << endl;

  client.update(commands, commands_nb);
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
