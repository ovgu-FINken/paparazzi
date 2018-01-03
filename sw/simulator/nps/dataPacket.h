<<<<<<< HEAD


typedef struct{
    double x;
    double y;
    double z;
    double s;

    template <typename Archive>
    void serialize( Archive & ar, const unsigned int version){
        ar & x;
        ar & y;
        ar & z;
        ar & s;
    }
} DataPacket;

=======
#include <boost/serialization/vector.hpp>

typedef struct{
    int ac_id;
    double pitch;
    double roll;
    double yaw;
    double thrust;

    template <typename Archive>
    void serialize( Archive & ar, const unsigned int version){
        ar & ac_id;
        ar & pitch;
        ar & roll;
        ar & yaw;
        ar & thrust;
    }
} paparazziPacket;


typedef struct{
    std::vector<double> pos = {0,0,0};
    std::vector<double> quat = {0,0,0,0};
    std::vector<double> vel = {0,0,0};
    std::vector<double> rotVel ={0,0,0};
    std::vector<double> accel = {0,0,0};
    std::vector<double> rotAccel ={0,0,0};
    
    template <typename Archive>
    void serialize( Archive & ar, const unsigned int version){
        ar & pos;
        ar & quat;
        ar & vel;
        ar & rotVel;
        ar & accel;
        ar & rotAccel;
    }
} vrepPacket;
>>>>>>> 2af1cdb96c18492d0b3f069cf9204def57abc3a8
