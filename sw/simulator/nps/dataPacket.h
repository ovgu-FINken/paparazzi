#pragma once
#include <boost/serialization/vector.hpp>

enum class SensorTypes {Sonar, Position, Height, Attitude, Acceleration};

typedef struct{
    int ac_id;
    double pitch;
    double roll;
    double yaw;
    double thrust;
    double dt;
    int block_ID;

    template <typename Archive>
    void serialize( Archive & ar, const unsigned int version){
        ar & ac_id;
        ar & pitch;
        ar & roll;
        ar & yaw;
        ar & thrust;
        ar & dt;
        ar & block_ID;
    }
} paparazziPacket;


typedef struct{
    std::vector<double> pos = {0,0,0};
    std::vector<double> quat = {0,0,0,0};
    std::vector<double> vel = {0,0,0};
    std::vector<double> rotVel ={0,0,0};
    std::vector<double> accel = {0,0,0};
    std::vector<double> rotAccel ={0,0,0};
    double simTime = 0;
    double dt;

    template <typename Archive>
    void serialize( Archive & ar, const unsigned int version){
        ar & pos;
        ar & quat;
        ar & vel;
        ar & rotVel;
        ar & accel;
        ar & rotAccel;
        ar &  dt;
    }
} vrepPacket;
