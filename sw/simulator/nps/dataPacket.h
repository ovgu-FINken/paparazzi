#pragma once
#include <boost/serialization/vector.hpp>

/** Enum storing all available sensor types. */
enum class SensorTypes {Sonar, Position, Height, Attitude, Acceleration};

/**
 * Struct used to encapsulate all data that Paparazzi sends to V-REP 
 */
typedef struct{
    int ac_id; /**< the aircraft id */
    double north_east; /**< motorcommand NE*/
    double south_east; /**< motorcommand SE*/
    double south_west; /**< motorcommand SW*/
    double north_west; /**< motorcommand NW*/
    int block_ID;      /**< ID of the current NAV_BLOCK, used for logging data */
    
    /** 
     * Serialization function to send the data via boost::archive
     */
    template <typename Archive>
    void serialize( Archive & ar, const unsigned int version){
        ar & ac_id;
        ar & north_east;
        ar & south_east;
        ar & south_west;
        ar & north_west;
        ar & block_ID;
    }
} paparazziPacket;

/**
 * Struct used to encapsulate all data that V-REP sends to Paparazzi 
 */
typedef struct{
    std::vector<float> pos = {0,0,0};     /**< The position of the copter (x,y,z) */
    std::vector<float> quat = {0,0,0,0};  /**< TThe quaternion of the copter (x,y,z,w)*/
    std::vector<float> vel = {0,0,0};     /**< The velocity of the copter (x,y,z) */
    std::vector<float> rotVel ={0,0,0};   /**< The angular velocity of the copter (x,y,z) */
    std::vector<float> accel = {0,0,0};   /**< The velocity of the copter (x,y,z) */
    std::vector<float> rotAccel ={0,0,0}; /**< The angular acceleration of the copter (x,y,z) */
    double simTime = 0;
    double dt;
    
     /** 
     * Serialization function to send the data via boost::archive
     */
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
