

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

