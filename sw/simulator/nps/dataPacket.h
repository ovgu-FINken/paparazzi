

typedef struct{
    double nw;
    double ne;
    double se;
    double sw;

    template <typename Archive>
    void serialize( Archive & ar, const unsigned int version){
        ar & nw;
        ar & ne;
        ar & se;
        ar & sw;
    }
} paparazziPacket;


typedef struct{
    double x;
    double y;
    double z;

    template <typename Archive>
    void serialize( Archive & ar, const unsigned int version){
        ar & x;
        ar & y;
        ar & z;
    }
} vrepPacket;
