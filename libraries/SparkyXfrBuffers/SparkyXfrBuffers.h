struct FROM_SPARKY_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t buttonstate;
  int16_t supplyvoltagereading;
  int16_t ballready;
  int16_t packetreceivedcount;
  int16_t transmitpacketcount;
  int16_t shooterspeedecho;
};

struct TO_SPARKY_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t stick1x;
  int16_t stick1y;
  int16_t stick1button;
  int16_t stick2x;
  int16_t stick2y;
  int16_t stick2button;
  int16_t shooterspeed;
  int16_t intake;
  int16_t shoot;
  int16_t drivemode;
  int16_t enabled;
  int16_t counter;
};

