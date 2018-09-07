#include <subsystems/gps/gps_nmea.h>

#ifndef MTK_BAUD
  #error "Undefined baudrate for mtk3339"
#endif

#ifndef MTK_RATE
  #error "Undefined update rate for mtk3339"
#endif

#define MTK_BAUD_STRING STRINGIFY(MTK_BAUD)
#define MTK_UPDATE_STRING STRINGIFY(1000/MTK_UPDATE)

#define MTK_BAUD_RATE_CMD "$PMTK251,"
#define MTK_UPDATE_RATE_CMD "$PMTK220,"

static uint8_t toHex(uint8_t digit){
  return digit<10?digit+'0':digit-10+'A';
}

static void sendCmd(const char* cmd, uint8_t cmdSize, const char* arg, uint8_t argSize) {
  struct link_device* dev = &((NMEA_GPS_LINK).device); 
  char cmdBuffer[32];
  char* it=cmdBuffer;

  memcpy(it, cmd, cmdSize);
  it+= cmdSize;
  memcpy(it, arg, argSize);
  it+= argSize;
  uint8_t crc=nmea_calc_crc(cmdBuffer+1,it-cmdBuffer);
  *it++='*';
  *it++=toHex((crc>>4)&0x0F);
  *it++=toHex(crc&0x0F);
  *it++='\r';
  *it++='\n';
  
  dev->put_buffer(dev->periph, 0, (uint8_t*)cmdBuffer, it-cmdBuffer);
}

void nmea_configure(void) {

  struct link_device* dev = &((NMEA_GPS_LINK).device); 
  //Send BaudRateChange Command
  sendCmd(MTK_BAUD_RATE_CMD, sizeof(MTK_BAUD_RATE_CMD)-1, 
          MTK_BAUD_STRING  , sizeof(MTK_BAUD_STRING)-1  );
  //Wait for the transmission to finish
  while(!dev->check_free_space(dev->periph, NULL, UART_TX_BUFFER_SIZE-1));
  //Change the baud rate of the UART
  dev->set_baudrate(dev->periph, MTK_BAUD);
  sendCmd(MTK_UPDATE_RATE_CMD, sizeof(MTK_UPDATE_RATE_CMD)-1, 
          MTK_UPDATE_STRING  , sizeof(MTK_UPDATE_STRING)-1  );
  //Done
  gps_nmea.is_configured = true;
}

void nmea_parse_prop_init(void) {

}

bool nmea_parse_prop_msg(void) {
  //Parse VTG Messages
  return false;
}
