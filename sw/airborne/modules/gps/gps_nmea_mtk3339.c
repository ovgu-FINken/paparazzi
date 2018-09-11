#include <subsystems/gps/gps_nmea.h>

#ifndef MTK_BAUD
  #error "Undefined baudrate for mtk3339"
#endif

#ifndef MTK_PERIOD
  #error "Undefined update rate for mtk3339"
#endif

#define MTK_BAUD_STRING STRINGIFY(MTK_BAUD)
#define MTK_PERIOD_STRING STRINGIFY(MTK_PERIOD)

#define MTK_BAUD_RATE_CMD "$PMTK251,"
#define MTK_UPDATE_PERIOD_CMD "$PMTK220,"

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
  for(uint16_t i=0;i<10000;i++)
    if(dev->check_free_space(dev->periph, NULL, UART_TX_BUFFER_SIZE-1))
      break;
  //Change the baud rate of the UART
  dev->set_baudrate(dev->periph, MTK_BAUD);
  //Done
  gps_nmea.is_configured = true;
}

static uint8_t findNextComma(uint8_t pos) {
  for(uint8_t i=pos;i<gps_nmea.msg_len;i++)
    if(gps_nmea.msg_buf[i]==',')
      return i;
  return gps_nmea.msg_len;
}

static bool findNextDatum(uint8_t* pos, uint8_t* next, char c) {

  uint8_t start = findNextComma(*pos)+1;
  uint8_t end = findNextComma(start);
  uint8_t end2 = findNextComma(end+1);
  if(end2-end == 2 && gps_nmea.msg_buf[end+1]==c) {
    gps_nmea.msg_buf[end]='\0';
    *pos=start;
    *next=end2;
    return true;
  } else
    return false;
}

static bool nmea_parse_vtg(void) {
  uint8_t pos=0;
  uint8_t next;
  if(!findNextDatum(&pos, &next, 'T'))
    return false;
  gps_nmea.state.course = atof(gps_nmea.msg_buf+pos);
  pos=next;
  if(!findNextDatum(&pos, &next, 'M'))
    return false;
  pos=next;
  if(!findNextDatum(&pos, &next, 'N'))
    return false;
  pos=next;
  if(!findNextDatum(&pos, &next, 'K'))
    return false;
  gps_nmea.state.gspeed = atof(gps_nmea.msg_buf+pos)*1000/36;
  return true;
}

enum mtk_states {
  INIT,
  DONE
};

static enum mtk_states mtk_state;

static bool nmea_parse_ack(void) {
  uint8_t pos=findNextComma(0)+1;
  uint8_t next=findNextComma(pos);
  gps_nmea.msg_buf[next]='\0';
  uint8_t cmd = atoi(gps_nmea.msg_buf+pos);
  pos=next+1;
  next=findNextComma(pos);
  gps_nmea.msg_buf[next]='\0';
  uint8_t state = atoi(gps_nmea.msg_buf+pos);
  if(cmd==220 && state==3)
    mtk_state=DONE;
  return true;
}


void nmea_parse_prop_init(void) {
  mtk_state=INIT;
}

bool nmea_parse_prop_msg(void) {
  gps_nmea.state.pacc=mtk_state;
  if(mtk_state==INIT) 
    sendCmd(MTK_UPDATE_PERIOD_CMD, sizeof(MTK_UPDATE_PERIOD_CMD)-1, 
            MTK_PERIOD_STRING  , sizeof(MTK_PERIOD_STRING)-1  );
  if( gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[2], "VTG", 3))
    return nmea_parse_vtg();
  if( gps_nmea.msg_len > 5 && !strncmp(&gps_nmea.msg_buf[0], "PMTK001", 7))
    return nmea_parse_ack();
  return false;
}
