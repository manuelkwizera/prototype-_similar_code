#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h> //


/**
 * PMS7003 sensor pin map and packet header
 */

//#define MODEM_TX             27
//#define MODEM_RX             26
#define PMS7003_TX          14
#define PMS7003_RX          12
#define PMS7003_PREAMBLE_1  0x42 // From PMS7003 datasheet
#define PMS7003_PREAMBLE_2  0x4D
#define PMS7003_DATA_LENGTH 31
/**  `
 *   Wemos serial RX - TX PMS7003
 *                TX - RX
 */
SoftwareSerial pmsSerial(PMS7003_TX, PMS7003_RX); // RX, TX
int _pm1, _pm25, _pm10;

void readPmsSensor() {
  int checksum = 0;
  unsigned char pms[32] = {0,};
  /**
   * Search preamble for Packet
   * Solve trouble caused by delay function
   */
  while( pmsSerial.available() && 
      pmsSerial.read() != PMS7003_PREAMBLE_1 &&
      pmsSerial.peek() != PMS7003_PREAMBLE_2 ) {
  }   
  if( pmsSerial.available() >= PMS7003_DATA_LENGTH ){
    pms[0] = PMS7003_PREAMBLE_1;
    checksum += pms[0];
    for(int j=1; j<32 ; j++){
      pms[j] = pmsSerial.read();
      if(j < 30)
        checksum += pms[j];
    }
    pmsSerial.flush();
    if( pms[30] != (unsigned char)(checksum>>8) 
      || pms[31]!= (unsigned char)(checksum) ){
      Serial.println("Checksum error");
      return;
    }
    if( pms[0]!=0x42 || pms[1]!=0x4d ) {
      Serial.println("Packet error");
      return;
    }
    _pm1  = makeWord(pms[10],pms[11]);
    _pm25 = makeWord(pms[12],pms[13]);
    _pm10 = makeWord(pms[14],pms[15]);
  }   
}

void setup()
{
  Serial.begin(115200); // For debugging
  pmsSerial.begin(9600);  // For communicating with PMS7003 sensor
  Serial.printf("\nAir quality monitoring system using PMS7003 sensor with WIFI\n");
  
}

void loop()
{
  readPmsSensor();
  Serial.printf("PM1.0:%d PM2.5:%d PM10.0:%d\n", _pm1, _pm25, _pm10);
   delay(3000); // Wait 5 minutes
}