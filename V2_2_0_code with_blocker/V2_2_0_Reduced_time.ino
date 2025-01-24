
/*
Bona May 27 2024
* V2.2.0 All is well_Countdowbn
==>V2.1 => Added SD card data logging
==> V2.0.1 Added the Date and time Json
==> V2.0.1 Added the Date and time
==> v2.0: new PMS reading logic
* ==> V1.1_0 Key update: We have added the display of information on start up
* ==> V1.1_0_1 Key update: We commented line 200, (  ESP.restart();) and line 412: // return; to prevent the modem to restart when the MQTT cannot connect
* ==> To prevent it from restarting when no connection to GPRS

*/

// Select your modem:
#define TINY_GSM_MODEM_SIM800  // Modem is SIM800L
// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands
#define SerialAT Serial1
// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon
// set GSM PIN, if any
#define GSM_PIN ""
// Your GPRS credentials, if any
const char apn[] = "default";  //"mtn.internet"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char gprsUser[] = "";
const char gprsPass[] = "";
// SIM card PIN (leave empty, if not defined)
const char simPIN[] = "";
// MQTT details
const char* broker = "154.68.126.10";        // Public IP address or domain name
const char* mqttUsername = "mqtt_username";  // MQTT username
const char* mqttPassword = "mqtt_password";  // MQTT password
const char* topicOutput1 = "esp/output1";
const char* topicOutput2 = "esp/output2";
/*
const char* topicOutput3 = "esp/output3";
const char* topicOutput4 = "esp/output4";
const char* topicOutput5 = "esp/output5";
const char* topicOutput6 = "esp/output6";
*/

String deviceID = "House_004";
const char* topicTemperature = "House_004/temperature";
const char* topicHumidity = "House_004/humidity";
const char* topicPm1 = "House_004/pm1";
const char* topicPm2_5 = "House_004/pm2_5";
const char* topicPm10 = "House_004/pm10";
const char* topicCo = "House_004/co";
const char* topiceCo2 = "House_004/eco2";
const char* topicTvoc = "House_004/tvoc";
const char* topicData = "House_004/Sensor_data";

//const char* topicPressure = "House_004/pressure";
//const char* topicAltitude = "House_004/altitude";
// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS

// Define your time zone offset in hours (e.g., for UTC+3, use 3)
const int timeZoneOffsetHours = 2;


#include <Wire.h>
#include <TinyGsmClient.h>
#include <TimeLib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_BME280.h>  //BME
#include <Adafruit_SH110X.h>  //OLED
#include <Adafruit_CCS811.h>  //VVS8111
#include <SoftwareSerial.h>   //PMS
#include "MQ7.h"              //MQ7
#include "PMS.h"
#include <PubSubClient.h>
#include <ArduinoJson.h>

//Libraries for microSD card
#include "FS.h"
#include "SD.h"
#include "SPI.h"


// Define SD card connection

//Normal Big SD Card
#define SD_MISO 2   //19
#define SD_MOSI 19  //23
#define SD_SCLK 0   //18
#define SD_CS 18    //5


/*
//Normal small SD Card
#define SD_MOSI     19 //23
#define SD_MISO     18 //19
#define SD_SCLK     0 //18 
#define SD_CS       2 //5



//Other big SD Card ports
#define SD_MOSI     0 //23
#define SD_MISO     19 //19
#define SD_SCLK     18 //18 
#define SD_CS       2 //5
*/

File myFile;
File dataFile;


//OLED Screen settup
#define i2c_Address 0x3c  //initialize with the I2C addr 0x3C //OLED Address
#define SCREEN_WIDTH 128  // OLED display width, in pixels 128
#define SCREEN_HEIGHT 64  // OLED display height, in pixels 64
#define OLED_RESET -1     //   QT-PY / XIAO
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

int itimezone = 2;

TinyGsmClient client(modem);
PubSubClient mqtt(client);
// TTGO T-Call pins
#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26
#define I2C_SDA 21
#define I2C_SCL 22
#define OUTPUT_1 2
#define OUTPUT_2 15
uint32_t lastReconnectAttempt = 0;


// Correction factors obtained from the calibration process
const float CORRECTION_FACTOR_PM1 = 0.79;
const float CORRECTION_FACTOR_pm25 = 0.54;
const float CORRECTION_FACTOR_PM10 = 0.6;
const float CORRECTION_FACTOR_eCO2 = 1.4;
const float CORRECTION_FACTOR_Temp = 0.86;
const float CORRECTION_FACTOR_Hum = 1.48;

//BME Initialization
Adafruit_BME280 bme;
#define SEALEVELPRESSURE_HPA (1013.25)
#define IP5306_ADDR 0x75
#define IP5306_REG_SYS_CTL0 0x00

//CCS811 Initialization
Adafruit_CCS811 ccs;

//PM7003 initialization
#define PMS7003_TX 14
#define PMS7003_RX 12
/* 
 *  serial RX - TX PMS7003
 *         TX - RX
 */

#define PMS7003_PREAMBLE_1 0x42  // From PMS7003 datasheet
#define PMS7003_PREAMBLE_2 0x4D
#define PMS7003_DATA_LENGTH 31
SoftwareSerial pms_serial(PMS7003_TX, PMS7003_RX);  // RX, TX
int _pm1, _pm25, _pm10;
float rawCO2, rawTemp, rawHum;


struct RawData {
  float pm1;
  float pm2_5;
  float pm10;
  float neweCO2;
  float temp;
  float hum;
};


//MQ7 initialization and Definig variables
#define MQ7_PIN 25
#define VOLTAGE 3.3  //You can use "5" or "3.3"

// init MQ7 device
MQ7 mq7(MQ7_PIN, VOLTAGE);
float sensorValue;  // variable to store sensor value

// Define your temperature Threshold (in this case it's 28.0 degrees Celsius)
float temperatureThreshold = 28.0;
float coThreshold = 10;
float pm25Threshold = 30;
float pm25CuttOff = 100;
float tvocThreshold = 100;
float coSensorValue;     // variable to store CO sensor value
float pm2_5SensorValue;  // variable to store pm 2.5 sensor value

// Flag variable to keep track if alert SMS was sent or not
bool smsSent = false;
#define SMS_TARGET "+250788817357"

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800    // Modem is SIM800L
#define TINY_GSM_RX_BUFFER 1024  // Set RX buffer to 1Kb

float temperature = 0;
float humidity = 0;
long lastMsg = 0;
bool setPowerBoostKeepOn(int en) {
  Wire.beginTransmission(IP5306_ADDR);
  Wire.write(IP5306_REG_SYS_CTL0);
  if (en) {
    Wire.write(0x37);  // Set bit1: 1 enable 0 disable boost keep on
  } else {
    Wire.write(0x35);  // 0x37 is default reg value
  }
  return Wire.endTransmission() == 0;
}


//MQTT Calback
void mqttCallback(char* topic, byte* message, unsigned int len) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < len; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
  // Feel free to add more if statements to control more GPIOs with MQTT
  // If a message is received on the topic House_004/output1, you check if the message is either "true" or "false".
  // Changes the output state according to the message
  if (String(topic) == "esp/output1") {
    Serial.print("Changing output to ");
    if (messageTemp == "true") {
      Serial.println("true");
      digitalWrite(OUTPUT_1, HIGH);
    } else if (messageTemp == "false") {
      Serial.println("false");
      digitalWrite(OUTPUT_1, LOW);
    }
  } else if (String(topic) == "esp/output2") {
    Serial.print("Changing output to ");
    if (messageTemp == "true") {
      Serial.println("true");
      digitalWrite(OUTPUT_2, HIGH);
    } else if (messageTemp == "false") {
      Serial.println("false");
      digitalWrite(OUTPUT_2, LOW);
    }
  }
}
boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);
  // Connect to MQTT Broker without username and password
  //boolean status = mqtt.connect("GsmClientN");
  // Or, if you want to authenticate MQTT:
  boolean status = mqtt.connect("GsmClientN", mqttUsername, mqttPassword);
  if (status == false) {
    SerialMon.println(" fail");
    //ESP.restart(); // This can be commented to avoid the modem to keep restarting when the MQtt is not available
    return false;
  }
  SerialMon.println(" success");
  mqtt.subscribe(topicOutput1);
  mqtt.subscribe(topicOutput2);
  /*
    mqtt.subscribe(topicOutput3);
  mqtt.subscribe(topicOutput4);
    mqtt.subscribe(topicOutput5);
  mqtt.subscribe(topicOutput6);
  */
  return mqtt.connected();
}

//========= Setup method Start ============
void setup() {

  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);
  // Keep power when running from battery
  //Serial.begin(9600);
  //PMS communication
  pms_serial.begin(9600);  // For communicating with PMS7003 sensor

  /*
    // Send the command to start the sensor
    byte cmd[7] = {0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70};
     pms_serial.write(cmd, 7);
*/

  Wire.begin(I2C_SDA, I2C_SCL);

  delay(250);                 // wait for the OLED to power up
  display.begin(0x3C, true);  // Address 0x3C default
  Serial.println("OLED begun");

  display.clearDisplay();  //Clear the screen

  display.setTextColor(SH110X_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 5);
  display.print("Welcome to AIR Quality Monitoring");  //Welcome Message

  /*
  display.setTextSize(1.8);
  display.setCursor(0, 45);
  display.print("Please wait..!");
  */

  display.display();
  delay(1500);

  // Startup message
  display.clearDisplay();			 
  display.setTextSize(2);
  display.setCursor(0, 5);
  display.print("Starting \n up...!");
  display.setTextSize(2);
  display.setCursor(0, 45);
  display.print(" V 2.2.0 ");
  display.display();  //Display all of the above



  // Keep power when running from battery
  bool isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));
  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  pinMode(OUTPUT_1, OUTPUT);
  pinMode(OUTPUT_2, OUTPUT);

  SerialMon.println("Wait...");
  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(6000);
  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();
  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);
  // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3) {
    modem.simUnlock(GSM_PIN);
  }
  // You might need to change the BME280 I2C address, in our case it's 0x76
  bool status = bme.begin(0x76);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }

  /*
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  */

  // Clear the buffer.
  display.clearDisplay();
  //display.display();
  // text display tests
  //display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 5);
  display.print("Connected to APN: ");
  display.setTextSize(2);
  display.setCursor(0, 45);
  display.print(apn);
  display.display();
  SerialMon.print("Connecting to APN: ");
  delay(1500);
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    //delay(1500);
    //display.clearDisplay();
    // display PM10
    display.setTextSize(2);
    display.setCursor(0, 5);
    display.print("Failed to Connect!!!");
    //display.clearDisplay();
    display.display();  // actually display all of the above
    //  ESP.restart(); //This can be commented to avoid the modem to restart when no SIM casrd or GPRS connecttion
  }

  else {
    SerialMon.println(" OK");
  }

  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
    //delay(1500);
    display.clearDisplay();
    // display PM10
    display.setTextSize(2);
    display.setCursor(0, 5);
    display.print("GPRS connected");
    //display.clearDisplay();
    display.display();  // actually display all of the above
  }


  //SD Card mounting

  Serial.println("SD Setup start");

  display.clearDisplay();
  // display PM10
  display.setTextSize(2);
  display.setCursor(0, 5);
  display.print("SD initialization");
  //display.clearDisplay();
  display.display();  // actually display all of the above
  delay(1500);

  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card MOUNT FAILED");
    // Clear the buffer.
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 30);
    display.print("SD Card FAILED!");
    //display.clearDisplay();
    display.display();  // actually display all of the above
    delay(1500);
  } else {
    Serial.println("SD Card MOUNTED");
    Serial.println("");

    // Clear the buffer.
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 30);
    display.print("SD Card MOUNTED!");
    //display.clearDisplay();
    display.display();  // actually display all of the above
    delay(1500);

    uint32_t cardSize = SD.cardSize() / (1024 * 1024);
    String str = "SDCard Size: " + String(cardSize) + "MB";
    Serial.println(str);
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
      Serial.println("No SD card attached");
    }
    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC) {
      Serial.println("MMC");
    } else if (cardType == CARD_SD) {
      Serial.println("SDSC");
    } else if (cardType == CARD_SDHC) {
      Serial.println("SDHC");
    } else {
      Serial.println("UNKNOWN");
    }

    Serial.println("INFO: Setup complete");

    Serial.println("Initializing SD card.....");

    // Check to see if the file exists:


    String fileHeader = "Date, Time, DeviceID, Temperature, corrected_Temp, Humidity, corrected_Hum, RawPM1, CorrectedPM1, RawPM2.5, CorrectedPM2.5, RawPM10, CorrectedPM10, CO, RaweCO2, corrected_eCO2, TVOC, Pressure, Altitude";
    if (SD.exists("/data.txt")) {
      Serial.println("data.txt exists.");

      dataFile = SD.open("/data.txt", FILE_WRITE);
      // Move the write pointer to the end of the file
      dataFile.seek(dataFile.size());

      dataFile.println(fileHeader);
      delay(1000);
      dataFile.close();
    } else {
      Serial.println("data.txt doesn't exist.");
      // Create data.text if doesn't exist:
      Serial.println("Creating data.txt...");


      dataFile = SD.open("/data.txt", FILE_WRITE);
      Serial.println("data.txt file created successfully!");

      dataFile.println(fileHeader);
      delay(1000);
      dataFile.close();

      //dataFile.close();
    }

    //String fileHeader = "Date, Time, DeviceID, Temperature, Humidity, Pressure, Altitude, ";
    Serial.println("SD card initialized successfully.");

    // Clear the buffer.
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 30);
    display.print("SD card initialized successfully.!");
    //display.clearDisplay();
    display.display();  // actually display all of the above
    delay(1500);
  }

  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);
  // keepmodemup();
  //MQ7 Calibrating
  Serial.println("");  // blank new line
  Serial.println("Calibrating MQ7");

  display.clearDisplay();
  // display PM10
  display.setTextSize(2);
  display.setCursor(0, 5);
  display.print("MQ7 Calibration");
  //display.clearDisplay();
  display.display();  // actually display all of the above

  mq7.calibrate();  // calculates R0
  Serial.println("Calibration done!");

  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 30);
  display.print("Calibration done!");
  //display.clearDisplay();
  display.display();  // actually display all of the above

  //Oled power up
  delay(250);                        // wait for the OLED to power up
  display.begin(i2c_Address, true);  // Address 0x3C default
  display.setContrast(0);            // dim display
  //display.display();
  delay(1500);
  // Clear the buffer.
  display.clearDisplay();
  //CCS811 intialization
  Serial.println("CCS811 test");

  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 5);
  display.print("Checking CCS811 sensor");
  //display.clearDisplay();
  display.display();  // actually display all of the above


  //Oled power up
  delay(250);                        // wait for the OLED to power up
  display.begin(i2c_Address, true);  // Address 0x3C default
  display.setContrast(0);            // dim display
  //display.display();
  delay(1500);
  // Clear the buffer.
  display.clearDisplay();
  if (!ccs.begin()) {
    Serial.println("Failed to start CCS811 sensor! Please check your wiring.");
  }
  // Wait for the sensor to be ready
  //while (!ccs.available());
  //delay(2000);
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  /*
 //pms_serial.flush();
  
// Display sensor data on OLED screen
  display.clearDisplay();
  display.setTextSize(1);
  //display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Date/Time: ");
  display.println(dateTime);
  display.print("Temperature: ");
  display.println(bme.readTemperature());
  display.print("Pressure: ");
  display.println(bme.readPressure() / 100.0F);
  display.print("Humidity: ");
  display.println(bme.readHumidity());
  display.print("eCO2: ");
  display.println(ccs.geteCO2());
  display.print("TVOC: ");
  display.println(ccs.getTVOC());
  display.print("MQ7: ");
  display.println(analogRead(MQ7_PIN));
  display.print("PM1.0: ");
  display.println(_pm1);
  display.print("PM2.5: ");
  display.println(_pm25);
  display.print("PM10: ");
  display.println(_pm10);
  display.display();

    delay(10000);
*/
}
//========= Setup method End ============

// Function to apply correction factors to the readings
RawData applyCorrection(RawData rawReadings) {
  RawData correctedReadings;
  correctedReadings.pm1 = _pm1 * CORRECTION_FACTOR_PM1;
  correctedReadings.pm2_5 = _pm25 * CORRECTION_FACTOR_pm25;
  correctedReadings.pm10 = _pm10 * CORRECTION_FACTOR_PM10;
  correctedReadings.neweCO2 = ccs.geteCO2() * CORRECTION_FACTOR_eCO2;
  correctedReadings.temp = bme.readTemperature() * CORRECTION_FACTOR_Temp;
  correctedReadings.hum = bme.readHumidity() * CORRECTION_FACTOR_Hum;
  return correctedReadings;
}


//======Start Loop method

void loop() {

  for (int m = 1; m < 101; m++) {

    Serial.println("Starting loop method ====================!!!!!!!!!!!!");


    //keepmodemup();
    // Keep power when running from battery
    //Wire.begin(I2C_SDA, I2C_SCL);



    /*/////////////////////========> MQTT Connect Starts ......... ////////////////

  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
       // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 5);
  display.print("Connecting to server ...");
//display.clearDisplay();
  display.display(); // actually display all of the above      
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 100L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
   SerialMon.println("=== MQTT CONNECTED ===");
     // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 25);
  display.print("DATA SENT");
//display.clearDisplay();
  display.display(); // actually display all of the above      
      }
    }
    delay(1000);
   // return;
  }

 //////////////////////........ MQTT Connect ends <========///////////////*/

    long now = millis();

    if (now - lastMsg > 3000) {
      lastMsg = now;

      Serial.println("Pms value at mqtt values====== ");
      Serial.printf("PM1.0: %d PM2.5: %d PM10.0: %d\n", _pm1, _pm25, _pm10);
      /* 
//int _pm1, _pm25, _pm10;
     //pms_serial.flush(); //check if flush is working
  if (pms_serial.available() >= 32) {
    byte buffer[32];
    pms_serial.readBytes(buffer, 32);

    // Check if the data is valid
    if (buffer[0] == 0x42 && buffer[1] == 0x4D) {
      // Parse the data
       _pm1 = buffer[6] * 256 + buffer[7];
       _pm25 = buffer[8] * 256 + buffer[9];
       _pm10 = buffer[10] * 256 + buffer[11];
  }
  }
  

  */

      // Temperature in Celsius
      temperature = bme.readTemperature();
      // Uncomment the next line to set temperature in Fahrenheit
      // (and comment the previous temperature line)
      //temperature = 1.8 * bme.readTemperature() + 32; // Temperature in Fahrenheit
      // Convert the value to a char array
      char tempString[8];
      dtostrf(temperature, 1, 2, tempString);
      Serial.print("Temperature: ");
      Serial.println(tempString);
      mqtt.publish(topicTemperature, tempString);

      humidity = bme.readHumidity();
      // Convert the value to a char array
      char humString[8];
      dtostrf(humidity, 1, 2, humString);
      Serial.print("Humidity: ");
      Serial.println(humString);
      mqtt.publish(topicHumidity, humString);


      char pm1String[8];
      dtostrf(_pm1, 1, 2, pm1String);
      Serial.print("pm1: ");
      Serial.println(pm1String);
      mqtt.publish(topicPm1, pm1String);
      //pm1 = _pm1;

      // Convert the value to a char array
      char pm2_5String[8];
      dtostrf(_pm25, 1, 2, pm2_5String);
      Serial.print("pm2_5: ");
      Serial.println(pm2_5String);
      mqtt.publish(topicPm2_5, pm2_5String);
      // pm2_5 = _pm25;

      // Convert the value to a char array
      char pm10String[8];
      dtostrf(_pm10, 1, 2, pm10String);
      Serial.print("pm10: ");
      Serial.println(pm10String);
      mqtt.publish(topicPm10, pm10String);
      //pm10 = _pm10;


      // Convert the value to a char array
      char coString[8];
      float coSensorValue = mq7.readPpm();
      dtostrf(coSensorValue, 1, 2, coString);
      Serial.print("CO: ");
      Serial.println(coString);
      mqtt.publish(topicCo, coString);

      //Convert the value to a char array
      char eCo2String[8];
      float eCo2SensorValue = ccs.geteCO2();
      dtostrf(eCo2SensorValue, 1, 2, eCo2String);
      Serial.print("TVOC: ");
      Serial.println(eCo2String);
      mqtt.publish(topiceCo2, eCo2String);
      //eCoSensorValue = ccs.geteCO2();

      // Convert the value to a char array
      char tvocString[8];
      float tvocSensorValue = ccs.getTVOC();
      dtostrf(tvocSensorValue, 1, 2, tvocString);
      Serial.print("CO: ");
      Serial.println(tvocString);
      mqtt.publish(topicTvoc, tvocString);

      delay(1000);


      // Get time from GSM module
      String dateTime = modem.getGSMDateTime(DATE_TIME);



      delay(1000);  // Wait for a second before printing again

      Serial.println("Current time from GSM module: " + dateTime);
      Serial.println();
      // Get date from GSM module
      String date = modem.getGSMDateTime(DATE_DATE);
      Serial.println("Current date from GSM module: " + date);


      // Read the raw data from PMS7003
      RawData rawReadings = readRawData();

      // Apply correction factors to the readings
      RawData correctedReadings = applyCorrection(rawReadings);

      // Create JSON object
      StaticJsonDocument<512> doc;

      // Add data to the JSON object
      doc["Date"] = date;
      doc["Time"] = dateTime;
      doc["DeviceID"] = deviceID;
      doc["Temperature"] = bme.readTemperature();
      doc["Correct_Temperature"] = correctedReadings.temp;
      doc["Humidity"] = bme.readHumidity();
      doc["Correct_Humidity"] = correctedReadings.hum;
      doc["eCO2"] = ccs.geteCO2();
      doc["Correct_eCO2"] = correctedReadings.neweCO2;
      doc["TVOC"] = ccs.getTVOC();
      doc["CO"] = mq7.readPpm();
      doc["PM1"] = _pm1;
      doc["PM2.5"] = _pm25;
      doc["PM10"] = _pm10;
      doc["CorrectPM1"] = correctedReadings.pm1;
      doc["CorrectPM2.5"] = correctedReadings.pm2_5;
      doc["CorrectPM10"] = correctedReadings.pm10;
      doc["Pressure"] = bme.readPressure() / 100.0F;  // Convert to hPa
      doc["Altitude"] = bme.readAltitude(SEALEVELPRESSURE_HPA);



      // Convert the value to a char array
      // Serialize JSON object to a string
      String jsonStr;
      serializeJson(doc, jsonStr);
      // Publish JSON string to MQTT topic

      mqtt.publish(topicData, jsonStr.c_str());
      /*
        // Convert the value to a char array
    char pressureString[8];
    float pressureSensorValue = bme.readPressure() / 100.0F;
    dtostrf(pressureSensorValue, 1, 2, pressureString);
    Serial.print("Pressure: ");
    Serial.println(pressureString);
    mqtt.publish(topicPressure, pressureString);

            // Convert the value to a char array
    char altitudeString[8];
    float altitudeSensorValue = bme.readAltitude(SEALEVELPRESSURE_HPA);
    dtostrf(altitudeSensorValue, 1, 2, altitudeString);
    Serial.print("Altitude: ");
    Serial.println(altitudeString);
    mqtt.publish(topicAltitude, altitudeString);
*/
    }

    mqtt.loop();

    //delay(1500);
    displayTempAndHum();
    displayPressAndAlt();
    readPmsSensor();
    displayCorrectedData();
    printPmsAndCOSensorsValue();
    display_e_CO2_TVOC();
    highPm25();
    //ESP.restart();
    //pms_serial.flush();
    displayDate_makeJsonString();
    //closeFile();
    displayDeviceID();


    Serial.println("Count= " + String(m));
    // Clear the buffer.
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 5);
    display.print("Count= ");
    display.print(m);
    //display.clearDisplay();
    display.display();  // actually display all of the above
    delay(1000);

    if (m == 100) {
      ESP.restart();
      //  modem.restart();
      //  delay(1000);
    }
  }
}

//////////////=========> End of loop method




void printDigits(int digits) {
  // Add leading 0 if needed
  if (digits < 10) {
    Serial.print("0");
  }
  Serial.print(digits);
}							  

void closeFile() {
  // Close the data file
  dataFile.close();
  SD.end();
  SPI.end();
}


void displayDate_makeJsonString() {
  // Create JSON object
  // Get time from GSM module
  String dateTime = modem.getGSMDateTime(DATE_TIME);
  Serial.println("Current time from GSM module: " + dateTime);
  Serial.println();

  // Get date from GSM module
  String date = modem.getGSMDateTime(DATE_DATE);
  // Serial.println("Current date from GSM module: " + date);

  Serial.println("Pms value at make json ==================");
  Serial.printf("PM1.0: %d PM2.5: %d PM10.0: %d\n", _pm1, _pm25, _pm10);


  // Read the raw data from PMS7003
  RawData rawReadings = readRawData();

  // Apply correction factors to the readings
  RawData correctedReadings = applyCorrection(rawReadings);

  // Display current date and time
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(2, 0);
  display.print("Date:");
  display.setTextSize(2);
  display.setCursor(0, 10);
  display.print(date);

  display.setTextSize(1);
  display.setCursor(0, 35);
  display.print("Time:");
  display.setTextSize(2);
  display.setCursor(0, 45);
  display.print(dateTime);
  display.display();

  delay(2000);  // Update time every 5 seconds
                /*
//PMS communication
  pms_serial.begin(9600);  // For communicating with PMS7003 sensor
  pms_serial.flush();

  // Read PMS7003 sensor data
 
      // Send the command to start the sensor
  byte cmd[7] = {0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70};
  pms_serial.write(cmd, 7);


  delay(1000);
//int _pm1, _pm25, _pm10;
     //pms_serial.flush(); //check if flush is working
  if (pms_serial.available() >= 32) {
    byte buffer[32];
    pms_serial.readBytes(buffer, 32);

    // Check if the data is valid
    if (buffer[0] == 0x42 && buffer[1] == 0x4D) {
      // Parse the data
       _pm1 = buffer[6] * 256 + buffer[7];
       _pm25 = buffer[8] * 256 + buffer[9];
       _pm10 = buffer[10] * 256 + buffer[11];
  }
  
  }
*/

  StaticJsonDocument<512> doc;

  // Add data to the JSON object
  doc["Date"] = date;
  doc["Time"] = dateTime;
  doc["DeviceID"] = deviceID;
  doc["Temperature"] = bme.readTemperature();
  doc["Correct_Temperature"] = correctedReadings.temp;
  doc["Humidity"] = bme.readHumidity();
  doc["Correct_Humidity"] = correctedReadings.hum;
  doc["eCO2"] = ccs.geteCO2();
  doc["Correct_eCO2"] = correctedReadings.neweCO2;
  doc["TVOC"] = ccs.getTVOC();
  doc["CO"] = mq7.readPpm();
  doc["PM1"] = _pm1;
  doc["PM2.5"] = _pm25;
  doc["PM10"] = _pm10;
  doc["CorrectPM1"] = correctedReadings.pm1;
  doc["CorrectPM2.5"] = correctedReadings.pm2_5;
  doc["CorrectPM10"] = correctedReadings.pm10;
  doc["Pressure"] = bme.readPressure() / 100.0F;  // Convert to hPa
  doc["Altitude"] = bme.readAltitude(SEALEVELPRESSURE_HPA);

  // float eCO2_data=+ccs.geteCO2();

  // Convert the value to a char array
  // Serialize JSON object to a string
  String jsonStr;
  serializeJson(doc, jsonStr);

  //String sensorData = jsonStr;

  String sensorData = date + ", " + dateTime + ", " + deviceID + ", " + bme.readTemperature() + ", " + correctedReadings.temp + ", " + bme.readHumidity() + ", " + correctedReadings.hum + ", " + _pm1 + ", " + correctedReadings.pm1+ ", " + _pm25 + ", " + correctedReadings.pm2_5 + ", " + _pm10  + ", " + correctedReadings.pm10 + ", " + mq7.readPpm() + ", " + ccs.geteCO2() + ", " + correctedReadings.neweCO2 + ", " + ccs.getTVOC() + ", " + bme.readPressure() / 100.0F + ", " + bme.readAltitude(SEALEVELPRESSURE_HPA);

  Serial.println("Logging data...");

  //dataFile = SD.open("/");
  // Open the data file in write mode
  dataFile = SD.open("/data.txt", FILE_WRITE);

  if (dataFile) {
    // Move the write pointer to the end of the file
    dataFile.seek(dataFile.size());

    // Log data to the SD card
    dataFile.println(sensorData);

    Serial.println("Data appended to file.");

    // Print data to serial monitor
    Serial.println(sensorData);

    dataFile.close();
  } else {
    Serial.println("Error opening file for appending.");

    // Display current date and time
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 5);
    display.print("SD card Error!");


    display.setTextSize(2);
    display.setCursor(0, 40);
    display.print("Data save fail!");
    display.display();

    delay(2000);  // Update time every 5 seconds
  }
  // Wait a little bit between measurements
  delay(1000);  // Adjust delay as needed
}



//Display functions
void ledAlert() {
  sensorValue = mq7.readPpm();
  Serial.println(sensorValue);
  //Serial.println(mq7.readAnalog());
  if (sensorValue > coThreshold) {
    Serial.println("High CO gas concentration detected!");
    digitalWrite(13, HIGH);  // turn on the LED indicator
  } else {
    Serial.println("CO Gas concentration within safe limits.");
    digitalWrite(13, LOW);  // turn off the LED indicator
  }
}

//Display functions
void highPm25() {
  float pm2_5SensorValue = _pm25;
  Serial.println(pm2_5SensorValue);
  //Serial.println(mq7.readAnalog());
  if (pm2_5SensorValue > pm25CuttOff) {
    Serial.println("High PM25 gas concentration detected! Need recalibrate");
    //digitalWrite(13, HIGH);  // turn on the LED indicator
    //ESP.restart();

  } else {
    Serial.println("PM25 concentration within safe limits.");
    //digitalWrite(13, LOW);  // turn off the LED indicator
  }
}

void readPmsSensor() {
  int checksum = 0;
  unsigned char pms[32] = {
    0,
  };
  /**
   * Search preamble for Packet
   * Solve trouble caused by delay function
   */
  while (pms_serial.available() && pms_serial.read() != PMS7003_PREAMBLE_1 && pms_serial.peek() != PMS7003_PREAMBLE_2) {
  }
  if (pms_serial.available() >= PMS7003_DATA_LENGTH) {
    pms[0] = PMS7003_PREAMBLE_1;
    checksum += pms[0];
    for (int j = 1; j < 32; j++) {
      pms[j] = pms_serial.read();
      if (j < 30)
        checksum += pms[j];
    }
    pms_serial.flush();
    if (pms[30] != (unsigned char)(checksum >> 8)
        || pms[31] != (unsigned char)(checksum)) {
      Serial.println("Checksum error");
      return;
    }
    if (pms[0] != 0x42 || pms[1] != 0x4d) {
      Serial.println("Packet error");
      return;
    }
    _pm1 = makeWord(pms[10], pms[11]);
    _pm25 = makeWord(pms[12], pms[13]);
    _pm10 = makeWord(pms[14], pms[15]);
  }

  //int _pm1, _pm25, _pm10;
  // Print the data
  Serial.println("========>PM1<readPms===========");
  Serial.print("PM1.0: ");
  Serial.print(_pm1);
  Serial.println(" ug/m3");
  Serial.print("PM2.5: ");
  Serial.print(_pm25);
  Serial.println(" ug/m3");
  Serial.print("PM10: ");
  Serial.print(_pm10);
  Serial.println(" ug/m3");
  // Serial.println("=====================");
  // Serial.print("\n");


  //delay(1500);
}


//Display functions
void printPmsAndCOSensorsValue() {

  // Read the raw data from PMS7003
  RawData rawReadings = readRawData();

  // Apply correction factors to the readings
  RawData correctedReadings = applyCorrection(rawReadings);

  Serial.printf("PM1.0: %d PM2.5: %d PM10.0: %d\n", _pm1, _pm25, _pm10);
  //int _pm1, _pm25, _pm10;
  // Print the data
  Serial.println("========>PM2<Co..===========");
  Serial.print("PM1.0: ");
  Serial.print(correctedReadings.pm1);
  Serial.println(" ug/m3");
  Serial.print("PM2.5: ");
  Serial.print(correctedReadings.pm2_5);
  Serial.println(" ug/m3");
  Serial.print("PM10: ");
  Serial.print(correctedReadings.pm10);
  Serial.println(" ug/m3");
  // Serial.println("=====================");
  // Serial.print("\n");


  // delay(1000);

  display.clearDisplay();
  // display PM1.0
  display.setTextSize(1);
  display.setCursor(2, 0);
  display.print("PM1.0:  ");
  display.setTextSize(2);
  display.setCursor(0, 10);
  display.print(correctedReadings.pm1);
  display.setTextSize(2);
  display.print("  ug/m3");


  //display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 35);
  display.print("PM2.5: ");
  display.setTextSize(2);
  display.setCursor(0, 45);
  display.print(correctedReadings.pm2_5);
  display.print("  ug/m3");
  display.display();
  delay(1500);

  display.clearDisplay();
  // display PM10
  display.setTextSize(1);
  display.setCursor(0, 5);
  display.print("PM10: ");
  display.setTextSize(2);
  display.setCursor(0, 15);
  display.print(correctedReadings.pm10);
  display.print("  ug/m3");


  // display CO
  Serial.print("CO= ");
  Serial.print(mq7.readPpm());
  Serial.println(" PPM");
  //Serial.println("");   // blank new line
  //delay(1000);
  //display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 35);
  display.print("CO: ");
  display.setTextSize(2);
  display.setCursor(0, 45);
  display.print(String(mq7.readPpm()));
  display.print("  ppm");
  display.display();
  delay(1500);
}

/*
void readPmsSensor() {
  int checksum = 0;
  unsigned char pms[32] = {
    0,
  };
  
   // Search preamble for Packet
   // Solve trouble caused by delay function
   
  while (pms_serial.available() && pms_serial.read() != PMS7003_PREAMBLE_1 && pms_serial.peek() != PMS7003_PREAMBLE_2) {
  }
  if (pms_serial.available() >= PMS7003_DATA_LENGTH) {
    pms[0] = PMS7003_PREAMBLE_1;
    checksum += pms[0];
    for (int j = 1; j < 32; j++) {
      pms[j] = pms_serial.read();
      if (j < 30)
        checksum += pms[j];
    }
    pms_serial.flush();
    if (pms[30] != (unsigned char)(checksum >> 8)
        || pms[31] != (unsigned char)(checksum)) {
      Serial.println("Checksum error");
      return;
    }
    if (pms[0] != 0x42 || pms[1] != 0x4d) {
      Serial.println("Packet error");
      return;
    }
    _pm1 = makeWord(pms[10], pms[11]);
    _pm25 = makeWord(pms[12], pms[13]);
    _pm10 = makeWord(pms[14], pms[15]);
  }
}
*/


//Display functions
void displayTempAndHum() {

  // Read the raw data from PMS7003
  RawData rawReadings = readRawData();

  // Apply correction factors to the readings
  RawData correctedReadings = applyCorrection(rawReadings);

  /*

correctedReadings.pm1
correctedReadings.pm2_5
correctedReadings.pm10
correctedReadings.neweCO2
correctedReadings.temp
correctedReadings.hum

*/

  Serial.print("Temperature = ");
  Serial.print(correctedReadings.temp);
  Serial.println(" *C");
  Serial.print("Humidity = ");
  Serial.print(correctedReadings.hum);
  Serial.println(" %");
  Serial.println();
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");
  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
  display.clearDisplay();
  // display temperature
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Temperature: ");
  display.setTextSize(2);
  display.setCursor(0, 10);
  display.print(correctedReadings.temp);
  display.print(" ");
  display.setTextSize(1);
  display.cp437(true);
  display.write(167);
  display.setTextSize(2);
  display.print(" *C");
  // display humidity
  display.setTextSize(1);
  display.setCursor(0, 35);
  display.print("Humidity: ");
  display.setTextSize(2);
  display.setCursor(0, 45);
  display.print(correctedReadings.hum);
  display.print(" %");
  display.display();
  delay(1500);
}

void displayPressAndAlt() {
  display.clearDisplay();
  // display Pressure
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Pressure: ");
  display.setTextSize(2);
  display.setCursor(0, 10);
  display.print(String(bme.readPressure() / 100.0F));
  display.setTextSize(2);
  display.print(" hPa");
  // display Altitude
  display.setTextSize(1);
  display.setCursor(0, 35);
  display.print("Approx. Altitude: ");
  display.setTextSize(2);
  display.setCursor(0, 45);
  display.print(String(bme.readAltitude(SEALEVELPRESSURE_HPA)));
  display.print(" m");
  display.display();
  delay(1500);
}
void displayCorrectedData() {

  // Read the raw data from PMS7003
  RawData rawReadings = readRawData();

  // Apply correction factors to the readings
  RawData correctedReadings = applyCorrection(rawReadings);

  /*
correctedReadings.pm1
correctedReadings.pm2_5
correctedReadings.pm10
correctedReadings.neweCO2
correctedReadings.temp
correctedReadings.hum
*/


  // Print raw and corrected readings
  Serial.print("Raw Temperature: ");
  Serial.print(bme.readTemperature());
  Serial.print(" µg/m³, Corrected Temperature: ");
  Serial.print(correctedReadings.temp);
  Serial.println(" *C");

  // Print raw and corrected readings
  Serial.print("Raw Humidity: ");
  Serial.print(bme.readHumidity());
  Serial.print(" µg/m³, Corrected Humidity: ");
  Serial.print(correctedReadings.hum);
  Serial.println(" %");


  // Print raw and corrected readings
  Serial.print("Raw PM1.0: ");
  Serial.print(_pm1);
  Serial.print(" µg/m³, Corrected PM1.0: ");
  Serial.print(correctedReadings.pm1);
  Serial.println(" µg/m³");


  Serial.print("Raw PM2.5: ");
  Serial.print(_pm25);
  Serial.print(" µg/m³, Corrected PM2.5: ");
  Serial.print(correctedReadings.pm2_5);
  Serial.println(" µg/m³");


  Serial.print("Raw PM10: ");
  Serial.print(_pm10);
  Serial.print(" µg/m³, Corrected PM10: ");
  Serial.print(correctedReadings.pm10);
  Serial.println(" µg/m³");


  // Print raw and corrected readings
  Serial.print("Raw eCO2: ");
  Serial.print(ccs.geteCO2());
  Serial.print(" ppm, Corrected eCO2: ");
  Serial.print(correctedReadings.pm1);
  Serial.println(" ppm");


  /*
 display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(2, 0);
  display.print("New Temperature:  ");
  display.setTextSize(2);
  display.setCursor(0, 10);
  display.print(correctedReadings.temp);
  display.setTextSize(2);
  display.print(" *C");


  //display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 35);
  display.print("New Humidity: ");
  display.setTextSize(2);
  display.setCursor(0, 45);
  display.print(correctedReadings.hum);
  display.print(" %");
  display.display();
  delay(1500);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(2, 0);
  display.print("New PM1.0:  ");
  display.setTextSize(2);
  display.setCursor(0, 10);
  display.print(correctedReadings.pm1);
  display.setTextSize(2);
  display.print("  ug/m3");


  //display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 35);
  display.print("New PM2.5: ");
  display.setTextSize(2);
  display.setCursor(0, 45);
  display.print(correctedReadings.pm2_5);
  display.print("  ug/m3");
  display.display();
  delay(1500);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(2, 0);
  display.print("New PM10:  ");
  display.setTextSize(2);
  display.setCursor(0, 10);
  display.print(correctedReadings.pm10);
  display.setTextSize(2);
  display.print("  ug/m3");


  //display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 35);
  display.print("New eCO2: ");
  display.setTextSize(2);
  display.setCursor(0, 45);
  display.print(correctedReadings.neweCO2);
  display.print("  ug/m3");
  display.display();
  delay(1500);

 */

  // Delay for a while before the next reading
  // delay(1000);
}


// Function to read data from the PMS7003 sensor
RawData readRawData() {

  RawData data;
  if (pms_serial.available()) {
    // Parse the data
    data.pm1 = _pm1;
    data.pm2_5 = _pm25;
    data.pm10 = _pm10;
  }

  data.neweCO2 = ccs.geteCO2();
  data.temp = bme.readTemperature();
  data.hum = bme.readHumidity();

  return data;
}


void display_e_CO2_TVOC() {
  // Read the raw data from PMS7003
  RawData rawReadings = readRawData();

  // Apply correction factors to the readings
  RawData correctedReadings = applyCorrection(rawReadings);

  /*
correctedReadings.pm1
correctedReadings.pm2_5
correctedReadings.pm10
correctedReadings.neweCO2
correctedReadings.temp
correctedReadings.hum
*/

  if (ccs.available()) {
    if (!ccs.readData()) {
      Serial.print("e-CO2: ");
      Serial.print(correctedReadings.neweCO2);
      Serial.print("ppm, TVOC: ");
      Serial.println(ccs.getTVOC());
      display.clearDisplay();
      // display eCO2
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.print("e-CO2: ");
      display.setTextSize(2);
      display.setCursor(0, 10);
      display.print(correctedReadings.neweCO2);
      display.setTextSize(2);
      display.print(" ppm");
      // display Altitude
      display.setTextSize(1);
      display.setCursor(0, 35);
      display.print("TVOC: ");
      display.setTextSize(2);
      display.setCursor(0, 45);
      display.print(String(ccs.getTVOC()));
      display.print("  bpm");
      display.display();
      delay(1500);
    }
  }
}

void displayDeviceID() {
  display.clearDisplay();
  // display PM10
  display.setTextSize(1.5);
  display.setCursor(0, 5);
  display.print("Device ID: ");
  display.setTextSize(2);
  display.setCursor(0, 25);
  display.print("House_004");
  //display.clearDisplay();
  display.display();  // actually display all of the above
  delay(1500);
}
