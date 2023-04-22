#include <dummy.h>
#include <dummy.h>

/////////////////////////////////////////////////////// SIM800L CONFIGURATION /////////////////////////////////////////////////////

// Your GPRS credentials (leave empty, if not needed)
// #define BLYNK_TEMPLATE_ID "TMPLt1eNAJqq"
// #define BLYNK_TEMPLATE_NAME "Quickstart Template"
// #define BLYNK_AUTH_TOKEN "7EWdSVMlXpNR96KCqlW9aB0w4JobWvtW"

char apn[] = "internet";
const char user[] = "";
const char pass[] = "";

// SIM card PIN (leave empty, if not defined)
const char simPIN[]   = "";


/////////////////////////////////////////////////////// IOT CONFIGURATION /////////////////////////////////////////////////////
const char serverBlynk[] = "blynk.cloud";
const int  portBlynk = 80;
const char auth[] = "9kLBT8G007HKUPI-eZvJOZ_ziUDL7HdX";  //su info@saigi.it

//Your Domain name with URL path or IP address with path
/////////////////////////////////////////////////////// TTGO T-CALL PINS /////////////////////////////////////////////////////

#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22

// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to SIM800 module)
#define SerialAT Serial1
// Set serial for UART Communication (to Ultrasonic sensor)
HardwareSerial SerialPort(2); // use UART2

//#define Serial Serial2

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
//#define debug

/////////////////////////////////////////////////////// LIBRARIES /////////////////////////////////////////////////////
#include <Wire.h>
#include <TinyGsmClient.h>
#include <SoftwareSerial.h>
#include <ArduinoHttpClient.h>
#include <HardwareSerial.h>
#include <HTTPClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif



/////////////////////////////////////////////////////// ULTRASONIC SENSOR PINS /////////////////////////////////////////////////////

//byte byteRead;
#define BAUD 9600
#define txPin 33
#define rxPin 32
// SoftwareSerial sonarSerial(rxPin, txPin);

// bool receiving;
// byte buff[3];
// int index_STS = 0;
// uint32_t communicationStarted;

/////////////////////////////////////////////////////// LOGICAL OPERATION VARIABLES /////////////////////////////////////////////////////


// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);


unsigned long previousMillis1 = 0;  // will store last time LED was updated
unsigned long previousMillis2 = 0;  // will store last time LED was updated
unsigned long previousMillis3 = 0;  // will store last time LED was updated
unsigned long previousMillis4 = 0;  // will store last time LED was updated

int Sleep_Button_SS = 0;

// TinyGSM Client for Internet connection
TinyGsmClient client(modem);


#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */ // DONT CHANGE THIS
double TIME_TO_SLEEP = 300;        /* Time ESP32 will go to sleep (in seconds) */ // DEEP SLEEP TIME CURRENTLY SET TO 5 Mins

int BLYNK_Upload_Time = 1000; // BLYNK Upload Time Interval, Currently set to 05 seconds
int Switch_OFF_Deep_Sleep_Activation_Time = 180000; // TIME AFTER WHICH DEEP SLEEP WILL BE TRIGGERED CURRENTLY SET TO 23Mins

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00


#define VpinUltrasonic          "v1"


RTC_DATA_ATTR unsigned long TimeToSleep = 61 * 60;

bool setPowerBoostKeepOn(int en) {
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en) {
    I2CPower.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    I2CPower.write(0x35); // 0x37 is default reg value
  }
  return I2CPower.endTransmission() == 0;
}


void setup() {
  // Set serial monitor debugging window baud rate to 115200
  SerialMon.begin(115200);

  // Start I2C communication
  I2CPower.begin(I2C_SDA, I2C_SCL, 400000);

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

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(1000);

  // Set UART baud rate and UART pins
  SerialPort.begin(BAUD, SERIAL_8N1, rxPin, txPin);

  // Restart SIM800 module, it takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // use modem.init() if you don't need the complete restart

  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
    modem.simUnlock(simPIN);
  }
  
  Modem_On();

  // Configure the wake up source as timer wake up
  //esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}


void loop() {

  unsigned long currentMillis = millis();


  if (currentMillis - previousMillis1 >= 1000) {
    // save the last time you blinked the LED
    previousMillis1 = currentMillis;
    }

  if (Sleep_Button_SS == 0) {
    unsigned long currentMillis2 = millis();

    if (currentMillis2 - previousMillis2 >= BLYNK_Upload_Time) {
      // save the last time you blinked the LED
      previousMillis2 = currentMillis2;
      sendBlynk();
      Sleep_Button_SS = (GetFromBlynk("v0")).toInt();
      TIME_TO_SLEEP = (GetFromBlynk("v2")).toDouble();
      SerialMon.println("Sleep Button Status: " + String(Sleep_Button_SS));
      SerialMon.println("Sleep Time (s): " + String(TIME_TO_SLEEP));
    }
  } else if (Sleep_Button_SS == 1) {
    unsigned long currentMillis2 = millis();

    if (currentMillis2 - previousMillis2 >= BLYNK_Upload_Time) {
      // save the last time you blinked the LED
      previousMillis2 = currentMillis2;
      sendBlynk();
      Sleep_Button_SS = (GetFromBlynk("v0")).toInt();
      TIME_TO_SLEEP = (GetFromBlynk("v2")).toDouble();
      SerialMon.println("Sleep Button Status: " + String(Sleep_Button_SS));
      SerialMon.println("Sleep Time (s): " + String(TIME_TO_SLEEP));
    }

    unsigned long currentMillis3 = millis();

    if (currentMillis3 - previousMillis3 >= Switch_OFF_Deep_Sleep_Activation_Time) {
      // save the last time you blinked the LED
      previousMillis3 = currentMillis3;
      sendBlynk();
      modem.gprsDisconnect();
      SerialMon.println(F("GPRS disconnected"));
      SerialMon.println("Going to sleep ");
      yield();
      delay(2000);


      /*
        First we configure the wake up source
        We set our ESP32 to wake up every 5 seconds
      */
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
                     " Seconds");

      /*
        Next we decide what all peripherals to shut down/keep on
        By default, ESP32 will automatically power down the peripherals
        not needed by the wakeup source, but if you want to be a poweruser
        this is for you. Read in detail at the API docs
        http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
        Left the line commented as an example of how to configure peripherals.
        The line below turns off all RTC peripherals in deep sleep.
      */
      //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
      //Serial.println("Configured all RTC Peripherals to be powered down in sleep");

      /*
        Now that we have setup a wake cause and if needed setup the
        peripherals state in deep sleep, we can now start going to
        deep sleep.
        In the case that no wake up sources were provided but deep
        sleep was started, it will sleep forever unless hardware
        reset occurs.
      */
      Serial.println("Going to sleep now");
      delay(1000);
      Serial.flush();
      esp_deep_sleep_start();
      Serial.println("This will never be printed");
    }
  }
}


void Modem_On() {
  SerialMon.println("\nWaiting for network...");
  while (!modem.waitForNetwork()) {
    delay(500);
   // ESP.restart();
  //return;
  }
  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }

  SerialMon.print("GPRS status: ");
  if (modem.isGprsConnected()) {
    SerialMon.println("connected");
  } else {
    SerialMon.println("Connecting to: " + String(apn));
    while (!modem.gprsConnect(apn, user, pass)) {
      SerialMon.println(" fail");
     // delay(1000);
     // ESP.restart();
    }
  }
  SerialMon.println("OK");
}

void sendBlynk() {
  SendToBlynkValue(VpinUltrasonic, get_fuel_level());
}


String get_fuel_level(){
  int min_string_len = 36;
  while (SerialPort.read() >= 0){
    SerialMon.println("dropped byte");
    continue;
  }
  while(1){
   String rx = SerialPort.readStringUntil('#');
   if (rx.length()== 0){
    SerialMon.println("No Data Available!");
     delay(1000);
     continue;
   }
   SerialMon.println("Sensor data output, len :" + String(rx.length()) + rx);
  
   if(rx.length() != min_string_len){
      continue;
   }
   if(!rx.startsWith("*XD")) {
      continue;
   }
   String value_str = rx.substring(17,21);
   double value = value_str.toDouble();
   value = value/100;
   SerialMon.print("Ultrasonic_Sensor_Value(cm) = ");
   SerialMon.println(String(value,2));
   if(value == 0){
      continue;
   }
  return String(value,2);
  }
}

String GetFromBlynk(String Vpin) {
  HttpClient httpBlynk(client, serverBlynk, portBlynk);
  String resourceBlynk = "";
  String body = "";
  ///external/api/update?token={token}&pin=V1&value=lon&value=lat
  resourceBlynk = String("/external/api/get?token=") + auth + String("&") + String(Vpin);
  //SerialMon.println(resourceBlynk);
  // SerialMon.println(F("Connecting to "));
  // SerialMon.print(resourceBlynk);

 // SerialMon.println(F("Performing HTTP GET request... "));
  int err = httpBlynk.get(resourceBlynk);
  if (err != 0) {
    SerialMon.println(F("Failed to connect"));
    delay(1000);
  }
  else {
    int status = httpBlynk.responseStatusCode();
    SerialMon.print(F("Response status code: "));
    SerialMon.println(status);
    if (status < 1) {
      delay(1000);
      httpBlynk.stop();
      SerialMon.print(F("Http Error"));
    }
    else if (status == 200) {
      //SerialMon.println(F("Response Headers:"));
      int timoutTimer = 20000;
      long startTimer = millis();
      while (httpBlynk.headerAvailable()) {
        String headerName = httpBlynk.readHeaderName();
        String headerValue = httpBlynk.readHeaderValue();
       // SerialMon.println("    " + headerName + " : " + headerValue);
        if ((millis() - startTimer) > timoutTimer) {
          break;
        }
      }

      int length = httpBlynk.contentLength();
      if (length >= 0) {
        //SerialMon.print(F("Content length is: "));
        //SerialMon.println(length);
      }
      if (httpBlynk.isResponseChunked()) {
        SerialMon.println(F("The response is chunked"));
      }

      body = httpBlynk.responseBody();
      // SerialMon.println(F("Response:"));
      // SerialMon.println(body);

      // SerialMon.print(F("Body length is: "));
      // SerialMon.println(body.length());

      // Shutdown

     /* httpBlynk.stop();
      SerialMon.println(F("Server disconnected"));

      SerialMon.println(body);
      SerialMon.println((body.length() == 0 ? "OK" : "FAIL"));*/

    }
  }
  // SerialMon.println(F("Final Response:"));
  // SerialMon.print(body);
  return body;
}


void SendToBlynkValue(String Vpin, String val) {

  HttpClient httpBlynk(client, serverBlynk, portBlynk);
  String resourceBlynk = "";
  resourceBlynk = String("/external/api/update?token=") + auth + String("&") + String(Vpin) + "=" + val;
  //SerialMon.println(resourceBlynk);
  // SerialMon.println(F("Connecting to "));
  // SerialMon.print(resourceBlynk);

  //SerialMon.println(F("Performing HTTP GET request... "));
  int err = httpBlynk.get(resourceBlynk);
  if (err != 0) {
    SerialMon.println(F("failed to connect"));
    delay(1000);
  }
  else {
    int status = httpBlynk.responseStatusCode();
    SerialMon.print(F("Response status code: "));
    SerialMon.println(status);
    if (status < 1) {
      delay(1000);
      httpBlynk.stop();
      SerialMon.print(F("Http Error"));
    }
    else if (status == 200) {
      //SerialMon.println(F("Response Headers:"));
      int timoutTimer = 20000;
      long startTimer = millis();
      while (httpBlynk.headerAvailable()) {
        String headerName = httpBlynk.readHeaderName();
        String headerValue = httpBlynk.readHeaderValue();
        //SerialMon.println("    " + headerName + " : " + headerValue);
        if ((millis() - startTimer) > timoutTimer) {
          break;
        }
      }

      int length = httpBlynk.contentLength();
      if (length >= 0) {
        // SerialMon.print(F("Content length is: "));
        // SerialMon.println(length);
      }
      if (httpBlynk.isResponseChunked()) {
        SerialMon.println(F("The response is chunked"));
      }

      String body = httpBlynk.responseBody();
      // SerialMon.println(F("Response:"));
      // SerialMon.println(body);

      // SerialMon.print(F("Body length is: "));
      // SerialMon.println(body.length());

      // Shutdown
/*
      httpBlynk.stop();
      SerialMon.println(F("Server disconnected"));

      SerialMon.println(body);
      SerialMon.println((body.length() == 0 ? "OK" : "FAIL"));*/
    }
  }
}
