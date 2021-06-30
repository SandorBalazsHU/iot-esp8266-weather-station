/**
 * ESP8266 Weather station
 * Waterproof, solar charged weather station with battery based on ESP8266, BME280 and DS3231 RTC 
 * Data storage: SD
 * 
 * Created by: Sándor Balázs
 * sandorbalazs9402@gmail.com
 * AX400
 * 
 * DEBUG MODE: SERIAL 115200
 * 
 * Features:
 * 
 */
#include <Wire.h>
#include "RTClib.h"
#include <EEPROM.h>
#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266WebServer.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//Sensor
Adafruit_BME280 bme;
//Senzor status
bool bme280Status = false;
//Pressure init
#define SEALEVELPRESSURE_HPA (1013.25)
//Senzor variables
float temperature, humidity, pressure, altitude, wifiStrength;

//RTC init
RTC_DS3231 rtc;
const long utcOffsetInSeconds = 3600;
//char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
bool rtcStatus = false;

//EEPROM
//last address = 4095;

//WIFI datas
const char* ssid = "SB";
const char* password = "117aSd558qWe94*";
const char* APssid = "SBWeather";
const char* APpassword = "11755894";
//WIFI Connection trying overtime
#define OVERTIME 10
//WIFI timeout counter
int timeout = 0;
//Modul is online?
boolean isOnline = false;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

//Webserver start
ESP8266WebServer server(80);              
 
void setup() {
  //Serial start for DEBUG
  Serial.begin(115200);
  delay(100);

  //LED INIT
  pinMode(LED_BUILTIN, OUTPUT);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  
  //Setup start
  Serial.println("");
  Serial.println("ESP8266 - Start");
  
  //Sensor init
  if(!(bme280Status = bme.begin(0x76))){
    Serial.println("BME280 Init ERROR");
  }else{
    Serial.println("BME280 Init OK");
  }

  //Wifi setup Station mode
  WiFi.mode(WIFI_STA);

  //connect to your local wi-fi network
  WiFi.begin(ssid, password);

  //check wifi is connected 
  Serial.println("---");
  while (!WiFi.isConnected() && timeout <= OVERTIME) {
    Serial.print("WIFI Connencting to: ");
    Serial.println(WiFi.SSID());
    Serial.print("WIFI status: ");
    Serial.println(wifiStatuscode(WiFi.status()));
    delay(1000);
    timeout++;
  }
  Serial.println("---");
  
  //Status update
  isOnline = WiFi.isConnected();

  if(isOnline){
    Serial.println("WIFI Connection successfull");
    Serial.print("WIFI status: ");
    Serial.println(wifiStatuscode(WiFi.status()));
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
  }else{
    Serial.println("WIFI Connection ERROR");
    Serial.print("WIFI status: ");
    Serial.println(wifiStatuscode(WiFi.status()));
  }
  
  //If not connected create AP
  if(timeout > OVERTIME){
    WiFi.disconnect();
    Serial.println("Overtime, start AP");
    //Wifi setup Station mode
    WiFi.mode(WIFI_AP);
    WiFi.softAP(APssid, APpassword);
    Serial.print("WIFI status: ");
    Serial.println(wifiStatuscode(WiFi.status()));
  }


  
  //RTC update from WIFI
  if(isOnline) timeClient.begin();
  if (!(rtcStatus= rtc.begin())) {
    Serial.println("Couldn't find RTC");
  }
  if(isOnline && rtcStatus)
  {
    Serial.println("RTC OK");
    if(timeClient.update()){
      rtc.adjust(DateTime(timeClient.getEpochTime()));
      Serial.print("RTC updated from NTP: ");
      Serial.println(timeClient.getEpochTime());
    }else{
      Serial.println("NTP ERROR");
    }
  }
  
  //Webserver initialisation
  Serial.println("Server Starting...");
  server.on("/", outputHTML);
  server.on("/raw", outputRAW);
  server.onNotFound(outputNotFound);
  server.begin();
  Serial.println("Server Started!");
}

//Main loop
void loop() {
  DateTime currentTime = rtc.now();
  if(currentTime.second() == 0) {
    
  }
  server.handleClient();
}

//Create output webpage
void outputHTML() {
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
  DateTime currentTime = rtc.now();
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = (bme.readPressure() / 100.0f) + 10.44f;
  altitude = bme.readAltitude(pressure);
  wifiStrength = WiFi.RSSI();
  server.send(200, "text/html", sendHTML(temperature, humidity, pressure, altitude, wifiStrength, currentTime)); 
}

//Create output RAW data
void outputRAW() {
  DateTime currentTime = rtc.now();
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  wifiStrength = WiFi.RSSI();
  server.send(200, "text/plain", sendRAW(temperature, humidity, pressure, altitude, wifiStrength, currentTime)); 
}

//Notfound page
void outputNotFound(){
  server.send(404, "text/plain", "Not found");
}

//Raw data string creator
String sendRAW(float temperature,float humidity,float pressure,float altitude, float wifiStrength, DateTime currentTime){
  String ptr = "[";
  ptr +=sizeof(temperature);
  ptr +=",";
  ptr +=sizeof(humidity);
  ptr +=",";
  ptr +=sizeof(pressure);
  ptr +=",";
  ptr +=altitude;
  ptr +=",";
  ptr +=wifiStrength;
  ptr +=",";
  ptr += sizeof(currentTime.unixtime());
  ptr +="]";
  return ptr;
}

//HTML Creator
String sendHTML(float temperature,float humidity,float pressure,float altitude, float wifiStrength, DateTime currentTime){
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr +="<meta http-equiv=\"refresh\" content=\"5\">\n";
  ptr +="<title>ESP8266 Weather Station</title>\n";
  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;}\n";
  ptr +="p {font-size: 24px;color: #444444;margin-bottom: 10px;}\n";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<div id=\"webpage\">\n";
  ptr +="<h1>ESP8266 Weather Station</h1>\n";
  ptr +="<p>Temperature: ";
  ptr +=temperature;
  ptr +="&deg;C</p>";
  ptr +="<p>Humidity: ";
  ptr +=humidity;
  ptr +="%</p>";
  ptr +="<p>Pressure: ";
  ptr +=pressure;
  ptr +="hPa</p>";
  ptr +="<p>Altitude: ";
  ptr +=altitude;
  ptr +="m</p>";
  ptr +="<p>WIFI strength: ";
  ptr +=wifiStrength;
  ptr +="</p>";
  ptr +="<p>RTC time:  ";
  ptr += String(currentTime.year(), DEC) + '/' + String(currentTime.month(), DEC) + '/' + String(currentTime.day(), DEC) + " " + String(currentTime.hour(), DEC) + ':' + String(currentTime.minute(), DEC) + ':' + String(currentTime.second(), DEC);;
  ptr +="</p>";
  ptr +="</div>\n";
  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}

//Status code creator
String wifiStatuscode(wl_status_t code) {
  String str = "";
  switch( code ) {
  case WL_CONNECTED:
      str = "WL_CONNECTED";
      break;
  case WL_NO_SHIELD:
      str = "WL_NO_SHIELD";
      break;
  case WL_IDLE_STATUS:
      str = "WL_IDLE_STATUS";
      break;
  case WL_NO_SSID_AVAIL:
      str = "WL_NO_SSID_AVAIL";
      break;
  case WL_SCAN_COMPLETED:
      str = "WL_SCAN_COMPLETED";
      break;
  case WL_CONNECT_FAILED:
      str = "WL_CONNECT_FAILED";
      break;
  case WL_CONNECTION_LOST:
      str = "WL_CONNECTION_LOST";
      break;
  case WL_DISCONNECTED:
      str = "WL_DISCONNECTED";
      break;
  }
  return str;
}
