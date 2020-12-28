/**
 * ESP8266 Weather station
 * Waterproof, solar charged weather station with battery based on ESP8266, BME280 and DS3231 RTC 
 * Data storage: 24C32 EEPROM
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
#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266WebServer.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//WIFI Connection trying overtime
#define OVERTIME 10

//Pressure init
#define SEALEVELPRESSURE_HPA (1013.25)

//Senzor
Adafruit_BME280 bme;

//Senzor variables
float temperature, humidity, pressure, altitude, wifiStrength;

//WIFI datas
const char* ssid = "";
const char* password = "";
const char* APssid = "";
const char* APpassword = "";

//Modul is online?
boolean isOnline = false;

//RTC init
RTC_DS3231 rtc;
const long utcOffsetInSeconds = 3600;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

//Webserver start
ESP8266WebServer server(80);              
 
void setup() {
  //Serial start for DEBUG
  Serial.begin(115200);
  delay(100);
  Serial.println(".");
  Serial.println("Inditas");
  
  //Sensor init
  bme.begin(0x76);

  //Wifi setup Station mode
  WiFi.mode(WIFI_STA);

  //connect to your local wi-fi network
  WiFi.begin(ssid, password);

  //check wi-fi is connected to wi-fi network
  int timeout = 0;
  while (!WiFi.isConnected() && timeout <= OVERTIME) {
    Serial.println("Connencting...");
    delay(1000);
    timeout++;
  }
  //Status update
  isOnline = WiFi.isConnected();
  //If not connected create AP
  if(timeout > OVERTIME){
    WiFi.disconnect();
    Serial.println("Overtime, start AP");
    //Wifi setup Station mode
    WiFi.mode(WIFI_AP);
    WiFi.softAP(APssid, APpassword);
  }

  if(isOnline){
    Serial.println("Sikeres kapcsolodasas");
  }else{
    Serial.println("Sikertelen kapcsolodasas");
  }
  
  //RTC update from WIFI
  if(isOnline) timeClient.begin();
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if(isOnline)
  {
    timeClient.update();
    rtc.adjust(DateTime(timeClient.getEpochTime()));
    Serial.print("RTC Frissitve: ");
    Serial.println(timeClient.getEpochTime());
  }
  Serial.println("Server Starting...");
  //Webserver initialisation
  server.on("/", outputHTML);
  server.on("/raw", outputRAW);
  server.onNotFound(outputNotFound);
  server.begin();
  Serial.println("Server Started");
}

void loop() {
  server.handleClient();
}

void outputHTML() {
  DateTime currentTime = rtc.now();
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  wifiStrength = WiFi.RSSI();
  server.send(200, "text/html", sendHTML(temperature, humidity, pressure, altitude, wifiStrength, currentTime)); 
}

void outputRAW() {
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  wifiStrength = WiFi.RSSI();
  server.send(200, "text/plain", sendRAW(temperature, humidity, pressure, altitude, wifiStrength)); 
}

void outputNotFound(){
  server.send(404, "text/plain", "Not found");
}

String sendRAW(float temperature,float humidity,float pressure,float altitude, float wifiStrength){
  String ptr = "";
  ptr +=temperature;
  ptr +=";";
  ptr +=humidity;
  ptr +=";";
  ptr +=pressure;
  ptr +=";";
  ptr +=altitude;
  ptr +=";";
  ptr +=wifiStrength;
  ptr +=";";
  return ptr;
}

String sendHTML(float temperature,float humidity,float pressure,float altitude, float wifiStrength, DateTime currentTime){
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr +="<meta http-equiv=\"refresh\" content=\"2\">\n";
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
  ptr +="<p>Current time:  ";
  ptr += String(currentTime.year(), DEC) + '/' + String(currentTime.month(), DEC) + '/' + String(currentTime.day(), DEC) + " " + String(currentTime.hour(), DEC) + ':' + String(currentTime.minute(), DEC) + ':' + String(currentTime.second(), DEC);;
  ptr +="</p>";
  ptr +="</div>\n";
  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}
