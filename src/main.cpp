#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <SFE_BMP180.h>
#include <MQ7.h>
#include <movingAvg.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <OSCMessage.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>
#include "SPIFFS.h"

//192.168.0.138

#define ANEMRADIUS         36.5                    //Radius of anemometer

#define DHTTYPE            DHT11
#define COV_RATIO          0.2                     //ug/mmm / mv
#define NO_DUST_VOLTAGE    400                     //no dust standart voltage

#define DHTPIN             23                      //dht11 signal pin
#define DLED               15                      //Dust sensor trigger led
#define WIND               4                       //interrupt pin to anemometer
#define ACRYLIC_OFFSET     0.14
#define WIND_UPDATE        3000
#define SENDTHINGSPEAK     30000

#define UV                 1                       //ADS A
#define LDR                2                       //ADS A
#define RAIN               3                       //ADS A

#define DUST               1                       //ADS B
#define MQ                 2                       //ADS B
#define MOIST              3                       //ADS B

#define PORT               5005                     //Server port
#define SSID               ""            //Wifi name
#define PASS               ""             //Wifi password
#define SERVERIP           "192.168.0.161"          //Server ip

#define READTIME           100                     //time between sensor send mensages

String hostname = "WeatherStation";                //Nome da estação meteorologica no dns

char addr_thingspeak[] = "api.thingspeak.com";
String key_thingspeak_1 = "";
String key_thingspeak_2 = "";
WiFiClient client;

WiFiUDP Udp;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

//Analog input modules
Adafruit_ADS1115 adsA;
Adafruit_ADS1115 adsB;

//Dust, temperature, pressure and humidity sensors instance
SFE_BMP180 bmp;
DHT dht(DHTPIN, DHTTYPE);
MQ7 mq7;

//Moving average for lowpass dust info
movingAvg avgDust(10);

//UDP client sender
WiFiUDP udp_client;

unsigned long previousCheck = 0;                    //last check if wifi is connected
unsigned long interval = 3000;                      //intervel between wifi checkins

//All collected data values
double T, P, H, CO, L, D, W, U, R, M;
//Anemometer variables
int count = 0;                                      //revolution counter for anemometer
float rpm = 0;                                      //rpm variable
int windCount = 0;
float windMean = 0.0;
unsigned long int lastRpm, lastRead, last_connection_time;

int humidMean[10];
int countH = 0;

/// <summary> Send a json over UDP </summary>
/// <param name="doc"> Json to be sended </param>
void udpSend(StaticJsonDocument<512> doc) {
  udp_client.beginPacket(SERVERIP, PORT);
  serializeJson(doc, udp_client);
  udp_client.println();
  udp_client.endPacket();
}

/// <summary> Emergency loop to be used if something goes wrong </summary>
void emergencyUpdateHandleLoop() {
  while(1) ArduinoOTA.handle();
}

void advertiseServices(const char *MyName) {
  if (MDNS.begin(MyName)) {
    Serial.println(F("mDNS responder started"));
    Serial.print(F("I am: "));
    Serial.println(MyName);
    MDNS.addService("n8i-mlp", "tcp", 23);
  }
  else {
      Serial.println("Error setting up MDNS responder");
      emergencyUpdateHandleLoop();
  }
}

/// <summary> Start wifi as a client </summary>
void WiFiStart() {
  delay(1000);
  Serial.print("Conectando em ");
  Serial.println(SSID);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(hostname.c_str());
  WiFi.begin(SSID, PASS);
  while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
  }
  Serial.println("Ready");
  Serial.print("IP address Host: ");
  Serial.println(WiFi.localIP());
  Serial.print("IP address Dest: ");
  Serial.println(SERVERIP);
  advertiseServices(hostname.c_str());
}

/// <summary> Check if wifi still is connected </summary>
void check_wifi() {
  unsigned long currentCheck = millis();
  if ((WiFi.status() != WL_CONNECTED) && (currentCheck - previousCheck>=interval)) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousCheck = currentCheck;
  }
}



/// <summary> Build a json with sensors parameters, print it to serial and send it </summary>
void send () {
  StaticJsonDocument<512> doc;
  char data[512];
  doc["temperature"] = T;
  doc["pressure"] = P;
  doc["humidity"] = H;
  doc["co"] = CO;
  doc["light"] = int(L);
  doc["dust"] = D;
  doc["wind"] = W;
  doc["rain"] = int(R);
  doc["moisture"] = int(M);
  doc["uv"] = U;
  serializeJson(doc, Serial);
  serializeJson(doc, data);
  ws.textAll(data);
  //udpSend(doc);
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
 void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void sendUDP () {
  String m = "";
  OSCMessage msg("/WEATHER");
  m+=String(T)+ " ";
  m+=String(P)+ " ";
  m+=String(H)+ " ";
  m+=String(CO)+ " ";
  m+=String(L)+ " ";
  m+=String(D)+ " ";
  m+=String(W)+ " ";
  m+=String(U)+ " ";
  m+=String(R)+ " ";
  m+=String(M)+ " ";
  Udp.beginPacket(SERVERIP, PORT);
  Udp.print(m);
  Udp.endPacket();
}
/// <summary> USED IN INTERRUPTION! count the number of revolutions of anemometer </summary>
void IRAM_ATTR addRotation() {
   count++;
}

/// <summary> Calculate the RPM </summary>
void rpmCalc() {
    float period = (millis() - lastRpm)/1000.;
    lastRpm = millis();
    rpm = ((count/2.)*60.)/period;
    count = 0;
}


/// <summary> Start the AD modules </summary>
void startAdc1115 () {
  if (!adsA.begin(0x48)) {
    Serial.println("Failed to initialize ADS1115 A.");
    emergencyUpdateHandleLoop();
  }
  if (!adsB.begin(0x49)) {
    Serial.println("Failed to initialize ADS1115 B.");
    emergencyUpdateHandleLoop();
  }
}

/// <summary> Calculate the wind speed in meter by second </summary>
// Unit: m/s
void windSpeed(){
  if(millis() - lastRpm > WIND_UPDATE){
    rpmCalc();
    W = ((4. * PI * ANEMRADIUS * rpm)/60.) / 1000.;
    windMean += W;
    windCount++;
  }
}

float windAverage() {
  float meanW = windMean/float(windCount);
  windCount = 0;
  windMean = 0.;
  return meanW;
}

/// <summary> Start the BMP180 sensor module </summary>
void startBmp180() {
  if (!bmp.begin()) {
    Serial.println("Failed to initialize BMP180");
    emergencyUpdateHandleLoop();
  }
}

/// <summary> Start the dust sensor module </summary>
void startDustsensor() {
  pinMode(DLED, OUTPUT);
	digitalWrite(DLED, LOW);
}

/// <summary> Read temperature from BMP180 </summary>
// Unit: C
void readTemperature() {
  T = dht.readTemperature();
}

/// <summary> Read pressure from BMP180 </summary>
// Unit mmHg
void readPressure() {
  char status;
  status = bmp.startPressure(3);
  if (status != 0) {
    delay(status);
    status = bmp.getPressure(P,T);
    P = P * 0.750061683;
  }
}

/// <summary> Read umidity from DHT11 </summary>
// Unit percent
void readHumidity() {
  int h = dht.readHumidity();
  humidMean[countH] = h;
  countH = countH % 10;
  countH++;
  H = 0;
  for (int i = 0; i < 10; i++) H += humidMean[i];
  H = H/10;
}

/// <summary> Encapsulate temperature, pressure and humidity reads, because then are sequence dependents </summary>
void getTempPressHum() {
  readTemperature();
  readPressure();
  readHumidity();
}

/// <summary> Get voltage reference to compare other reads </summary>
float getRef() {
  int16_t adValue;
  adValue = adsB.readADC_SingleEnded(0);
  return adsB.computeVolts(adValue);
}

/// <summary> Read CO concentration from MQ7 </summary>
// Unit ppm
void getCO() {
  int16_t adValue = adsB.readADC_SingleEnded(MQ);
  float volts = adsB.computeVolts(adValue);
  Serial.println(volts);
  CO = mq7.getPPM(volts, getRef());
}

/// <summary> Read light intensity from LDR </summary>
// Unit percentage
void getLight() {
  int16_t adValue = adsA.readADC_SingleEnded(LDR);
  L = 100 - (adsA.computeVolts(adValue)/getRef()) * 100;
}

/// <summary> Read UV intensity from UV sensor </summary>
// Unit UV index
void getUV() {
  int16_t adValue = adsA.readADC_SingleEnded(UV);
  float volts = adsA.computeVolts(adValue)+ACRYLIC_OFFSET;
  if (volts<=0.227)                       U = 0;
  else if (volts>0.227 && volts<=0.318)   U = 1;
  else if (volts>0.318 && volts<=0.408)   U = 2;
  else if (volts>0.408 && volts<=0.503)   U = 3;
  else if (volts>0.503 && volts<=0.606)   U = 4;
  else if (volts>0.606 && volts<=0.696)   U = 5;
  else if (volts>0.696 && volts<=0.795)   U = 6;
  else if (volts>0.795 && volts<=0.881)   U = 7;
  else if (volts>0.881 && volts<=0.976)   U = 8;
  else if (volts>0.976 && volts<=0.1079)  U = 9;
  else if (volts>0.1079 && volts<=0.1170) U = 10;
  else if (volts>0.1170)                U = 11;
}

/// <summary> Read Dust concentration </summary>
// Unit ug/m3
void getDust() {
  float D_inst;
  unsigned long int sum = 0;

  digitalWrite(DLED, LOW);
	delayMicroseconds(280);

  for (int i = 0; i < 10; i++) sum += adsB.readADC_SingleEnded(DUST);
  delayMicroseconds(40);
  digitalWrite(DLED, HIGH);

  float mv = adsB.computeVolts(sum/10)*1000.;
  D_inst = 0.17*mv-0.1;

  if ( D_inst < 0) D = 0.00;
  D = avgDust.reading(1000. * D_inst)/1000.;
}

/// <summary> Read rain status </summary>
// Unit percentage
void getRain() {
  int16_t adValue = adsA.readADC_SingleEnded(RAIN);
  R = 100 - (adsA.computeVolts(adValue)/getRef()) * 100;
}

/// <summary> Read soil moisture status </summary>
// Unit percentage
void getMoisture() {
  int16_t adValue = adsB.readADC_SingleEnded(MOIST);
  M = 100 - (adsB.computeVolts(adValue)/getRef()) * 100;
}

/// <summary> Read velocity of wind from anemometer </summary>
// Unit m/s
void startAnemometer() {
  pinMode(WIND, INPUT_PULLUP);
  attachInterrupt(WIND, addRotation, RISING);
  lastRpm = millis();
}


void startServer() {
server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html");
  });
  server.on("/effects.js", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/effects.js");
    });

}

void initwebsocket(){
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

//Setup function
void setup(void) {
  Serial.begin(115200);
  WiFiStart();
  ArduinoOTA.begin();
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  dht.begin();
  avgDust.begin();
  startAdc1115();
  startBmp180();
  startDustsensor();
  startAnemometer();
  last_connection_time = 0;
  startServer();
  initwebsocket();
  server.begin();
}

void printData() {
  Serial.println("-----------------------------------------------------------");
  Serial.print("REF_VOLTS: "); Serial.print(getRef()); Serial.println("V");
  Serial.print("Temperature: "), Serial.print(T), Serial.println(" C");
  Serial.print("Pressure: "), Serial.print(P), Serial.println(" mmHg");
  Serial.print("Humidity: "), Serial.print(H), Serial.println(" %");
  Serial.print("CO conc: "), Serial.print(CO), Serial.println(" ppm");
  Serial.print("Light intens: "),Serial.print(L), Serial.println(" %");
  Serial.print("Dust conc: "),Serial.print(D), Serial.println(" ug/m3");
  Serial.print("Wind speed: "),Serial.print(W), Serial.println(" m/s");
  Serial.print("UV: "),Serial.print(U), Serial.println(" Index");
  Serial.print("Rain: "),Serial.print(R), Serial.println(" %");
  Serial.print("Moisture: "),Serial.print(M), Serial.println(" %");

}



void loop() {
  //ws.cleanupClients();
  ArduinoOTA.handle();
  getTempPressHum();
  getCO();
  getLight();
  getDust();
  windSpeed();
  getUV();
  getRain();
  getMoisture();

  if(millis() - lastRead > READTIME) send();
  check_wifi();
}
