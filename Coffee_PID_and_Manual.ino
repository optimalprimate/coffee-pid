
#include <max6675.h>
#include <Wire.h>
#include <SPI.h>
#include <PID_v1.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET -1  // GPIO0
Adafruit_SSD1306 OLED(OLED_RESET);

#define PIN_INPUT 0
#define RELAY_PIN 5 //this pin is set manualy in code below

const char* ssid = "***";
const char* password =  "***";
const char* mqttServer = "192.168.1.**";
const int mqttPort = 1883;
int holding_var = 0;
unsigned long time_now = 0;
int pid_status = 0; //turns PID on or off (off for warm up)
int pidVmanual = 1; //switch for PID(0) or Manual (1) boiler control
int pidVmanual_prev;
int manual_set = 98; //SETPOINT for manual temp (will set to -3 on thermostat)
int heater_status = 0;
int system_status = 0;
int system_status_prev = 0;
int count = 0;
int count2 = 0;
int temp_off_timer = 0;
int manual_timer = 0;
WiFiClient espClient;
PubSubClient client(espClient);

//define thermocouple pins
int ktcSO = 12;
int ktcCS = 13;
int ktcCLK = 14;
MAX6675 ktc(ktcCLK, ktcCS, ktcSO);

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=1.5, Ki=1, Kd=4;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize = 1000;
unsigned long windowStartTime;
int temp_time = 0; //for temp read loop (sensor sample rate 225ms)

void setup()
{
   Serial.begin(9600);
  OLED.begin();
  OLED.display();

   //WIFI ------------------

   WiFi.mode(WIFI_STA);
WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  client.setServer(mqttServer, mqttPort);
client.setCallback(callback);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
// Create a random client ID
    String clientId = "coffee_";
    clientId += String(random(0xffff), HEX);
if (client.connect(clientId.c_str())) {    
      Serial.println("connected");  
 
    } else {
 
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
 
    }
  }
 
  client.publish("esp/test", "Hello from ESP_Coffee");
  //subscribe to coffee topic
client.subscribe("esp/coffee_status");

    ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();

  //WIFI ends ------------------------
  windowStartTime = millis();
 pinMode(16, OUTPUT);
 pinMode(3, INPUT);
  //initialize the variables we're linked to
  Setpoint = 85;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
   Input = ktc.readCelsius();
client.publish("esp/coffee_statread",String("0").c_str());
sysoffdisp();
}

//reconnect loop -----------------------------------
void reconnect() {
  Serial.println("Reconnect activated");
  // Loop until we're reconnected
  while (!client.connected()) {
  Serial.print("Attempting MQTT connection...");
String clientId = "coffee_";
    clientId += String(random(0xffff), HEX);
if (client.connect(clientId.c_str())) {    
      Serial.println("connected");
        client.publish("esp/test", "Hello from ESP_coffee(recon)");
        client.subscribe("esp/coffee_status");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//Callback function for recieving messages ------------------->
void callback(char* topic, byte* payload, unsigned int length) {
  for (int i = 0; i < length; i++) {
    holding_var = (payload[i]);
      if(holding_var == 49) { //set to what inject is 
      system_status = 1; //COFFEE ON, 49 = '1'
    }
     if(holding_var == 48) {
      system_status = 0; //COFFEE OFF, 48 = '0'
    }
  if(holding_var == 50) { //set to what inject is 
      pidVmanual = 0; //PID Heater Control ON, 50 = '2'
    }
  if(holding_var == 51) { //set to what inject is 
      pidVmanual = 1; //Manual Heater Control ON, 51 = '3'
    }
  }
}

//end of callback ----------------------------------------->

void sysoffdisp(){ //this is the display when the system is off
        OLED.begin();
      OLED.clearDisplay();
      OLED.setTextSize(1);
      OLED.setTextColor(SSD1306_WHITE);
      OLED.setCursor(0, 0);
      OLED.println("System OFF");
      OLED.setTextSize(2);
      OLED.setCursor(64, 16);
      OLED.println(Input);
      OLED.display(); //output 'display buffer' to screen  
}

void manualdisp(){ //this is the display when manual heating
      OLED.begin();
      OLED.clearDisplay();
      OLED.setTextSize(1);
      OLED.setTextColor(SSD1306_WHITE);
      OLED.setCursor(0, 0);
      OLED.println("Manual Heater Control");
      OLED.setTextSize(2);
      OLED.setCursor(64, 16);
      OLED.println(Input);
      OLED.display(); //output 'display buffer' to screen  
}

void print2screen() { //this is the PID display function
 OLED.begin();
 OLED.clearDisplay();
 //text1
 OLED.setTextSize(1);
 OLED.setTextColor(SSD1306_WHITE);
 OLED.setCursor(0, 0);
 OLED.println("Current Temp:");
 //text2
  OLED.setTextSize(2);
 OLED.setCursor(0, 16);
 OLED.println(Input);
//text3
  OLED.setTextSize(1);
 OLED.setCursor(105, 0);
 OLED.println("Set:");
//text4
  OLED.setTextSize(1);
 OLED.setCursor(95, 16);
 OLED.println(Setpoint);
 
 
  OLED.display(); //output 'display buffer' to screen  
 

}
void heateron(){
    digitalWrite(16, HIGH);
  if (heater_status == 0){
    client.publish("esp/kit_cof_heat",String("1").c_str());
  }
  heater_status = 1;
  
}
void heateroff(){
    digitalWrite(16, LOW);
  if (heater_status == 1){
    client.publish("esp/kit_cof_heat",String("0").c_str());
  }
  heater_status = 0;
}

//system check for nodered, tells if on or off
void system_status_check(){
  if (system_status != system_status_prev) {
    if (system_status == 1) {
      client.publish("esp/coffee_statread",String("1").c_str());
    }
    else if (system_status == 0) {
      client.publish("esp/coffee_statread",String("0").c_str());
      heateroff();
      //sysoffdisp();
    }
      system_status_prev = system_status;
  }
  //PID manual check
   if (pidVmanual != pidVmanual_prev) {
    if (pidVmanual == 1) {
      client.publish("esp/coffee_statread",String("MANUAL ON").c_str());
       heateroff();
    }
    else if (pidVmanual == 0) {
      client.publish("esp/coffee_statread",String("PID ON").c_str());
      heateroff();
    }
      pidVmanual_prev = pidVmanual;
  }
}

void loop()
{
  ArduinoOTA.handle();
  system_status_check();

if (system_status == 1) { //system on

if (pidVmanual ==0){ //check if we want PID on

//%%%%%%%%%% SEPARATE WARM UP IF FROM COLD %%%%%%%%%%%
  if (millis() > temp_time + 250){
   Input = ktc.readCelsius();
   if (Input > 85 && Input < 120){
     pid_status = 1;
      myPID.Compute();
   }
    else if (Input < 85){
      pid_status = 0;
      heateron(); //Heat if under 85C
    }
    else if (Input > 120){ //safety cutoff
      pid_status = 0;
      heateroff();
   }
   temp_time = millis();
  }

//%%%%%%%%%% SEPARATE WARM UP END %%%%%%%%%%%
 //Set start time of new window:
    if (millis() - windowStartTime > WindowSize)
    { 
     windowStartTime += WindowSize;
     Serial.println(Input);
     //pub temp:
      client.publish("esp/kit_coffee",String(Input).c_str());
     //every 10s
        if (count > 10){
          count = 0;
          print2screen();
        }
        count += 1;
   }

  if (pid_status == 1){
   //If within the first *output* ms of window, heater on:
   if (Output > millis() - windowStartTime){
     heateron();
   }
   else {
     heateroff();
    }
  }
}//end of if PIDVManual=0 = if PID ON

//%%%%%%MANUAL HEATER CONTROL ALGORITHMS%%%%%%%%%%%%%%%%%%

else if (pidVmanual == 1){ //if manual heater control
    if (millis() > manual_timer + 300){
    //pub temp:
    Input = ktc.readCelsius();
    
    //TEMP CONTROL
      //if (Input > 85 && Input < 120){
      if (Input < manual_set-3){
        heateron();
      }
      else if (Input > manual_set-3){
        heateroff(); //Heat if under 85C
      }
      else if (Input > 120){ //safety cutoff
      heateroff();
      }
 
    //every 3s
      if (count2 > 10){
      count2 = 0;
       client.publish("esp/coffee_setpoint",String(manual_set).c_str());
       client.publish("esp/kit_coffee",String(Input).c_str());
      manualdisp();
      }
        count2 += 1;
    manual_timer = millis();
    }
  
  
}//end of if Manual Heater Control
  
//%%%%%%END OF MANUAL HEATER CONTROL ALGORITHMS%%%%%%%%%%%%%%
}//end of if system_status=1

else if (system_status == 0){ //this is the OFF display
  heateroff();
  if (millis() > temp_off_timer + 5000){
 //pub temp:
 Input = ktc.readCelsius();
      client.publish("esp/kit_coffee",String(Input).c_str());
      sysoffdisp();
       temp_off_timer = millis();
  }
}
client.loop();
}
