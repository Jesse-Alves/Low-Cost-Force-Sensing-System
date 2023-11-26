    /////////////////////////////////////////////////////////////////
   //        BRACELET TO MEASURE THE UPPER-ARM CONTRACTION        //
  //           Author: Jesse de Oliveira Santana Alves           //
 //                 Email: jessalves2@gmail.com                 //
/////////////////////////////////////////////////////////////////

// =======================================================
// ====================>  Libraries  <====================
// =======================================================
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include <iostream>
#include <stdio.h>
#include "AsyncUDP.h"

#include <vector>
#include <numeric>

// =======================================================
// ================>  Global Variables  <=================
// =======================================================

// ============================================ ADJUST VARIABLES ================================================

// Define the quantity of sensors - YOU JUST CAN CHOOSE 6 OR 3.
int qtd_sensors = 3;

// Define the time [IN SECONDS] desired for get minimum and maximum time in calibration.
int calibration_time = 5; //seconds

// The frequency to send data in microcontroller (Hz)
unsigned long timer_frequency = 2000; // Hz

// ==============================================================================================================

// =======> Sensor and Calibration
// The Sensors Inputs ports in ESP32 board
#define sensor1  25
#define sensor2  27
#define sensor3  32
#define sensor4  33
#define sensor5  34
#define sensor6  35
//#define sensor7  32
//#define sensor8  4

// The FSR variables for 6 or 3 sensors
float fsr1, fsr2, fsr3, fsr4, fsr5, fsr6, fsr;
float fsr1_min, fsr2_min, fsr3_min, fsr4_min, fsr5_min, fsr6_min;
float fsr1_max, fsr2_max, fsr3_max, fsr4_max, fsr5_max, fsr6_max;
float value, v_min, v_max, v_mapped;
unsigned long startTime; // Variable for calibration process

// =======> Reset Button - To call the protocol setup
#define buttonPin 15

// =======> Digital Outputs to Controll Battery Charging Current with Modulus Click3
#define sclPin 22
#define sdaPin 21


// =======> Wifi Communication
AsyncUDP udp;
String confirm_message;

// =======> Setup Communication
const char* type_protocol;
const char* ssid_json;
const char* password_json; 
StaticJsonDocument<200> doc;  // Create a JSON object
String tp_check;

// =======> GUI variables
String type_protocolGUI;
String ssidGUI;
String passwordGUI;
int cont = 0;


// =======> Timer Callback Variables
unsigned long previousTime = 0;
unsigned long interval = 1000/timer_frequency; // Time interval in milliseconds (1 second)
unsigned long currentTime;

// =======================================================
// ====================>  Functions  <====================
// =======================================================
void import_initial_variables(){
  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
    return;
  }

  // Read the file from SPIFFS
  File file = SPIFFS.open("/protocol_info.json", "r");
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  // Parse the JSON file and store the data in the JSON object
  DeserializationError error = deserializeJson(doc, file);
  if (error) {
    Serial.println("Failed to parse file");
  }

  // ============================================= Reading the variables from json file
  type_protocol = doc["type_protocol"];
  ssid_json     = doc["ssid_json"];
  password_json = doc["password_json"]; 


  // Print Variable to test if the reading was with success
  // Serial.println("The values was imported with succes: ");
  // Serial.println(type_protocol);
  // Serial.println(ssid_json);
  // Serial.println(password_json);
  // Serial.println(" ");

  // Close the file
  file.close();
}
void save_variable_jsonfile(const char* variable, const char* new_value){

   // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
    return;
  }

  // ========================================= Update the specific variable
  doc[variable] = new_value;

  // Open the file in write mode
  File file = SPIFFS.open("/protocol_info.json", "w");
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  // Serialize the JSON object to the file
  if (serializeJson(doc, file) == 0) {
    Serial.println("Failed to write to file");
  }
  // Close the file
  file.close();  
}
void setup_type_of_communication(){  

  // ===============================> Get data from setup bracelet python executable
  Serial.println("Waiting the Setup GUI...");
  while(true){    
    //Serial.println(cont);

    if (Serial.available() > 0) {
      if (cont == 0){
        type_protocolGUI = Serial.readString();  // Read the incoming data from the serial port
        if ((type_protocolGUI == "serial") || (type_protocolGUI == "wifi")){
          cont++;
          Serial.println("received protocol: " + String(type_protocolGUI));  
        }
      }
      else if(cont == 1){
        ssidGUI = Serial.readString();  // Read the incoming data from the serial port
        cont++;
        Serial.println("received ssid: " + String(ssidGUI));          
      }
      else if(cont == 2){
        passwordGUI = Serial.readString();  // Read the incoming data from the serial port        
        Serial.println("received password: " + String(passwordGUI));          
        break;
      }
    }
  }

  //Serial.println("Passed the while loop: ");
  //type_protocolGUI = "wifi";
  //ssidGUI = "Jesse Alves";
  //passwordGUI = "thalitajesse";  
  //Serial.println(type_protocolGUI.c_str());
  //Serial.println(ssidGUI.c_str());
  //Serial.println(passwordGUI.c_str());

  // ===============================> Convert String to Const Char
  type_protocol = type_protocolGUI.c_str();
  ssid_json = ssidGUI.c_str();
  password_json = passwordGUI.c_str();

  // ===============================> Save receipt data in json file
  const char* tp_name = "type_protocol";
  const char* ssid_name = "ssid_json";
  const char* pass_name = "password_json";
  
  save_variable_jsonfile(tp_name, type_protocol);
  save_variable_jsonfile(ssid_name, ssid_json);
  save_variable_jsonfile(pass_name, password_json);

  delay(1000);
  // Configure wifi
  if (type_protocolGUI == "wifi"){
    configure_wifi();
  }
     
  // After that, the code go to Calibration Step (In Setup Function).
}
void configure_wifi(){
  import_initial_variables();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid_json, password_json);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("WiFi Failed! Trying to connect again!");
      delay(1000);
      configure_wifi();
      //while(1) {
      //    delay(1000);
      //}
  }
  if(udp.listen(1234)) {
      Serial.print("Wifi Connected!! The UDP Listening on IP: ");
      Serial.println(WiFi.localIP());
      udp.onPacket([](AsyncUDPPacket packet) {
          // ======================================================> Get data from python/ROS code via Wifi
          char* tmpStr = (char*) malloc(packet.length() + 1);
          memcpy(tmpStr, packet.data(), packet.length());
          tmpStr[packet.length()] = '\0'; // ensure null termination
          confirm_message = String(tmpStr);
          free(tmpStr); // Strign(char*) creates a copy so we can delete our one

          //Serial.println(confirm_message);          
      });
  }

  // Notify that wifi connected
  // ========================> Turn on a green LED here
  
}
void getWifimsg(){
  if(udp.listen(1234)) {
      udp.onPacket([](AsyncUDPPacket packet) {
          // ======================================================> Get data from python/ROS code via Wifi
          char* tmpStr = (char*) malloc(packet.length() + 1);
          memcpy(tmpStr, packet.data(), packet.length());
          tmpStr[packet.length()] = '\0'; // ensure null termination

          // Get the Wifi Message from Python Code 
          confirm_message = String(tmpStr);
          //Serial.println(confirm_message);

          free(tmpStr); // Strign(char*) creates a copy so we can delete our one                    
      });
  }
}
void getSerialmsg(){
    if (Serial.available() > 0) {        
      confirm_message = Serial.readString();  // Read the incoming data from the serial port
    }  
}
void calibration(){
    // ======================================= Vectors for Calibration ===================================================
    float sum = 0.0;
    std::vector<float> vec_fsr1;    std::vector<float> vec_fsr2;    std::vector<float> vec_fsr3;
    std::vector<float> vec_fsr4;    std::vector<float> vec_fsr5;    std::vector<float> vec_fsr6;
    
 
    //================================================== WIFI =============================================================   
    if (tp_check == "wifi"){

      // I - Notify that the communication is via serial
      confirm_message = " ";
      //Serial.println("wifi");
      while(true){
        Serial.println("wifi");
        udp.broadcast("wifi");
        delay(10);
        getWifimsg();               
        if (confirm_message == "got_it"){
          //Serial.println("First Got it message received!"); 
          //Serial.println("got_it");  
          break;
        }        
        

        // =================================== Check if the setup button was pressed ===========================
        if((digitalRead(buttonPin))){
          type_protocol = "not configured";
          const char* tp_name = "type_protocol";  
          save_variable_jsonfile(tp_name, type_protocol);
          setup_type_of_communication();        
        } 
        // ======================================================================================================
      }    


      // II - Get the MINIMUM value of each FSR sensors
      // ===========================> Wait the confirmation in GUI to start the reading
      confirm_message = " ";
      while(true){      
        //Serial.println("getting_minimum");
        getWifimsg();
        if (confirm_message == "got_it"){   
          //Serial.println("Second Got it message received!"); 
          //delay(50);   
          //Serial.println("got_it"); 
          //udp.broadcast("got_it");   
          break;
        }   

        // =================================== Check if the setup button was pressed ===========================
        if((digitalRead(buttonPin))){
          type_protocol = "not configured";
          const char* tp_name = "type_protocol";  
          save_variable_jsonfile(tp_name, type_protocol);
          setup_type_of_communication();        
        } 
        // ======================================================================================================
      }

      // ===========================> Start to read during the calibration time defined
      startTime = millis();
      while (millis() - startTime < 1000*calibration_time) {  
        //udp.broadcast("calibrating...");
        //Serial.println("calibrating...");
        delay(10);        
        vec_fsr1.push_back(analogRead(sensor1));   vec_fsr2.push_back(analogRead(sensor2));   vec_fsr3.push_back(analogRead(sensor3));
        if (qtd_sensors == 6){
          vec_fsr4.push_back(analogRead(sensor4));   vec_fsr5.push_back(analogRead(sensor5));   vec_fsr6.push_back(analogRead(sensor6)); 
        }
      }

      // for (int ii=0;ii<50;ii++){
      //   udp.broadcast("Calibration finished");
      //   Serial.println("Calibration finished");
      // }  
      
      // Compute the Vector Mean
      sum = std::accumulate(vec_fsr1.begin(), vec_fsr1.end(), 0.0);      fsr1_min = sum / vec_fsr1.size();
      sum = std::accumulate(vec_fsr2.begin(), vec_fsr2.end(), 0.0);      fsr2_min = sum / vec_fsr2.size();
      sum = std::accumulate(vec_fsr3.begin(), vec_fsr3.end(), 0.0);      fsr3_min = sum / vec_fsr3.size();
      if (qtd_sensors == 6){
        sum = std::accumulate(vec_fsr4.begin(), vec_fsr4.end(), 0.0);      fsr4_min = sum / vec_fsr4.size();
        sum = std::accumulate(vec_fsr5.begin(), vec_fsr5.end(), 0.0);      fsr5_min = sum / vec_fsr5.size();
        sum = std::accumulate(vec_fsr6.begin(), vec_fsr6.end(), 0.0);      fsr6_min = sum / vec_fsr6.size();
      }
      
      //Serial.println("Clearing the vectors...");      
      // Clear the Vector to use them to maximum as well.
      vec_fsr1.clear();      vec_fsr2.clear();      vec_fsr3.clear();
      if (qtd_sensors == 6){vec_fsr4.clear();      vec_fsr5.clear();      vec_fsr6.clear();}
 
      
      //Serial.println("got_min");
      // for (int ii=0;ii<10;ii++){
      //   udp.broadcast("got_min");
      //   delay(10);
      // }
      udp.broadcast("got_min");
      udp.broadcast("got_min");
          
      // SEND AND RECEIVE CONFIRMATION.
      confirm_message = " ";
      while(true){     
        //getSerialmsg();
        getWifimsg();
        //delay(50);
        if (confirm_message == "got_it"){ 
          //Serial.println("First Got it message received!");   
          //delay(50); 
          //Serial.println("got_it");    
          //udp.broadcast("got_it"); 
          break;
        }       
      } 

      // III - Get the MAXIMUM value of each FSR sensors
      startTime = millis();
      while (millis() - startTime < 1000*calibration_time) {  
        //Serial.println("calibrating...");
        //udp.broadcast("calibrating...");
        delay(10); 
        vec_fsr1.push_back(analogRead(sensor1));   vec_fsr2.push_back(analogRead(sensor2));   vec_fsr3.push_back(analogRead(sensor3));
        if (qtd_sensors == 6){
          vec_fsr4.push_back(analogRead(sensor4));   vec_fsr5.push_back(analogRead(sensor5));   vec_fsr6.push_back(analogRead(sensor6));
        }
      }

      // for (int ii=0;ii<50;ii++){
      //   Serial.println("Calibration finished");
      //   udp.broadcast("Calibration finished");
      // }  

      // Compute the Vector Mean
      sum = std::accumulate(vec_fsr1.begin(), vec_fsr1.end(), 0.0);      fsr1_max = sum / vec_fsr1.size();
      sum = std::accumulate(vec_fsr2.begin(), vec_fsr2.end(), 0.0);      fsr2_max = sum / vec_fsr2.size();
      sum = std::accumulate(vec_fsr3.begin(), vec_fsr3.end(), 0.0);      fsr3_max = sum / vec_fsr3.size();
      if (qtd_sensors == 6){
        sum = std::accumulate(vec_fsr4.begin(), vec_fsr4.end(), 0.0);      fsr4_max = sum / vec_fsr4.size();
        sum = std::accumulate(vec_fsr5.begin(), vec_fsr5.end(), 0.0);      fsr5_max = sum / vec_fsr5.size();
        sum = std::accumulate(vec_fsr6.begin(), vec_fsr6.end(), 0.0);      fsr6_max = sum / vec_fsr6.size();
      }

      // Clear the Vectors
      vec_fsr1.clear();      vec_fsr2.clear();      vec_fsr3.clear();
      if (qtd_sensors == 6){vec_fsr4.clear();      vec_fsr5.clear();      vec_fsr6.clear();}


      // for (int ii=0;ii<10;ii++){
      //   udp.broadcast("got_max");
      //   delay(10);
      // }
      udp.broadcast("got_max");
      udp.broadcast("got_max");
      
      // SEND AND RECEIVE CONFIRMATION.
      confirm_message = " ";
      while(true){        
        //getSerialmsg();
        getWifimsg();
        if (confirm_message == "got_it"){ 
          //Serial.println("First Got it message received!");  
          //delay(50);
          //Serial.println("got_it");       
          //udp.broadcast("got_it");
          break;
        }       
      }

    }

    //================================================== SERIAL =============================================================    
    else if (tp_check == "serial"){    

      // I - Notify that the communication is via serial
      confirm_message = " ";
      while(true){
        Serial.println("serial");
        delay(50);
        if (Serial.available() > 0) {
          confirm_message = Serial.readString();
          
          if (confirm_message == "got_it"){
            //Serial.println("First Got it message received!"); 
            Serial.println("got_it");  
            break;
          }        
        }

        // =================================== Check if the setup button was pressed ===========================
        if((digitalRead(buttonPin))){
          type_protocol = "not configured";
          const char* tp_name = "type_protocol";  
          save_variable_jsonfile(tp_name, type_protocol);
          setup_type_of_communication();        
        } 
        // ======================================================================================================
      }    


      // II - Get the MINIMUM value of each FSR sensors
      // ===========================> Wait the confirmation in GUI to start the reading
      confirm_message = " ";
      while(true){      
        //Serial.println("getting_minimum");
        getSerialmsg();
        if (confirm_message == "got_it"){   
          //Serial.println("Second Got it message received!"); 
          delay(50);   
          Serial.println("got_it");    
          break;
        }   

        // =================================== Check if the setup button was pressed ===========================
        if((digitalRead(buttonPin))){
          type_protocol = "not configured";
          const char* tp_name = "type_protocol";  
          save_variable_jsonfile(tp_name, type_protocol);
          setup_type_of_communication();        
        } 
        // ======================================================================================================
      }

      // ===========================> Start to read during the calibration time defined
      startTime = millis();
      while (millis() - startTime < 1000*calibration_time) {  
        Serial.println("calibrating...");
        delay(50);        
        vec_fsr1.push_back(analogRead(sensor1));   vec_fsr2.push_back(analogRead(sensor2));   vec_fsr3.push_back(analogRead(sensor3));
        if (qtd_sensors == 6){
          vec_fsr4.push_back(analogRead(sensor4));   vec_fsr5.push_back(analogRead(sensor5));   vec_fsr6.push_back(analogRead(sensor6)); 
        }
      }

      for (int ii=0;ii<50;ii++){
        Serial.println("Calibration finished");
      }  
      
      // Compute the Vector Mean
      sum = std::accumulate(vec_fsr1.begin(), vec_fsr1.end(), 0.0);      fsr1_min = sum / vec_fsr1.size();
      sum = std::accumulate(vec_fsr2.begin(), vec_fsr2.end(), 0.0);      fsr2_min = sum / vec_fsr2.size();
      sum = std::accumulate(vec_fsr3.begin(), vec_fsr3.end(), 0.0);      fsr3_min = sum / vec_fsr3.size();
      if (qtd_sensors == 6){
        sum = std::accumulate(vec_fsr4.begin(), vec_fsr4.end(), 0.0);      fsr4_min = sum / vec_fsr4.size();
        sum = std::accumulate(vec_fsr5.begin(), vec_fsr5.end(), 0.0);      fsr5_min = sum / vec_fsr5.size();
        sum = std::accumulate(vec_fsr6.begin(), vec_fsr6.end(), 0.0);      fsr6_min = sum / vec_fsr6.size();
      }
      
      Serial.println("Clearing the vectors...");      
      // Clear the Vector to use them to maximum as well.
      vec_fsr1.clear();      vec_fsr2.clear();      vec_fsr3.clear();
      if (qtd_sensors == 6){vec_fsr4.clear();      vec_fsr5.clear();      vec_fsr6.clear();}


      // for (int ii=0;ii<3;ii++){
      //   Serial.println("got_min");
      // } 
      
      Serial.println("got_min");
      delay(50);
      // SEND AND RECEIVE CONFIRMATION.
      confirm_message = " ";
      while(true){     
        getSerialmsg();
        //delay(50);
        if (confirm_message == "got_it"){ 
          //Serial.println("First Got it message received!");   
          delay(50); 
          Serial.println("got_it");     
          break;
        }       
      } 

      // III - Get the MAXIMUM value of each FSR sensors
      startTime = millis();
      while (millis() - startTime < 1000*calibration_time) {  
        Serial.println("calibrating...");
        delay(50); 
        vec_fsr1.push_back(analogRead(sensor1));   vec_fsr2.push_back(analogRead(sensor2));   vec_fsr3.push_back(analogRead(sensor3));
        if (qtd_sensors == 6){
          vec_fsr4.push_back(analogRead(sensor4));   vec_fsr5.push_back(analogRead(sensor5));   vec_fsr6.push_back(analogRead(sensor6));
        }
      }

      for (int ii=0;ii<50;ii++){
        Serial.println("Calibration finished");
      }  

      // Compute the Vector Mean
      sum = std::accumulate(vec_fsr1.begin(), vec_fsr1.end(), 0.0);      fsr1_max = sum / vec_fsr1.size();
      sum = std::accumulate(vec_fsr2.begin(), vec_fsr2.end(), 0.0);      fsr2_max = sum / vec_fsr2.size();
      sum = std::accumulate(vec_fsr3.begin(), vec_fsr3.end(), 0.0);      fsr3_max = sum / vec_fsr3.size();
      if (qtd_sensors == 6){
        sum = std::accumulate(vec_fsr4.begin(), vec_fsr4.end(), 0.0);      fsr4_max = sum / vec_fsr4.size();
        sum = std::accumulate(vec_fsr5.begin(), vec_fsr5.end(), 0.0);      fsr5_max = sum / vec_fsr5.size();
        sum = std::accumulate(vec_fsr6.begin(), vec_fsr6.end(), 0.0);      fsr6_max = sum / vec_fsr6.size();
      }

      // Clear the Vectors
      vec_fsr1.clear();      vec_fsr2.clear();      vec_fsr3.clear();
      if (qtd_sensors == 6){vec_fsr4.clear();      vec_fsr5.clear();      vec_fsr6.clear();}


      Serial.println("got_max");
      delay(50);
      // SEND AND RECEIVE CONFIRMATION.
      confirm_message = " ";
      while(true){        
        getSerialmsg();
        if (confirm_message == "got_it"){ 
          //Serial.println("First Got it message received!");  
          delay(50);
          Serial.println("got_it");       
          break;
        }       
      }       

    }    
    // Then, it is ready to send message to ROS.
    //delay(3000);
}
float mapping(float value, float v_min, float v_max){
  v_mapped = ((value - v_min)/(v_max - v_min))*100;
  return v_mapped;
}

// =======================================================
// ===================>  Main Setup  <====================
// =======================================================
void setup() {

  // Set baud rate
  Serial.begin(115200);

  // Define all sensor port as inputs
  pinMode(sensor1, INPUT);  pinMode(sensor2, INPUT);  pinMode(sensor3, INPUT);
  if (qtd_sensors == 6){ pinMode(sensor4, INPUT);  pinMode(sensor5, INPUT);  pinMode(sensor6, INPUT); }

  // The button to call the protocol setup
  pinMode(buttonPin, INPUT);

  // Current Control Pins - The LOW value put a maximum current of 1.2 A, then charge the battery faster.
  pinMode(sclPin, OUTPUT);
  pinMode(sdaPin, OUTPUT);
  digitalWrite(sclPin, LOW);
  digitalWrite(sdaPin, LOW);

  // 1 - Import all initial variables from json file  
  import_initial_variables();
  //Serial.println("tp imported is: " + String(type_protocol));

  // 2 - Check the value of type_protocol variable 
  tp_check = String(type_protocol);

  if (tp_check == "not configured"){   
    setup_type_of_communication(); 
  }
  else if (tp_check == "serial"){
    // Pass and Go directly to Calibration Step  
  }
  else if (tp_check == "wifi"){    
    // If the tp is wifi, call a function to initialize the wifi connection
    configure_wifi();
  }

  // 3 - Calibration Step
  calibration();    
}

// =======================================================
// =============>  Main Loop to Send Data  <==============
// =======================================================

void send_data(){

  // Reading the sensors
  fsr1 = mapping(analogRead(sensor1),fsr1_min,fsr1_max);   
  fsr2 = mapping(analogRead(sensor2),fsr2_min,fsr2_max);
  fsr3 = mapping(analogRead(sensor3),fsr3_min,fsr3_max);

  if (qtd_sensors == 6){
    fsr4 = mapping(analogRead(sensor4),fsr4_min,fsr4_max);   
    fsr5 = mapping(analogRead(sensor5),fsr5_min,fsr5_max);
    fsr6 = mapping(analogRead(sensor6),fsr6_min,fsr6_max);
  }

  // Data mean
  if (qtd_sensors == 3){  
    fsr = (fsr1 + fsr2 + fsr3)/3;
  }
  else{
    fsr = (fsr1 + fsr2 + fsr3 + fsr4 + fsr5 + fsr6)/6;
  }
  
  // Saturation to avoid negative values or higher than 100%
  if(fsr < 0.0) {fsr = 0.0;} else if(fsr > 100.0) {fsr = 100.0;}


  // Decide for which protocol send the message
  if (tp_check == "serial"){
    Serial.println(fsr);
  }
  else if (tp_check == "wifi"){
    // Send data also via serial
    Serial.println(fsr);

    // Send data via wifi
    char buffer_msg[50];
    sprintf(buffer_msg, "%5.2f", fsr1_max);
    udp.broadcast(buffer_msg);
    //delay(1);
  }
}


void loop() {
  currentTime = millis();  // Get the actual time

  if (currentTime - previousTime >= interval) {
    previousTime = currentTime;  // Update the previous time

    // Call your function here
    send_data();
  }
}
