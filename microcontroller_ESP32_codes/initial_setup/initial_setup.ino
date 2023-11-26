    /////////////////////////////////////////////////////////////////
   //        BRACELET TO MEASURE THE UPPER-ARM CONTRACTION        //
  //           Author: Jesse de Oliveira Santana Alves           //
 //                 Email: jessalves2@gmail.com                 //
/////////////////////////////////////////////////////////////////


// =======================================================
// ================>  Code Description  <=================
// =======================================================

// This code has to be upload first in the new microcontroller (ESP32 used in this project),
// in order to set up in a - json file - initial values of the variables: 

// const char* type_protocol;
// const char* wifi_status;
// const char* ssid_json;
// const char* password_json; 

// This is necessary, because the microcontroller main code save the wifi and protocl information,
// in order to avoid wirte down the ssid and password wifi every time to use the bracelet.
// Therefore, some variables is save into a board memory.

// =======================================================
// ====================>  Libraries  <====================
// =======================================================
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

// =======================================================
// ====================>  Variables  <====================
// =======================================================
const char* type_protocol;
const char* ssid_json;
const char* password_json; 

void setup() {  
  Serial.begin(115200);
  
  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
    return;
  }

  // Create a JSON object
  StaticJsonDocument<200> doc;

  // Set variables in the JSON object
  doc["type_protocol"] = "not configured";
  doc["ssid_json"] = "none";
  doc["password_json"] = "none";

// ============================================= Saving the variables from json file
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

  // =========================================== CHECKING IF THE VARIABLES WAS WRITTEN ==============================
    // Read the file from SPIFFS
  //File file = SPIFFS.open("/config.json", "r");
  file = SPIFFS.open("/protocol_info.json", "r");
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
  ssid_json = doc["ssid_json"];
  password_json = doc["password_json"]; 

  // Print Variable to test if the reading was with success
  Serial.println("Check the Writing Variables:");
  Serial.println(type_protocol);
  Serial.println(ssid_json);
  Serial.println(password_json);

  // Close the file
  file.close();

  Serial.println(" ");
  Serial.println("The Initial Setup was done with success!");
}

void loop() {
}
