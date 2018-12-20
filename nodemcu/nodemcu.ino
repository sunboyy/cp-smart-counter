#include <Arduino.h>
#include <ArduinoJson.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h> 

#include <ESP8266HTTPClient.h>

ESP8266WiFiMulti WiFiMulti;
StaticJsonBuffer<1000> jsonBuffer;
HTTPClient http;

String site = "http://us-central1-cpsmartcounter.cloudfunctions.net/";
String connecting_site;
String payload;
char received_token;
uint8_t count, new_count;
int8_t change;
int http_code;
int try_connect;
bool reset_count = false;
uint8_t garbage[] = {0};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // WiFiMulti.addAP("OAK-M", "0840482880");
  WiFiMulti.addAP("OakChawit", "ZymphonY"); 
}

void loop() {
  // Receive count update from STM32
  if(Serial.available() > 0) {
//    String tmp = Serial.readString();
//    Serial.printf("%s\n", tmp.c_str());
    received_token = Serial.read();
    if(received_token == 'I')
      ++change;
    else if(received_token == 'D')
      --change;
  }

  if(WiFiMulti.run() == WL_CONNECTED) {
    // Reset count first
    if(!reset_count) {
      connecting_site = site + "resetCount";
      http.begin(connecting_site);
      http_code = http.POST(garbage, 1);
      if(http_code == HTTP_CODE_OK) {
        reset_count = true;
      }
      http.end();
    } else {
      // Update count according to STM32
      if(change > 0) {
        connecting_site = site + "incrementCount";
        try_connect = 1;
        http.begin(connecting_site);
        http_code = http.POST(garbage, 1);
        if(http_code == HTTP_CODE_OK)
          --change;
      } else if(change < 0) {
        connecting_site = site + "decrementCount";
        try_connect = 1;
        http.begin(connecting_site);
        http_code = http.POST(garbage, 1);
        if(http_code == HTTP_CODE_OK)
          ++change;
      } else {
        try_connect = 0;
      }
      
      if(try_connect == 1 && http_code == HTTP_CODE_OK) {
        payload = http.getString();
  
        jsonBuffer.clear();
        JsonObject& root = jsonBuffer.parseObject(payload);
  
        if(root.success()) {
          new_count = root["count"];
          if(count != new_count) {
            char p = new_count;
            Serial.printf("%c", p);
            count = new_count;
          }
        }
      }

      if(try_connect == 1) {
        try_connect = 0;
        http.end();
      }
    }
  }
  delay(500);
}
