#include <freertos/FreeRTOS.h> 
// TFT
#include "Free_Fonts.h" // Include the header file attached to this sketch
#include "SPI.h"
#include "TFT_eSPI.h"
// Wi-Fi
#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h" //freeRTOS items to be used
#include "wi-fi-config.h"
// RC-S620S
#include "RCS620S_ESP32.h"

// ScreenManagerTask - TFT
TFT_eSPI tft = TFT_eSPI();
typedef struct _ScreenStatus
{
  bool shiyou = false;
}ScreenStatus;
QueueHandle_t xQueueScreenStatus;

// MessageManagerTask - WiFi
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char *mqtt_broker = MQTT_BROKER;
const int mqtt_port = MQTT_PORT;
WiFiClient   wifiClient; // do the WiFi instantiation thing
PubSubClient MQTTclient( mqtt_broker, mqtt_port, wifiClient ); //do the MQTT instantiation thing
SemaphoreHandle_t sema_MQTT_KeepAlive;
typedef struct _AuthRequest
{
  unsigned char id[5];
}AuthRequest;
QueueHandle_t xMessageManagerRequest;

// NFC-Communication-Manager Task
#define COMMAND_TIMEOUT  400
#define POLLING_INTERVAL 500
RCS620S rcs620s(Serial2);


void ScreenManagerTask( void *pvParameters ) {
  tft.begin();
  tft.setRotation(3);
  
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF7);
  
  tft.fillScreen(TFT_BLACK);
  tft.drawString(" ID:-", 40, 20, GFXFF);
  tft.drawString("sts:-", 40, 60, GFXFF);
  tft.drawString("s_T:-", 40, 100, GFXFF);
  tft.drawString("e_T:-", 40, 140, GFXFF);

  ScreenStatus screenStatus;

  for(;;)
  {
    if(pdTRUE == xQueueReceive(xQueueScreenStatus,&screenStatus,portMAX_DELAY)){
      tft.fillScreen(TFT_BLACK);
      tft.drawString(" ID:13045", 40, 20, GFXFF);
      tft.drawString("s_T:13:00", 40, 100, GFXFF);
      tft.drawString("e_T:14:00", 40, 140, GFXFF);
      if(!screenStatus.shiyou){
          tft.drawString("sts:yoyaku", 40, 60, GFXFF);
      }else{
          tft.drawString("sts:shiyou", 40, 60, GFXFF);
      }
    }
    Serial.println("[ScreenManagerTask] hello");
    vTaskDelay(1000);
  }
}

void MessageManagerTask( void *pvParameters )
{
  sema_MQTT_KeepAlive   = xSemaphoreCreateBinary();
  xSemaphoreGive( sema_MQTT_KeepAlive ); // found keep alive can mess with a publish, stop keep alive during publish
  MQTTclient.setKeepAlive( 90 ); // setting keep alive to 90 seconds makes for a very reliable connection, must be set before the 1st connection is made.
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 250; //delay for ms
  for (;;)
  {
    //check for a is-connected and if the WiFi 'thinks' its connected, found checking on both is more realible than just a single check
    if ( (wifiClient.connected()) && (WiFi.status() == WL_CONNECTED) )
    {
      // httpによる要求応答の評価
      AuthRequest authRequest;
      if(pdTRUE == xQueueReceive(xMessageManagerRequest,&authRequest,0)) // mqttを待たせたくないため、0秒で即返却とする
      {
        // mqttを一度切る
        disconnectToMQTT();
        
        Serial.print("message receive:");
        for (int i = 0; i < 5; i++) {
          Serial.print(authRequest.id[i], HEX);
          Serial.print(" ");
        }
        Serial.println();

        // http
        HTTPClient http;
        http.begin(wifiClient,"192.168.50.225",5000,"/?id=13045",false); // sample url
        Serial.print("[HTTP] GET...\n");
        // start connection and send HTTP header
        int httpCode = http.GET();
        Serial.printf("[HTTP] GET...done %d\n",httpCode);
        if(httpCode > 0) {
            // HTTP header has been send and Server response header has been handled
            Serial.printf("[HTTP] GET... code: %d\n", httpCode);
  
            // file found at server
            if(httpCode == HTTP_CODE_OK) {
              StaticJsonDocument<200> doc;
              
              DeserializationError error = deserializeJson(doc, http.getString());
            
              // パースが成功したか確認。できなきゃ終了
              if (error) {
                Serial.print(F("deserializeJson() failed: "));
                Serial.println(error.f_str());
              }else{
                const char* jsonName = doc["result"];
                Serial.printf("jsonName = %s\n",jsonName);
              }
            }else{
              Serial.printf("[HTTP] parse failed");
            }
        } else {
            Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
        }

        // mqttが切れているのでつなげる
        connectToMQTT();
      }else{
        // mqttによる通知の評価
        xSemaphoreTake( sema_MQTT_KeepAlive, portMAX_DELAY ); // whiles MQTTlient.loop() is running no other mqtt operations should be in process
        MQTTclient.loop();
        xSemaphoreGive( sema_MQTT_KeepAlive );
      }
    }
    else {
      log_i( "MQTT keep alive found MQTT status % s WiFi status % s", String(wifiClient.connected()), String(WiFi.status()) );
      if ( !(wifiClient.connected()) || !(WiFi.status() == WL_CONNECTED) )
      {
        connectToWiFi();
      }
      connectToMQTT();
    }
    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
  vTaskDelete ( NULL );
}

void connectToMQTT()
{
  byte mac[5]; // create client ID from mac address
  WiFi.macAddress(mac); // get mac address
  String clientID = String(mac[0]) + String(mac[4]) ; // use mac address to create clientID
  while ( !MQTTclient.connected() )
  {
    MQTTclient.connect( "esp8266-client" );
    vTaskDelay( 250 );
  }
  MQTTclient.setCallback( mqttCallback );
  MQTTclient.subscribe  ( "test" );
} //void connectToMQTT()

void disconnectToMQTT()
{
  while(MQTTclient.connected())
  {
    MQTTclient.disconnect();
    vTaskDelay( 1 );
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);
    Serial.print("Message:");
    for (int i = 0; i < length; i++) {
        Serial.print((char) payload[i]);
    }

    if(payload[0] == 'y'){
      Serial.println("[mqtt] send to screen task yoyaku message");
      ScreenStatus screenStatus;
      screenStatus.shiyou = false;
      xQueueSend(xQueueScreenStatus, &screenStatus,0);
    }else if(payload[0] == 's'){
      Serial.println("[mqtt] send to screen task shiyou message");
      ScreenStatus screenStatus;
      screenStatus.shiyou = true;
      xQueueSend(xQueueScreenStatus, &screenStatus,0);
    }
    
    /*
    Serial.print("Message:");
    Serial.println();
    Serial.println("-----------------------");
    */
}

void IRAM_ATTR WiFiEvent(WiFiEvent_t event)
{
  switch (event) {
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("Connected to WiFi access point");
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Disconnected from WiFi access point");
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      Serial.println("WiFi client disconnected");
      break;
    default: break;
  }
} // void IRAM_ATTR WiFiEvent(WiFiEvent_t event)

void connectToWiFi()
{
  int TryCount = 0;
  while ( WiFi.status() != WL_CONNECTED )
  {
    TryCount++;
    WiFi.disconnect();
    WiFi.begin( ssid, password );
    vTaskDelay( 4000 );
    if ( TryCount == 10 )
    {
      ESP.restart();
    }
  }
  WiFi.onEvent( WiFiEvent );
}


void  NfcCommunicationManager( void *param )
{
  Serial2.begin(115200);
  delay(1000);

  int ret = rcs620s.initDevice();
  Serial.print("RCS620S Init = ");
  Serial.println(ret);
  
  uint8_t data;
  
  while( 1 ) {
    
    rcs620s.timeout = COMMAND_TIMEOUT;
    int ret = rcs620s.polling();
    // Serial.print("RCS620S polling = ");
    // Serial.println(ret);
    if (ret) {

      bool cardReadResult = false;
      uint8_t res[RCS620S_MAX_CARD_RESPONSE_LEN];
      uint8_t resLen = 0;

      cardReadResult = rcs620s.readCardId(
        rcs620s.pmm,
        res,
        &resLen);

      if(cardReadResult){
        for (int i = 0; i < resLen; i++) {
          Serial.print(res[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
        
        AuthRequest authRequest;
        memcpy(authRequest.id,res,5);
        xQueueSend(xMessageManagerRequest, &authRequest,0);
      }
      
      while(1){
        if(rcs620s.polling() == 0){
          break;
        }
        vTaskDelay(1);
      }
    }
  
    rcs620s.rfOff();
    vTaskDelay(POLLING_INTERVAL);
  }
}



void setup() {
  Serial.begin(115200);

  // メールボックスの作成
  xQueueScreenStatus = xQueueCreate(5, sizeof(ScreenStatus)); // タスク内で生成してしまうと、生成タスクよりも前に、別のタスクが送信してしまう可能性があるためここで生成する
  xMessageManagerRequest = xQueueCreate(5, sizeof(AuthRequest)); // タスク内で生成してしまうと、生成タスクよりも前に、別のタスクが送信してしまう可能性があるためここで生成する
  
  
  xTaskCreateUniversal( ScreenManagerTask, "ScreenManagerTask", 15000, NULL, 6, NULL, CONFIG_ARDUINO_RUNNING_CORE );
  xTaskCreateUniversal( MessageManagerTask, "MessageManagerTask", 15000, NULL, 7, NULL, CONFIG_ARDUINO_RUNNING_CORE );
  xTaskCreateUniversal( NfcCommunicationManager, "NfcCommunicationManager", 15000, NULL, 5, NULL, CONFIG_ARDUINO_RUNNING_CORE );
}
void loop() {
  // Serial.println("[main] hello");
  vTaskDelay(1000);
}


#ifndef LOAD_GLCD
//ERROR_Please_enable_LOAD_GLCD_in_User_Setup
#endif

#ifndef LOAD_GFXFF
ERROR_Please_enable_LOAD_GFXFF_in_User_Setup!
#endif
