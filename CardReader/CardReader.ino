#include <freertos/FreeRTOS.h> 
// TFT
#include "Free_Fonts.h" // Include the header file attached to this sketch
#include "SPI.h"
#include "TFT_eSPI.h"
// Wi-Fi
#include <WiFi.h>
#include <PubSubClient.h>
#include "freertos/FreeRTOS.h" //freeRTOS items to be used
#include "wi-fi-config.h"

// ScreenManagerTask - TFT
TFT_eSPI tft = TFT_eSPI();
typedef struct _ScreenStatus
{
  bool shiyou = false;
}ScreenStatus;
QueueHandle_t xQueueScreenStatus;

// MessageManagerTask - WiFi
const char* ssid     = MY_SSID;
const char* password = MY_PASSWORD;
const char *mqtt_broker = MY_BROKER;
const int mqtt_port = 1883;
WiFiClient   wifiClient; // do the WiFi instantiation thing
PubSubClient MQTTclient( mqtt_broker, mqtt_port, wifiClient ); //do the MQTT instantiation thing
SemaphoreHandle_t sema_MQTT_KeepAlive;

void ScreenManagerTask( void *pvParameters ) {
  tft.begin();
  tft.setRotation(3);
  
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF7);
  
  tft.fillScreen(TFT_BLACK);
  tft.drawString(" ID:-", 40, 40, GFXFF);
  tft.drawString("sts:-", 40, 80, GFXFF);
  tft.drawString("s_T:-", 40, 120, GFXFF);
  tft.drawString("e_T:-", 40, 160, GFXFF);

  ScreenStatus screenStatus;

  for(;;)
  {
    if(pdTRUE == xQueueReceive(xQueueScreenStatus,&screenStatus,portMAX_DELAY)){
      tft.fillScreen(TFT_BLACK);
      tft.drawString(" ID:13045", 40, 40, GFXFF);
      tft.drawString("s_T:13:00", 40, 120, GFXFF);
      tft.drawString("e_T:14:00", 40, 160, GFXFF);
      if(!screenStatus.shiyou){
          tft.drawString("sts:yoyaku", 40, 80, GFXFF);
      }else{
          tft.drawString("sts:shiyou", 40, 80, GFXFF);
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
      xSemaphoreTake( sema_MQTT_KeepAlive, portMAX_DELAY ); // whiles MQTTlient.loop() is running no other mqtt operations should be in process
      MQTTclient.loop();
      xSemaphoreGive( sema_MQTT_KeepAlive );
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




void setup() {
  Serial.begin(115200);

  // メールボックスの作成
  xQueueScreenStatus = xQueueCreate(5, sizeof(ScreenStatus)); // タスク内で生成してしまうと、生成タスクよりも前に、別のタスクが送信してしまう可能性があるためここで生成する
  
  xTaskCreateUniversal( ScreenManagerTask, "ScreenManagerTask", 15000, NULL, 6, NULL, CONFIG_ARDUINO_RUNNING_CORE );
  xTaskCreateUniversal( MessageManagerTask, "MessageManagerTask", 15000, NULL, 7, NULL, CONFIG_ARDUINO_RUNNING_CORE );
}
void loop() {
  Serial.println("[main] hello");
  vTaskDelay(1000);
}


#ifndef LOAD_GLCD
//ERROR_Please_enable_LOAD_GLCD_in_User_Setup
#endif

#ifndef LOAD_GFXFF
ERROR_Please_enable_LOAD_GFXFF_in_User_Setup!
#endif
