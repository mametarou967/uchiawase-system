#include "Free_Fonts.h" // Include the header file attached to this sketch

#include "SPI.h"
#include "TFT_eSPI.h"

TFT_eSPI tft = TFT_eSPI();

void ScreenManagerTask( void *pvParameters ) {
  tft.begin();
  tft.setRotation(3);
  
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFreeFont(FF7);
  
  tft.fillScreen(TFT_BLACK);
  tft.drawString(" ID:13045", 40, 40, GFXFF);
  tft.drawString("sts:yoyaku", 40, 80, GFXFF);
  tft.drawString("s_T:13:00", 40, 120, GFXFF);
  tft.drawString("e_T:14:00", 40, 160, GFXFF);

  for(;;)
  {
    Serial.println("[ScreenManagerTask] hello");
    vTaskDelay(1000);
  }
}

void setup() {
  Serial.begin(115200);
  xTaskCreateUniversal( ScreenManagerTask, "ScreenManagerTask", 15000, NULL, 6, NULL, CONFIG_ARDUINO_RUNNING_CORE );
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
