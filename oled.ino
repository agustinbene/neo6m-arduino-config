
#ifdef OLED_DEBUG
#include "heltec.h"
#include "Arduino.h"

//rotate only for GEOMETRY_128_64
 SSD1306Wire display(0x3c, SDA_OLED, SCL_OLED, RST_OLED);

 void oled() {

  delay(100);

  display.init();
  display.clear();
  display.display();
  
  display.setContrast(255);
  
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.clear();
  display.display();
  //display.screenRotate(ANGLE_0_DEGREE);
  display.setFont(ArialMT_Plain_10);
  display.drawString(10, 0, "Lat:");
  display.drawString(40, 0, String(latitude));
  display.drawString(10, 10, "Lon:");
  display.drawString(40, 10, String(longitude));
  display.drawString(10, 20, "Bat:");
  display.drawString(40, 20, String(ADC));
  display.drawString(10, 30, "Var:");
  display.drawString(40, 30, String(varianza));
  display.drawString(10, 40, "Tem:");
  display.drawString(40, 40, String(temperature));
  display.display();
}


#endif
