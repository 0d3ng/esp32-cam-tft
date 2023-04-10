#include <Arduino.h>
/*
ESP32-CAM 2.4 inch TFT LCD Display Module (ILI9341, SPI, 240x320)
Author : ChungYi Fu (Kaohsiung, Taiwan)  2020-12-15 00:00
https://www.facebook.com/francefu
Circuit
TFT_MOSI --> IO13
TFT_MISO --> IO12
TFT_SCLK --> IO14
TFT_CS   --> IO15
TFT_DC   --> IO2
TFT_RST  --> IO16 (Don't use IO4)
TFT_VCC  --> 3.3V
TFT_LED  --> 3.3V
TFT_GND  --> GND
Refer to
https://github.com/Bodmer/TFT_eSPI/blob/ef93dbe687b9b429f0b1116a0d4197e00a7fef44/examples/DMA%20test/Flash_Jpg_DMA/Flash_Jpg_DMA.ino
Introduction
http://www.lcdwiki.com/2.8inch_SPI_Module_ILI9341_SKU:MSP2807
Libraries：
https://github.com/Bodmer/TFT_eSPI
https://github.com/Bodmer/TJpg_Decoder
Add the settings at the end of the file.  C:\Users\..\Documents\Arduino\libraries\TFT_eSPI-master\User_Setup.h
#define TFT_MOSI 13
#define TFT_MISO 12
#define TFT_SCLK 14
#define TFT_CS   15
#define TFT_DC   2
#define TFT_RST  16
*/

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include "SPI.h"
#include <JPEGDecoder.h>
#include <TFT_eSPI.h>      // Hardware-specific library
TFT_eSPI tft = TFT_eSPI(); // Invoke custom library
#include <TFT_eFEX.h>
TFT_eFEX  fex = TFT_eFEX(&tft);

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#define FACE_COLOR_WHITE  0x00FFFFFF
#define FACE_COLOR_BLACK  0x00000000
#define FACE_COLOR_RED    0x000000FF
#define FACE_COLOR_GREEN  0x0000FF00
#define FACE_COLOR_BLUE   0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN   (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)

uint16_t dmaBuffer[16 * 16];

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap)
{
  if (y >= tft.height())
    return 0;
  else
  {
    tft.pushImageDMA(x, y, w, h, bitmap, dmaBuffer);
    return 1;
  }
}

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 10; // 0-63 lower number means higher quality
  config.fb_count = 2;

  if (psramFound())
  { // 是否有PSRAM(Psuedo SRAM)記憶體IC
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  tft.begin();
  tft.setRotation(1);  // 0 & 2 Portrait. 1 & 3 landscape
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(35,55);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  // tft.setCursor(0,0);
  // tft.println("Test ESP32 TFT");
  delay(5000);
}

void loop()
{
  camera_fb_t *fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("Camera capture failed");
    return;
  }

  fex.drawJpg((const uint8_t*)fb->buf, fb->len, 0, 6);

  esp_camera_fb_return(fb);
}