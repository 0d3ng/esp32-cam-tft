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

#include "img_converters.h" //影像格式轉換函
#include "fb_gfx.h"         //影像繪圖函式
#include "fd_forward.h"     //人臉偵測函式
#include "fr_forward.h"     //人臉辨識函式
#include "image_util.h"

#include <TJpg_Decoder.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#define GFXFF 1
#define FSB9 &FreeSerifBold9pt7b

TFT_eSPI tft = TFT_eSPI();

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

#define FACE_COLOR_WHITE 0x00FFFFFF
#define FACE_COLOR_BLACK 0x00000000
#define FACE_COLOR_RED 0x000000FF
#define FACE_COLOR_GREEN 0x0000FF00
#define FACE_COLOR_BLUE 0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)

uint16_t dmaBuffer[16 * 16];

// draw test on TFT
void tft_drawtext(int16_t x, int16_t y, String text, uint8_t font_size, uint16_t color)
{
  tft.setCursor(x, y);
  tft.setTextSize(font_size); // font size 1 = 6x8, 2 = 12x16, 3 = 18x24
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(strcpy(new char[text.length() + 1], text.c_str()));
}

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap)
{
  // Stop further decoding as image is running off bottom of screen
  if (y >= tft.height())
    return 0;

  // This function will clip the image block rendering automatically at the TFT boundaries
  tft.pushImage(x, y, w, h, bitmap);

  // This might work instead if you adapt the sketch to use the Adafruit_GFX library
  // tft.drawRGBBitmap(x, y, bitmap, w, h);

  // Return 1 to decode next block
  return 1;
}

static void draw_face_boxes(dl_matrix3du_t *image_matrix, box_array_t *boxes, int face_id)
{
  int x, y, w, h, i;
  uint32_t color = FACE_COLOR_YELLOW;
  if (face_id < 0)
  {
    color = FACE_COLOR_RED;
  }
  else if (face_id > 0)
  {
    color = FACE_COLOR_GREEN;
  }
  fb_data_t fb;
  fb.width = image_matrix->w;
  fb.height = image_matrix->h;
  fb.data = image_matrix->item;
  fb.bytes_per_pixel = 3;
  fb.format = FB_BGR888;
  Serial.printf("Boxes: %d\n", boxes->len);
  for (i = 0; i < boxes->len; i++)
  {
    // rectangle box
    x = (int)boxes->box[i].box_p[0];
    y = (int)boxes->box[i].box_p[1];
    w = (int)boxes->box[i].box_p[2] - x + 1;
    h = (int)boxes->box[i].box_p[3] - y + 1;
    fb_gfx_drawFastHLine(&fb, x, y, w, color);
    fb_gfx_drawFastHLine(&fb, x, y + h - 1, w, color);
    fb_gfx_drawFastVLine(&fb, x, y, h, color);
    fb_gfx_drawFastVLine(&fb, x + w - 1, y, h, color);
    // fb_gfx_printf(&fb, (x + (w / 2)), y + (h / 2), FACE_COLOR_GREEN, "%d", counter + 1);
    Serial.printf("Boxes x:%d y:%d w:%d h:%d\n", x, y, w, h);
    // int divL = h - w + 1;
    // w = h;

    // unsigned short cropLeft = x - (divL / 2);
    // unsigned short cropRight = fb.width - x - w + (divL / 2);
    // unsigned short cropTop = y;
    // unsigned short cropBottom = fb.height - y - h;

    // *cropped = *temp;
    // crop_image(cropped, temp, cropLeft, cropRight, cropTop, cropBottom);
    // printf("cropped[%d %d] temp[%d %d]\n", cropped->width, cropped->height, temp->width, temp->height);
    // uint8_t *jpg_buf = (uint8_t *)malloc(sizeof(uint8_t));
    // if (jpg_buf == NULL)
    // {
    //   printf("Malloc failed to allocate buffer for JPG.\n");
    // }
    // else
    // {
    //   size_t jpg_size = 0;

    //   // Convert the RAW image into JPG
    //   // The parameter "31" is the JPG quality. Higher is better.
    //   fmt2jpg(cropped->buf, cropped->len, cropped->width, cropped->height, cropped->format, 31, &jpg_buf, &jpg_size);
    //   String buffer = base64::encode(jpg_buf, jpg_size);
    //   client.publish("esp32/face", buffer.c_str());
    //   printf("Converted JPG size: %d bytes \n\n", jpg_size);
    //   free(jpg_buf);
    //   jpg_buf = NULL;
    //   counter++;
    // }

#if 0
        // landmark
        int x0, y0, j;
        for (j = 0; j < 10; j+=2) {
            x0 = (int)boxes->landmark[i].landmark_p[j];
            y0 = (int)boxes->landmark[i].landmark_p[j+1];
            fb_gfx_fillRect(&fb, x0, y0, 3, 3, color);
        }
#endif
  }
}

static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.type = FAST;
  mtmn_config.min_face = 60; // 80 default
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();

static esp_err_t stream_handler()
{
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];
  dl_matrix3du_t *image_matrix = NULL;
  bool detected = false;
  int face_id = 0;
  int64_t fr_start = 0;
  int64_t fr_ready = 0;
  int64_t fr_face = 0;
  int64_t fr_recognize = 0;
  int64_t fr_encode = 0;

  static int64_t last_frame = 0;
  if (!last_frame)
  {
    last_frame = esp_timer_get_time();
  }

  while (true)
  {
    detected = false;
    face_id = 0;

    fb = esp_camera_fb_get();
    if (!fb)
    {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    }
    else
    {

      // if (!client.connected())
      // {
      //   reconnect();
      // }
      // if (!client.loop())
      // {
      //   client.connect("ESP32-CAM-Client");
      // }

      fr_start = esp_timer_get_time();
      fr_ready = fr_start;
      fr_face = fr_start;
      fr_encode = fr_start;
      fr_recognize = fr_start;
      Serial.printf("img format:%d w:%d h:%d\n", fb->format, fb->width, fb->height);
      if (fb->width >= 400)
      {
        if (fb->format != PIXFORMAT_JPEG)
        {
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted)
          {
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        }
        else
        {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
      else
      {

        image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
        // Serial.println("Trap 1");
        if (!image_matrix)
        {
          Serial.println("dl_matrix3du_alloc failed");
          res = ESP_FAIL;
        }
        else
        {
          // Serial.println("Trap 2");
          if (!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item))
          {
            Serial.println("fmt2rgb888 failed");
            res = ESP_FAIL;
          }
          else
          {
            // Serial.println("Trap 3");
            fr_ready = esp_timer_get_time();
            box_array_t *net_boxes = NULL;
            net_boxes = face_detect(image_matrix, &mtmn_config);

            fr_face = esp_timer_get_time();
            fr_recognize = fr_face;
            if (net_boxes || fb->format != PIXFORMAT_JPEG)
            {
              if (net_boxes)
              {
                Serial.println("Detection.");
                // digitalWrite(LED_BUILTIN, HIGH);
                detected = true;
                fr_recognize = esp_timer_get_time();
                draw_face_boxes(image_matrix, net_boxes, 1);
                dl_lib_free(net_boxes->score);
                dl_lib_free(net_boxes->box);
                dl_lib_free(net_boxes->landmark);
                dl_lib_free(net_boxes);
                net_boxes = NULL;
                Serial.println("Detection finished.");
              }
              else
              {
                // digitalWrite(LED_BUILTIN, LOW);
                // Serial.println("No Detection.");
                // counter = 0;
              }
              if (!fmt2jpg(image_matrix->item, fb->width * fb->height * 3, fb->width, fb->height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len))
              {
                Serial.println("fmt2jpg failed");
                res = ESP_FAIL;
              }
              esp_camera_fb_return(fb);
              fb = NULL;
            }
            else
            {
              _jpg_buf = fb->buf;
              _jpg_buf_len = fb->len;
            }
            fr_encode = esp_timer_get_time();
          }
          dl_matrix3du_free(image_matrix);
        }
      }
    }
    if (res == ESP_OK)
    {
      TJpgDec.drawJpg(0, 6, (const uint8_t *)_jpg_buf, _jpg_buf_len);
    }
    if (fb)
    {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    }
    else if (_jpg_buf)
    {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    // int64_t fr_end = esp_timer_get_time();

    // int64_t ready_time = (fr_ready - fr_start) / 1000;
    // int64_t face_time = (fr_face - fr_ready) / 1000;
    // int64_t recognize_time = (fr_recognize - fr_face) / 1000;
    // int64_t encode_time = (fr_encode - fr_recognize) / 1000;
    // int64_t process_time = (fr_encode - fr_start) / 1000;

    // int64_t frame_time = fr_end - last_frame;
    // last_frame = fr_end;
    // frame_time /= 1000;
    // uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
    // Serial.printf("MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps), %u+%u+%u+%u=%u %s%d\n",
    //               (uint32_t)(_jpg_buf_len),
    //               (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time,
    //               avg_frame_time, 1000.0 / avg_frame_time,
    //               (uint32_t)ready_time, (uint32_t)face_time, (uint32_t)recognize_time, (uint32_t)encode_time, (uint32_t)process_time,
    //               (detected) ? "DETECTED " : "", face_id);
  }

  last_frame = 0;
  return res;
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
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_QVGA;
  // config.jpeg_quality = 10; // 0-63 lower number means higher quality
  // config.fb_count = 1;

  if (psramFound())
  { // 是否有PSRAM(Psuedo SRAM)記憶體IC
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 10;
    config.fb_count = 1;
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

  Serial.println("INIT DISPLAY");
  tft.begin();
  tft.setRotation(1); // 0 & 2 Portrait. 1 & 3 landscape
  tft.setTextColor(0xFFFF, 0x0000);
  tft.fillScreen(ST7735_BLACK);
  tft.setFreeFont(FSB9);
  // tft.setCursor(0, 0);
  // tft.println("Hello TFT");
  sleep(3);
  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(true);
  TJpgDec.setCallback(tft_output);

  stream_handler();
}

void loop()
{
  delay(10);
}