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

#include "base64.h"
#include "PubSubClient.h"
#include <WiFi.h>
#include "time.h"
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

// const char *ssid = "Uw415 Ph0n3";
// const char *password = "0dengbr0";

const char *ssid = "JTI-POLINEMA";
const char *password = "jtifast!";

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600 * 6;
const int daylightOffset_sec = 3600;

struct tm timeinfo;
String currentTime = "";
String MAC;
WiFiClient espClient;
PubSubClient client(espClient);
camera_fb_t *cropped = (camera_fb_t *)malloc(sizeof(camera_fb_t));

const char *brokerUser = NULL;
const char *brokerPass = NULL;
const char *brokerHost = "192.168.74.14";

unsigned long previousMillis = 0; // Stores last time images was published
const long interval = 1000 * 10;  // Interval at which to publish images
const int IMAGE_WIDTH = 100;      // image crop

void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MAC.c_str(), brokerUser, brokerPass))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");

      delay(5000);
    }
  }
}

void printLocalTime()
{
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  char timeStringBuff[50]; // 50 chars should be enough
  strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);
  // print like "const char*"
  //  Serial.println(timeStringBuff);

  // Optional: Construct String object
  //  String asString(timeStringBuff);
  //  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  /*
    Member  Type  Meaning Range
    tm_sec  int seconds after the minute  0-61*
    tm_min  int minutes after the hour  0-59
    tm_hour int hours since midnight  0-23
    tm_mday int day of the month  1-31
    tm_mon  int months since January  0-11
    tm_year int years since 1900
    tm_wday int days since Sunday 0-6
    tm_yday int days since January 1  0-365
    tm_isdst  int Daylight Saving Time flag
  */
  // currentTime = String(timeinfo.tm_year + 1900) + "/" + String(timeinfo.tm_mon + 1) + "/" + String(timeinfo.tm_mday) + " " + String(timeinfo.tm_hour) + ":" + String(timeinfo.tm_min) + ":" + String(timeinfo.tm_sec);
  currentTime = String(timeStringBuff);
  Serial.println(currentTime);
}

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

static void crop_image(camera_fb_t *dst, camera_fb_t *fb, unsigned short cropLeft, unsigned short cropRight, unsigned short cropTop, unsigned short cropBottom)
{
  unsigned int maxTopIndex = cropTop * fb->width * 2;
  unsigned int minBottomIndex = ((fb->width * fb->height) - (cropBottom * fb->width)) * 2;
  unsigned short maxX = fb->width - cropRight; // In pixels
  unsigned short newWidth = fb->width - cropLeft - cropRight;
  unsigned short newHeight = fb->height - cropTop - cropBottom;
  size_t newJpgSize = newWidth * newHeight * 2;

  unsigned int writeIndex = 0;
  // Loop over all bytes
  for (int i = 0; i < fb->len; i += 2)
  {
    // Calculate current X, Y pixel position
    int x = (i / 2) % fb->width;

    // Crop from the top
    if (i < maxTopIndex)
    {
      continue;
    }

    // Crop from the bottom
    if (i > minBottomIndex)
    {
      continue;
    }

    // Crop from the left
    if (x <= cropLeft)
    {
      continue;
    }

    // Crop from the right
    if (x > maxX)
    {
      continue;
    }

    // If we get here, keep the pixels
    dst->buf[writeIndex++] = dst->buf[i];
    dst->buf[writeIndex++] = dst->buf[i + 1];
  }

  // Set the new dimensions of the framebuffer for further use.
  dst->width = newWidth;
  dst->height = newHeight;
  dst->len = newJpgSize;
}

static void draw_face_boxes(camera_fb_t *temp, dl_matrix3du_t *image_matrix, box_array_t *boxes, int face_id)
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
  unsigned long currentMillis = millis();
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
    int divL = h - w + 1;
    w = h;

    unsigned short cropLeft = x - (divL / 2) - 10;
    unsigned short cropRight = fb.width - x - w + (divL / 2) - 10;
    unsigned short cropTop = y - 10;
    unsigned short cropBottom = fb.height - y - h - 10;

    *cropped = *temp;
    crop_image(cropped, temp, cropLeft, cropRight, cropTop, cropBottom);
    printf("cropped[%d %d] temp[%d %d]\n", cropped->width, cropped->height, temp->width, temp->height);
    uint8_t *jpg_buf = (uint8_t *)malloc(sizeof(uint8_t));
    if (jpg_buf == NULL)
    {
      printf("Malloc failed to allocate buffer for JPG.\n");
    }
    else
    {
      if (currentMillis - previousMillis >= interval && (cropped->width >= IMAGE_WIDTH || cropped->height>=IMAGE_WIDTH))
      {
        size_t jpg_size = 0;

        // Convert the RAW image into JPG
        // The parameter "31" is the JPG quality. Higher is better.
        fmt2jpg(cropped->buf, cropped->len, cropped->width, cropped->height, cropped->format, 31, &jpg_buf, &jpg_size);

        previousMillis = currentMillis;

        String buffer = base64::encode(jpg_buf, jpg_size);
        static char json_response[1024 * 3];
        char *q = json_response;
        *q++ = '{';
        q += sprintf(q, "\"data\":\"%s\",", buffer.c_str());
        q += sprintf(q, "\"ip\":\"%s\",", WiFi.localIP().toString().c_str());
        q += sprintf(q, "\"waktu\":\"%s\"", currentTime.c_str());
        *q++ = '}';
        *q++ = 0;
        printf("Converted JPG size: %d bytes \n\n", jpg_size);
        // counter++;
        // Serial.println(json_response);
        client.publish("/esp32/face", json_response, strlen(json_response));
        //   printf("Converted JPG size: %d bytes \n\n", jpg_size);
        // published = 1;
        Serial.println(currentTime);
        free(jpg_buf);
        jpg_buf = NULL;
      }
    }

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
  mtmn_config.min_face = 80; // 80 default
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

      if (!client.connected())
      {
        reconnect();
      }
      client.loop();

      fr_start = esp_timer_get_time();
      fr_ready = fr_start;
      fr_face = fr_start;
      fr_encode = fr_start;
      fr_recognize = fr_start;
      // Serial.printf("img format:%d w:%d h:%d\n", fb->format, fb->width, fb->height);
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
                draw_face_boxes(fb, image_matrix, net_boxes, 1);
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
      TJpgDec.drawJpg(0, 0, (const uint8_t *)_jpg_buf, _jpg_buf_len);
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

void callback(char *topic, byte *payload, unsigned int length)
{
  // variable StringPayload untuk menyimpan konten paket data yang diterima
  String StringPayload = "";

  // Menjadikan setiap character yang diterima menjadi string utuh
  // melalui proses penggabungan character
  for (int i = 0; i < length; i++)
  {
    StringPayload += (char)payload[i];
  }

  Serial.println("TOPIC: " + String(topic));
  Serial.println("PAYLOAD: " + String(StringPayload));
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

  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s,1);

  for (int i = 0; i < 2; i++)
  {
    WiFi.begin(ssid, password); // 執行網路連線

    delay(1000);
    Serial.println("");
    Serial.print("Connecting to ");
    Serial.println(ssid);

    long int StartTime = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      if ((StartTime + 5000) < millis())
        break; // 等待10秒連線
    }
  }

  Serial.print("ESP Board MAC Address:  ");
  MAC = WiFi.macAddress();
  Serial.println(MAC);

  client.setServer(brokerHost, 1883);
  client.setBufferSize(1024 * 5);
  client.setCallback(callback);
  delay(500);

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  delay(500);

  Serial.println("INIT DISPLAY");
  tft.begin();
  tft.setRotation(1); // 0 & 2 Portrait. 1 & 3 landscape
  tft.setTextColor(0xFFFF, 0x0000);
  tft.fillScreen(ST7735_BLACK);
  tft.setFreeFont(FSB9);
  // tft.setCursor(0, 0);
  // tft.println("Hello TFT");
  sleep(1);
  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(true);
  TJpgDec.setCallback(tft_output);
  printLocalTime();
  sleep(1);
  stream_handler();
}

void loop()
{
  delay(10);
}