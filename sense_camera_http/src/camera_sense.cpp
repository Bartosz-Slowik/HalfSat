#include "camera_sense.h"

#include <Arduino.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>

#ifndef CAM_MOTION_FRIENDLY
#define CAM_MOTION_FRIENDLY 1
#endif
#ifndef CAM_SENSOR_VFLIP
#define CAM_SENSOR_VFLIP 1
#endif
#ifndef CAM_SENSOR_HMIRROR
#define CAM_SENSOR_HMIRROR 0
#endif

#define PWDN_GPIO_NUM (-1)
#define RESET_GPIO_NUM (-1)
#define XCLK_GPIO_NUM (10)
#define SIOD_GPIO_NUM (40)
#define SIOC_GPIO_NUM (39)
#define Y9_GPIO_NUM (48)
#define Y8_GPIO_NUM (11)
#define Y7_GPIO_NUM (12)
#define Y6_GPIO_NUM (14)
#define Y5_GPIO_NUM (16)
#define Y4_GPIO_NUM (18)
#define Y3_GPIO_NUM (17)
#define Y2_GPIO_NUM (15)
#define VSYNC_GPIO_NUM (38)
#define HREF_GPIO_NUM (47)
#define PCLK_GPIO_NUM (13)

static const char *modelName(uint16_t pid) {
  switch (pid) {
    case OV3660_PID:
      return "OV3660";
    case OV2640_PID:
      return "OV2640";
    case OV5640_PID:
      return "OV5640";
    default:
      return "?";
  }
}

bool camera_sense_init(void) {
  camera_config_t config = {};
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

#if CAM_MOTION_FRIENDLY
  config.jpeg_quality = 7;
  static const framesize_t kTrySizes[] = {FRAMESIZE_XGA, FRAMESIZE_HD, FRAMESIZE_VGA};
#else
  config.jpeg_quality = 8;
  static const framesize_t kTrySizes[] = {FRAMESIZE_QXGA, FRAMESIZE_UXGA, FRAMESIZE_SXGA};
#endif
  config.fb_count = 2;
  config.grab_mode = CAMERA_GRAB_LATEST;

#if CONFIG_IDF_TARGET_ESP32S3
  config.fb_location = CAMERA_FB_IN_PSRAM;
#endif

  Serial.printf("[cam] init motion_friendly=%d\n", CAM_MOTION_FRIENDLY);

  esp_err_t err = ESP_FAIL;
  for (size_t i = 0; i < sizeof(kTrySizes) / sizeof(kTrySizes[0]); i++) {
    if (i > 0) {
      esp_camera_deinit();
    }
    config.frame_size = kTrySizes[i];
    err = esp_camera_init(&config);
    if (err == ESP_OK) {
      Serial.printf("[cam] frame_size try %u OK\n", (unsigned)i);
      break;
    }
    Serial.printf("[cam] frame_size try %u err 0x%x\n", (unsigned)i, (unsigned)err);
  }
  if (err != ESP_OK) {
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    if (s->set_vflip) {
      s->set_vflip(s, CAM_SENSOR_VFLIP);
    }
    if (s->set_hmirror) {
      s->set_hmirror(s, CAM_SENSOR_HMIRROR);
    }
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_gain_ctrl(s, 1);
    Serial.printf("[cam] %s pid=0x%04x vflip=%d hmirror=%d\n", modelName(s->id.PID),
                  (unsigned)s->id.PID, CAM_SENSOR_VFLIP, CAM_SENSOR_HMIRROR);

#if CAM_MOTION_FRIENDLY
    if (s->set_gainceiling) {
      s->set_gainceiling(s, GAINCEILING_8X);
    }
    if (s->set_ae_level) {
      int ae = (s->id.PID == OV3660_PID) ? -3 : -2;
      s->set_ae_level(s, ae);
    }
#endif
  }
  return true;
}
