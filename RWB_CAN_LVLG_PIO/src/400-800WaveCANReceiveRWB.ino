#include <Arduino.h>
#include <math.h>
#include <esp_display_panel.hpp>

#include <lvgl.h>
#include "lvgl_v8_port.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "ui_font_Hollow85.h"
#include "ui_font_Hollow38.h"
#include "ui_font_t20.h"

#include "waveshare_twai_port.h"
#include "driver/twai.h"

using namespace esp_panel::drivers;
using namespace esp_panel::board;
// Combined Buttons - Marker Byte 3 (0xFA) control
#define BTN_COMBO_EMERGENCY_STOP  0x01  // Bit 0: Emergency Stop
#define BTN_COMBO_CALIBRATION     0x02  // Bit 1: Calibration Mode
#define BTN_COMBO_LOGGING         0x04  // Bit 2: Data Logging
#define BTN_COMBO_TEST_MODE       0x08  // Bit 3: Test Mode
/***************************************************
 * BUTTON CONFIGURATION - LINK TO MARKER BITS
 ***************************************************/
// Left Button Functions - Marker Byte 1 (0xFC) control
#define BTN_LEFT_LAUNCH_CONTROL   0x01  // Bit 0: Launch Control
#define BTN_LEFT_SPORT_MODE       0x02  // Bit 1: Sport Mode
#define BTN_LEFT_MAP_SWITCH       0x04  // Bit 2: Map Switching
#define BTN_LEFT_TRACTION_OFF     0x08  // Bit 3: Traction Control Off

// Right Button Functions - Marker Byte 2 (0xFB) control  
#define BTN_RIGHT_ANTI_LAG        0x01  // Bit 0: Anti-Lag System
#define BTN_RIGHT_FLAT_SHIFT      0x02  // Bit 1: Flat Shift
#define BTN_RIGHT_AUTO_BLIP       0x04  // Bit 2: Auto Blip
#define BTN_RIGHT_DIAG_MODE       0x08  // Bit 3: Diagnostic Mode

// Combined Buttons - Marker Byte 3 (0xFA) control
#define BTN_COMBO_EMERGENCY_STOP  0x01  // Bit 0: Emergency Stop
#define BTN_COMBO_CALIBRATION     0x02  // Bit 1: Calibration Mode
#define BTN_COMBO_LOGGING         0x04  // Bit 2: Data Logging
#define BTN_COMBO_TEST_MODE       0x08  // Bit 3: Test Mode

/***************************************************
 * BUILD-TIME FLAGS
 ***************************************************/
#define TEST_RPM_MODE        1        // 1 = fake RPM, 0 = real CAN
#define ESPNOW_WIFI_CHANNEL  1
/***************************************************
 * BUTTON-CONTROLLED MARKER BYTES
 ***************************************************/
uint8_t custom_marker_1 = 0xFC;  // Default marker byte 1
uint8_t custom_marker_2 = 0xFB;  // Default marker byte 2  
uint8_t custom_marker_3 = 0xFA;  // Default marker byte 3

/***************************************************
 * ECU STRUCT
 ***************************************************/
bool can_ok = false;
uint32_t can_rx_count = 0;

struct ECUData {
  uint16_t rpm;
  float    battery_voltage;
  float    water_temp;
  float    lambda;
  uint16_t errors;
  bool     data_valid;
  uint32_t last_update;
} ecu;

/***************************************************
 * CONSTANTS / SMOOTHING
 ***************************************************/
#define RPM_SMOOTHING      0.3f
#define TEMP_SMOOTHING     0.1f
#define VOLTAGE_SMOOTHING  0.2f
#define LAMBDA_SMOOTHING   0.2f
/***************************************************
 * BUTTON STATE TRACKING
 ***************************************************/
uint8_t left_button_flags = 0;
uint8_t right_button_flags = 0;
uint8_t combo_button_flags = 0;
static lv_obj_t *btn_left;
static lv_obj_t *btn_right;
static lv_obj_t *btn_left_label;
static lv_obj_t *btn_right_label;

bool btn_left_on  = false;
bool btn_right_on = false;


/***************************************************
 * BAR SCALING — FULL BAR AT THIS RPM
 ***************************************************/
#define RPM_BAR_MAX_RPM_DISPLAY  8000  // full bar when rpm >= this

const uint32_t UI_UPDATE_INTERVAL_MS = 100;

static inline int16_t S16BE(const uint8_t *p) {
  return (int16_t)((uint16_t(p[0]) << 8) | uint16_t(p[1]));
}
static inline uint16_t U16BE(const uint8_t *p) {
  return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
}

bool isValidRPM(uint16_t r)  { return (r <= RPM_BAR_MAX_RPM_DISPLAY); }
bool isValidTemp(float t)    { return (t > -20 && t < 200); }
bool isValidBatt(float v)    { return (v > 8.0f && v < 16.5f); }
bool isValidLambda(float l)  { return (l >= 0.5f && l <= 2.0f); }

/***************************************************
 * ESP-NOW
 ***************************************************/
uint8_t espnow_broadcast_addr[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static uint16_t espnow_last_rpm = 0;
static uint32_t espnow_last_send_ms = 0;

const uint32_t ESPNOW_MIN_INTERVAL_MS = 50;

void init_espnow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] Init failed");
    return;
  }
  Serial.println("[ESPNOW] Init OK");

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, espnow_broadcast_addr, 6);
  peer.channel = ESPNOW_WIFI_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

void espnow_send_rpm(uint16_t rpm) {
  uint32_t now = millis();
  if (now - espnow_last_send_ms < ESPNOW_MIN_INTERVAL_MS &&
      rpm == espnow_last_rpm) return;

  espnow_last_send_ms = now;
  espnow_last_rpm     = rpm;

  uint8_t payload[4] = {0x00,0x00, uint8_t(rpm>>8), uint8_t(rpm&0xFF)};
  esp_now_send(espnow_broadcast_addr, payload, 4);
}

/***************************************************
 * CAN DECODER
 ***************************************************/
void processECUData_770(const twai_message_t &msg) {
  uint32_t now = millis();
  const uint8_t *d = msg.data;

  // ---- BATTERY ----
  if (d[0]==0x00 && d[2]==0x00 && d[3]==0x00 &&
      d[6]==0x00 && d[7]==0x00 && (d[4]!=0x00||d[5]!=0x00))
  {
    float newBat = S16BE(&d[4]) * 0.005f;
    if (isValidBatt(newBat)) {
      if (ecu.battery_voltage == 0.0f)
        ecu.battery_voltage = newBat;
      else
        ecu.battery_voltage += VOLTAGE_SMOOTHING * (newBat - ecu.battery_voltage);

      ecu.data_valid = true;
      ecu.last_update = now;
    }
    return;
  }

  // ---- ERRORS ----
  if (d[0]==0x00 && d[1]==0x00 && d[5]==0x41 && d[6]==0x1E && d[7]==0xFC) {
    ecu.errors = d[4];
    ecu.data_valid = true;
    ecu.last_update = now;
    return;
  }

  // ---- LAMBDA ----
  if (d[0]==0xFB && d[1]==0xFA) {
    float new_lambda = d[2] / 100.0f;
    if (isValidLambda(new_lambda)) {
      if (ecu.lambda == 0.0f) ecu.lambda = new_lambda;
      else ecu.lambda += LAMBDA_SMOOTHING * (new_lambda - ecu.lambda);

      ecu.data_valid = true;
      ecu.last_update = now;
    }
    return;
  }

  // ---- RPM + TEMP ----
  if (d[4]==0 && d[5]==0 && d[6]==0 && d[7]==0 &&
      (d[2]!=0 || d[3]!=0))
  {
    uint16_t new_rpm = U16BE(&d[0]);
    float new_temp = S16BE(&d[2]) * 0.1f;

    if (isValidRPM(new_rpm)) {
      if (ecu.rpm == 0) ecu.rpm = new_rpm;
      else ecu.rpm += RPM_SMOOTHING * (new_rpm - ecu.rpm);
      espnow_send_rpm(ecu.rpm);
    }

    if (isValidTemp(new_temp)) {
      if (ecu.water_temp == 0.0f) ecu.water_temp = new_temp;
      else ecu.water_temp += TEMP_SMOOTHING * (new_temp - ecu.water_temp);
    }

    ecu.data_valid = true;
    ecu.last_update = now;
    return;
  }

  // ---- ONLY RPM ----
  if (d[2]==0 && d[3]==0 && d[4]==0 && d[5]==0 &&
      d[6]==0 && d[7]==0)
  {
    uint16_t new_rpm = U16BE(&d[0]);

    if (isValidRPM(new_rpm)) {
      if (ecu.rpm == 0) ecu.rpm = new_rpm;
      else ecu.rpm += RPM_SMOOTHING * (new_rpm - ecu.rpm);
      espnow_send_rpm(ecu.rpm);

      ecu.data_valid = true;
      ecu.last_update = millis();
    }
    return;
  }
}

/***************************************************
 * LVGL OBJECTS
 ***************************************************/
static lv_obj_t *label_title;
static lv_obj_t *label_rpm;
static lv_obj_t *rpm_bar;
static lv_obj_t *label_batt;
static lv_obj_t *label_temp;
static lv_obj_t *label_lambda;
static lv_obj_t *label_status;
/***************************************************
 * BUTTON EVENT HANDLERS WITH MARKER CONTROL
 ***************************************************/
static void update_marker_bytes() {
  // Marker Byte 1 (0xFC) - Left Button Functions
  custom_marker_1 = 0xFC;  // Base value
  custom_marker_1 |= left_button_flags;  // Add button flags
  
  // Marker Byte 2 (0xFB) - Right Button Functions  
  custom_marker_2 = 0xFB;  // Base value
  custom_marker_2 |= right_button_flags;  // Add button flags
  
  // Marker Byte 3 (0xFA) - Combined Button Functions
  custom_marker_3 = 0xFA;  // Base value
  custom_marker_3 |= combo_button_flags;  // Add combo flags
  
  Serial.printf("[MARKERS] Updated: 0x%02X 0x%02X 0x%02X\n", 
                custom_marker_1, custom_marker_2, custom_marker_3);
}

static void btn_left_event_handler(lv_event_t * e) {
  lv_obj_t * btn = lv_event_get_target(e);
  btn_left_on = lv_obj_has_state(btn, LV_STATE_CHECKED);
  
  if (btn_left_on) {
    // Left button pressed - activate functions
    left_button_flags |= BTN_LEFT_SPORT_MODE;  // Example: Sport mode
    lv_label_set_text(btn_left_label, "Boom");
  } else {
    // Left button released - deactivate functions  
    left_button_flags &= ~BTN_LEFT_SPORT_MODE;
    lv_label_set_text(btn_left_label, "mode 1");
  }
  
  update_marker_bytes();
  
  Serial.printf("[BTN] Left: %s, Flags: 0x%02X\n", 
                btn_left_on ? "ON" : "OFF", left_button_flags);
}

static void btn_right_event_handler(lv_event_t * e) {
  lv_obj_t * btn = lv_event_get_target(e);
  btn_right_on = lv_obj_has_state(btn, LV_STATE_CHECKED);
  
  if (btn_right_on) {
    // Right button pressed - activate functions
    right_button_flags |= BTN_RIGHT_ANTI_LAG;  // Example: Anti-lag
    lv_label_set_text(btn_right_label, "Fire");
  } else {
    // Right button released - deactivate functions
    right_button_flags &= ~BTN_RIGHT_ANTI_LAG;
    lv_label_set_text(btn_right_label, "mode 2");  
  }
  
  update_marker_bytes();
  
  Serial.printf("[BTN] Right: %s, Flags: 0x%02X\n",
                btn_right_on ? "ON" : "OFF", right_button_flags);
}

/***************************************************
 * COMBINATION BUTTON DETECTION
 ***************************************************/
void check_combination_buttons() {
  static bool last_combo_state = false;
  bool current_combo_state = (btn_left_on && btn_right_on);
  
  if (current_combo_state != last_combo_state) {
    if (current_combo_state) {
      // Both buttons pressed - activate combo functions
      combo_button_flags |= BTN_COMBO_EMERGENCY_STOP;
      Serial.println("[COMBO] Emergency stop activated!");
    } else {
      // Combo released
      combo_button_flags &= ~BTN_COMBO_EMERGENCY_STOP;
      Serial.println("[COMBO] Emergency stop deactivated");
    }
    
    update_marker_bytes();
    last_combo_state = current_combo_state;
  }
}

bool verifyChecksum(const uint8_t *packet) {
  uint8_t checksum = 0;
  for (int i = 0; i < 34; i++) {
    checksum += packet[i];
  }
  return checksum == packet[34];
}

/***************************************************
 * CREATE UI
 ***************************************************/
void create_dashboard_ui() {
  lv_obj_t *scr = lv_scr_act();

  lv_obj_set_style_bg_color(scr, lv_color_hex(0x050608), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
// ---- TITLE ----
  label_title = lv_label_create(scr);
  lv_obj_set_style_text_font(label_title, &ui_font_t20, 0);
  lv_obj_set_style_text_color(label_title, lv_color_hex(0xAAAAAA), 0);
  lv_label_set_text(label_title, "RAUH Welt BEGRIFF");
  lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 30);

  // ---- TITLE ----
  label_title = lv_label_create(scr);
  lv_obj_set_style_text_font(label_title, &ui_font_Hollow85, 0);
  lv_obj_set_style_text_color(label_title, lv_color_hex(0xAAAAAA), 0);
  lv_label_set_text(label_title, "RWB JANINE / Trijekt");
  lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 70);

  // ---- BIG RPM ----
  label_rpm = lv_label_create(scr);
  lv_obj_set_style_text_font(label_rpm, &ui_font_Hollow85, 0);
  lv_obj_set_style_text_color(label_rpm, lv_color_hex(0xAAAAAA), 0);
  lv_label_set_text(label_rpm, "0000");
  lv_obj_align(label_rpm, LV_ALIGN_CENTER, 0, -35);

  // ==== RPM BAR CONTAINER ====
  lv_obj_t *rpm_cont = lv_obj_create(scr);
  lv_obj_remove_style_all(rpm_cont);
  lv_obj_set_size(rpm_cont, LV_HOR_RES - 40, 50);
  lv_obj_align(rpm_cont, LV_ALIGN_CENTER, 0, 25);

  lv_obj_set_style_bg_color(rpm_cont, lv_color_hex(0x101218), 0);
  lv_obj_set_style_bg_opa(rpm_cont, LV_OPA_90, 0);
  lv_obj_set_style_radius(rpm_cont, 12, 0);
  lv_obj_set_style_border_width(rpm_cont, 1, 0);
  lv_obj_set_style_border_color(rpm_cont, lv_color_hex(0x303848), 0);

  // Padding so bar fills inside the box
  lv_obj_set_style_pad_left(rpm_cont, 8, 0);
  lv_obj_set_style_pad_right(rpm_cont, 8, 0);
  lv_obj_set_style_pad_top(rpm_cont, 6, 0);
  lv_obj_set_style_pad_bottom(rpm_cont, 6, 0);

  // small label "RPM"
  lv_obj_t *lbl_caption = lv_label_create(rpm_cont);
  lv_obj_set_style_text_font(lbl_caption, &ui_font_t20, 0);
  lv_obj_set_style_text_color(lbl_caption, lv_color_hex(0x7A8896), 0);
  lv_label_set_text(lbl_caption, "rpm");
  lv_obj_align(lbl_caption, LV_ALIGN_TOP_LEFT, 0, -8);

  // ==== RPM BAR ====
  rpm_bar = lv_bar_create(rpm_cont);
  lv_bar_set_range(rpm_bar, 0, RPM_BAR_MAX_RPM_DISPLAY);

  // bar fills the inner width of rpm_cont
  lv_obj_set_width(rpm_bar, LV_PCT(100));   // 100% of inner width
  lv_obj_set_height(rpm_bar, 18);
  lv_obj_align(rpm_bar, LV_ALIGN_BOTTOM_MID, 0, 0);

  lv_obj_set_style_radius(rpm_bar, 8, LV_PART_MAIN);
  lv_obj_set_style_bg_color(rpm_bar, lv_color_hex(0x202633), LV_PART_MAIN);

  lv_obj_set_style_radius(rpm_bar, 8, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(rpm_bar, lv_color_hex(0x00FF5A), LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(rpm_bar, LV_OPA_TRANSP, LV_PART_INDICATOR);  // start hidden

  // start at 0 so nothing is shown until we get data
  lv_bar_set_value(rpm_bar, 0, LV_ANIM_OFF);

  // ========== LOWER INFO ==========
  label_batt = lv_label_create(scr);
  lv_obj_set_style_text_font(label_batt, &ui_font_Hollow85, 0);
  lv_obj_set_style_text_color(label_batt, lv_color_hex(0x8D9699), 0);
  lv_label_set_text(label_batt, "Batt: --.-V");
  lv_obj_align(label_batt, LV_ALIGN_BOTTOM_LEFT, 20, -100);

  label_temp = lv_label_create(scr);
  lv_obj_set_style_text_font(label_temp, &ui_font_Hollow85, 0);
  lv_obj_set_style_text_color(label_temp, lv_color_hex(0x8D9699), 0);
  lv_label_set_text(label_temp, "Temp: --.-C");
  lv_obj_align(label_temp, LV_ALIGN_BOTTOM_RIGHT, -30, -110);

  label_lambda = lv_label_create(scr);
  lv_obj_set_style_text_font(label_lambda, &ui_font_Hollow85, 0);
  lv_obj_set_style_text_color(label_lambda, lv_color_hex(0x8D9699), 0);
  lv_label_set_text(label_lambda, "Lamda: -.-");
  lv_obj_align(label_lambda, LV_ALIGN_BOTTOM_LEFT, 20, -35);

  label_status = lv_label_create(scr);
  lv_obj_set_style_text_font(label_status, &ui_font_Hollow38, 0);
  lv_obj_set_style_text_color(label_status, lv_color_hex(0x8D9699), 0);
  lv_label_set_text(label_status, "CAN: WAIT");
  lv_obj_align(label_status, LV_ALIGN_BOTTOM_RIGHT, -110, -55);

  // BUTTONS WITH EVENT HANDLERS
  btn_left = lv_btn_create(scr);
  lv_obj_add_flag(btn_left, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_size(btn_left, 120, 60);
  lv_obj_align(btn_left, LV_ALIGN_LEFT_MID, 20, -50);
  lv_obj_set_style_bg_color(btn_left, lv_color_hex(0x404040), LV_PART_MAIN);
  lv_obj_set_style_bg_color(btn_left, lv_color_hex(0x00FF00), LV_STATE_CHECKED);
  lv_obj_set_style_bg_opa(btn_left, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_radius(btn_left, 10, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_left, btn_left_event_handler, LV_EVENT_VALUE_CHANGED, NULL);
  btn_left_label = lv_label_create(btn_left);
  lv_label_set_text(btn_left_label, "mode 1");
  lv_obj_set_style_text_font(btn_left_label, &ui_font_Hollow38, 0);
  lv_obj_center(btn_left_label);

btn_right = lv_btn_create(scr);
  lv_obj_add_flag(btn_right, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_size(btn_right, 120, 60);
  lv_obj_align(btn_right, LV_ALIGN_RIGHT_MID, -20, -50);
  lv_obj_set_style_bg_color(btn_right, lv_color_hex(0x404040), LV_PART_MAIN);
  lv_obj_set_style_bg_color(btn_right, lv_color_hex(0x00FF00), LV_STATE_CHECKED);
  lv_obj_set_style_bg_opa(btn_right, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_radius(btn_right, 10, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_right, btn_right_event_handler, LV_EVENT_VALUE_CHANGED, NULL);
  btn_right_label = lv_label_create(btn_right);
  lv_label_set_text(btn_right_label, "mode 2");
  lv_obj_set_style_text_font(btn_right_label, &ui_font_Hollow38, 0);
  lv_obj_center(btn_right_label);


}






/***************************************************
 * UPDATE UI
 ***************************************************/
void update_dashboard_ui() {
  static uint16_t last_rpm_disp = 0;
  static float last_batt = -1;
  static float last_temp = -1000;
  static float last_lambda = -1;
  static uint8_t last_err = 255;
  static uint8_t last_band = 255;

  char buf[32];

  uint16_t rpm_disp = ecu.data_valid ? ecu.rpm : 0;

  // ---- RPM ----
  if (rpm_disp != last_rpm_disp) {
    snprintf(buf, sizeof(buf), "%04u", rpm_disp);
    lv_label_set_text(label_rpm, buf);
    last_rpm_disp = rpm_disp;

    uint16_t bar_val = rpm_disp;
    if (bar_val > RPM_BAR_MAX_RPM_DISPLAY)
      bar_val = RPM_BAR_MAX_RPM_DISPLAY;

    lv_bar_set_value(rpm_bar, bar_val, LV_ANIM_OFF);

    // hide bar when rpm is 0
    if (rpm_disp == 0) {
      lv_obj_set_style_bg_opa(rpm_bar, LV_OPA_TRANSP, LV_PART_INDICATOR);
    } else {
      lv_obj_set_style_bg_opa(rpm_bar, LV_OPA_COVER, LV_PART_INDICATOR);
    }

    uint8_t band;
    if (rpm_disp < 5500)      band = 0;
    else if (rpm_disp < 6500) band = 1;
    else                      band = 2;

    if (band != last_band) {
      if (band == 0)
        lv_obj_set_style_bg_color(rpm_bar, lv_color_hex(0x00FF40), LV_PART_INDICATOR);
      else if (band == 1)
        lv_obj_set_style_bg_color(rpm_bar, lv_color_hex(0xFFD000), LV_PART_INDICATOR);
      else
        lv_obj_set_style_bg_color(rpm_bar, lv_color_hex(0xFF0000), LV_PART_INDICATOR);

      last_band = band;
    }
  }

  // ---- Batt ----
  if (fabsf(ecu.battery_voltage - last_batt) > 0.01f) {
    snprintf(buf, sizeof(buf), "Batt: %.2fV", ecu.battery_voltage);
    lv_label_set_text(label_batt, buf);
    last_batt = ecu.battery_voltage;
  }

  // ---- Temp ----
  if (fabsf(ecu.water_temp - last_temp) > 0.1f) {
    snprintf(buf, sizeof(buf), "Temp: %.1fC", ecu.water_temp);
    lv_label_set_text(label_temp, buf);
    last_temp = ecu.water_temp;
  }

  // ---- Lambda ----
  if (fabsf(ecu.lambda - last_lambda) > 0.01f) {
    snprintf(buf, sizeof(buf), "Lam: %.2f", ecu.lambda);
    lv_label_set_text(label_lambda, buf);
    last_lambda = ecu.lambda;
  }

  // ---- Status ----
  uint8_t err_low = ecu.errors & 0xFF;
  static bool last_valid = false;

  if (err_low != last_err || ecu.data_valid != last_valid) {
    snprintf(buf, sizeof(buf),
             ecu.data_valid ? "CAN: OK  E:0x%02X" : "CAN: WAIT  E:0x%02X",
             err_low);
    lv_label_set_text(label_status, buf);

    last_err   = err_low;
    last_valid = ecu.data_valid;
  }
}

/***************************************************
 * SETUP
 ***************************************************/
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== RWB JANINE DASH + ESPNOW ===");

  Board *board = new Board();
  board->init();
  board->begin();

  lvgl_port_init(board->getLCD(), board->getTouch());

  lvgl_port_lock(-1);
  create_dashboard_ui();
  lvgl_port_unlock();

  can_ok = waveshare_twai_init();

  init_espnow();

  ecu = {0,0,0,0,0,false,0};
}

/***************************************************
 * LOOP
 ***************************************************/
void loop() {
  static uint32_t last_ui_update = 0;

#if TEST_RPM_MODE
  static uint32_t last_test = 0;
  static uint16_t fake_rpm = 1000;

  if (millis() - last_test > 1000) {
    last_test = millis();

    ecu.data_valid = true;
    ecu.last_update = millis();
    ecu.rpm = fake_rpm;

    espnow_send_rpm(fake_rpm);

    fake_rpm += 500;
    if (fake_rpm > RPM_BAR_MAX_RPM_DISPLAY) fake_rpm = 1000;
  }
#else
  twai_message_t msg;
  while (twai_receive(&msg,0) == ESP_OK) {
    if (msg.identifier == 0x770)
      processECUData_770(msg);
  }
#endif

  if (ecu.data_valid && millis() - ecu.last_update > 2000) {
    ecu.data_valid = false;
    ecu.rpm = 0;
  }

  if (millis() - last_ui_update > UI_UPDATE_INTERVAL_MS) {
    lvgl_port_lock(-1);
    update_dashboard_ui();
    lvgl_port_unlock();
    last_ui_update = millis();
  }

  delay(1);
}
