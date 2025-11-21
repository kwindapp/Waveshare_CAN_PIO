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
#include "driver/twai.h"   // for twai_message_t

using namespace esp_panel::drivers;
using namespace esp_panel::board;

/***************************************************
 * FLAGS / CAN
 ***************************************************/
bool testMode = false;   // true = simulate values

bool can_ok = false;

// Trijekt Premium – AIM Protocol #1 – single ID
const uint32_t ID_AIM = 0x770;

const uint16_t MAX_RPM         = 8000;
const uint32_t RPM_TIMEOUT_MS  = 2000;

uint32_t lastRpmUpdateMs = 0;
bool     funk_active     = false;

/***************************************************
 * BUTTON-CONTROLLED MARKER BYTES
 ***************************************************/
uint8_t custom_marker_1 = 0xFC;  // Default marker byte 1
uint8_t custom_marker_2 = 0xFB;  // Default marker byte 2  
uint8_t custom_marker_3 = 0xFA;  // Default marker byte 3

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
 * BUTTON STATE TRACKING
 ***************************************************/
uint8_t left_button_flags = 0;
uint8_t right_button_flags = 0;
uint8_t combo_button_flags = 0;

/***************************************************
 * CONFIG: COLORS & THRESHOLDS
 ***************************************************/
#define BATT_GOOD_MIN   12.8f
#define BATT_WARN_MIN   12.3f

#define MOTOR_GOOD_MAX  90.0f
#define MOTOR_WARN_MAX  100.0f

#define BATT_HYST       0.20f
#define MOTOR_HYST      5.0f

#define SMOOTH_ALPHA_BATT   0.15f
#define SMOOTH_ALPHA_MOTOR  0.10f

#define COL_HEX_GOOD    "00FF00"
#define COL_HEX_WARN    "FFD000"
#define COL_HEX_BAD     "FF0000"

#define LABEL_GREY_HEX  0xA8A3A2

// RPM color bands
#define RPM_YELLOW_START 5500
#define RPM_RED_START    6500

const uint32_t UI_UPDATE_INTERVAL_MS    = 20;   // ~50 fps
const uint32_t ESPNOW_SEND_INTERVAL_MS  = 100;
const uint32_t STATS_INTERVAL_MS        = 10000; // 10 seconds

/***************************************************
 * ECU DATA
 ***************************************************/
struct ECUData {
  uint16_t rpm;          // rpm
  float batt;            // V
  float motor;           // °C
  float throttle_dk;     // %
  float throttle_pedal;  // %
  float lambda;          // λ
  bool lambda_valid;
  uint16_t errors;
} ecu;

/***************************************************
 * ESP-NOW PACKET
 ***************************************************/
typedef struct {
  uint16_t rpm;
  float batt;
  float motor;
  float dk;
  float gp;
  uint8_t funk;
  bool btn_left;
  bool btn_right;
  uint8_t marker1;  // Add marker bytes to packet
  uint8_t marker2;
  uint8_t marker3;
} DashPacket;

uint8_t broadcastAddr[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

/***************************************************
 * LVGL OBJECTS
 ***************************************************/
static lv_obj_t *label_title;
static lv_obj_t *label_rpm;
static lv_obj_t *label_scale;
static lv_obj_t *rpm_bar;
static lv_obj_t *label_row1;     // Batt + Motor (Hollow85)
static lv_obj_t *label_row2;     // DK + Lambda (Hollow85)
static lv_obj_t *label_row3;     // GP + Funk (small)

static lv_obj_t *label_rpm_ticks[9];

static lv_obj_t *btn_left;
static lv_obj_t *btn_right;
static lv_obj_t *btn_left_label;
static lv_obj_t *btn_right_label;

bool btn_left_on  = false;
bool btn_right_on = false;

static lv_style_t style_rpm_label;

/***************************************************
 * WORKING AIM DECODER - PROVEN VERSION
 ***************************************************/
static uint8_t stream_buffer[100];
static uint8_t stream_pos = 0;
static uint32_t packet_count = 0;
static uint32_t error_count = 0;
static uint32_t can_rx_count = 0;
static uint32_t aim_message_count = 0;
static uint32_t last_stats_ms = 0;

// helpers: big-endian
static inline int16_t S16BE_bytes(const uint8_t *p) {
  return (int16_t)((uint16_t(p[0]) << 8) | uint16_t(p[1]));
}
static inline uint16_t U16BE_bytes(const uint8_t *p) {
  return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
}

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
    lv_label_set_text(btn_left_label, "SPORT ON");
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
    lv_label_set_text(btn_right_label, "ANTI-LAG");
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

void checkAlternativeValues(const uint8_t *packet) {
  static bool batt_found = false;
  static bool lambda_found = false;
  
  // Battery voltage - FIXED: bytes 12-13 with 0.005V/bit scaling
  if (!batt_found) {
    int16_t batt_pos12 = S16BE_bytes(&packet[12]);
    float battV = batt_pos12 * 0.005f;
    if (battV > 8.0f && battV < 16.0f) {
      ecu.batt = battV;
      batt_found = true;
      Serial.printf("[CAN] Battery found: pos12-13 = %.2fV\n", ecu.batt);
    }
  }
  
  // Lambda - FIXED: bytes 18-19 with 0.005 scaling
  if (!lambda_found) {
    uint16_t lambda_pos18 = U16BE_bytes(&packet[18]);
    float lambda_005 = lambda_pos18 * 0.005f;
    
    if (packet_count == 0) {
      Serial.printf("[CAN] Lambda scaling test: pos18-19=0x%04X -> %.3f\n", lambda_pos18, lambda_005);
    }
    
    if (lambda_005 >= 0.5f && lambda_005 <= 2.0f) {
      ecu.lambda = lambda_005;
      ecu.lambda_valid = true;
      lambda_found = true;
      Serial.printf("[CAN] Lambda found: pos18-19 = %.3f\n", ecu.lambda);
    }
  }
}

void processAIMPacket(const uint8_t *packet) {
  if (!verifyChecksum(packet)) {
    error_count++;
    if (error_count < 5) {
      Serial.println("[CAN] Checksum error");
    }
    return;
  }

  // Use custom marker bytes controlled by buttons
  if (packet[30] != custom_marker_1 || 
      packet[31] != custom_marker_2 || 
      packet[32] != custom_marker_3) {
    error_count++;
    if (error_count < 5) {
      Serial.println("[CAN] Header error - custom markers don't match");
    }
    return;
  }

  // RPM (bytes 0-1) - 1 RPM/bit
  uint16_t rpm_raw = U16BE_bytes(&packet[0]);
  if (rpm_raw > MAX_RPM) rpm_raw = MAX_RPM;
  ecu.rpm = rpm_raw;
  lastRpmUpdateMs = millis();

  // Water Temperature (bytes 8-9) - 0.1°C/bit
  int16_t temp_raw = S16BE_bytes(&packet[8]);
  float waterC = temp_raw * 0.1f;
  if (waterC > -10 && waterC < 150) {
    ecu.motor = (ecu.motor < 0.01f)
                  ? waterC
                  : ecu.motor + SMOOTH_ALPHA_MOTOR * (waterC - ecu.motor);
  }

  // Throttle Angle (bytes 14-15) - 0.1%/bit
  uint16_t throttle_raw = U16BE_bytes(&packet[14]);
  float throttle = throttle_raw * 0.1f;
  if (throttle >= 0 && throttle <= 100) {
    ecu.throttle_dk = throttle;
    ecu.throttle_pedal = throttle;
  }

  // Errors (bytes 28-29)
  ecu.errors = U16BE_bytes(&packet[28]);

  // Find battery and lambda using proven method
  checkAlternativeValues(packet);

  packet_count++;
  
  // Debug first few packets
  if (packet_count < 3) {
    Serial.printf("[CAN] Packet %u: RPM=%u, Batt=%.1fV, Motor=%.1fC, Lambda=%.3f\n",
                 packet_count, ecu.rpm, ecu.batt, ecu.motor, ecu.lambda);
  }
}

void decodeAIM_770(const twai_message_t &m) {
  aim_message_count++;
  const uint8_t *data = m.data;
  uint8_t data_length = m.data_length_code;

  // Debug first few CAN messages
  if (aim_message_count < 5) {
    Serial.printf("[CAN] Message %u: ID=0x%03X DLC=%d Data: ", 
                 aim_message_count, m.identifier, data_length);
    for (int i = 0; i < data_length; i++) {
      Serial.printf("%02X ", data[i]);
    }
    Serial.println();
  }

  // Add this CAN message to our stream buffer
  for (int i = 0; i < data_length; i++) {
    stream_buffer[stream_pos] = data[i];
    stream_pos = (stream_pos + 1) % 100;
  }

  // Look for the header pattern with custom markers in the stream buffer
  for (int start = 0; start < 100; start++) {
    int idx30 = (start + 30) % 100;
    int idx31 = (start + 31) % 100;
    int idx32 = (start + 32) % 100;
    
    if (stream_buffer[idx30] == custom_marker_1 &&
        stream_buffer[idx31] == custom_marker_2 &&
        stream_buffer[idx32] == custom_marker_3) {
      
      // Found header with current marker configuration!
      uint8_t packet[35];
      for (int j = 0; j < 35; j++) {
        packet[j] = stream_buffer[(start + j) % 100];
      }
      
      processAIMPacket(packet);
      
      // Debug: show when markers match our button configuration
      static uint32_t last_match_print = 0;
      if (millis() - last_match_print > 5000) {
        Serial.printf("[MARKERS] Match! Custom: 0x%02X 0x%02X 0x%02X\n",
                     custom_marker_1, custom_marker_2, custom_marker_3);
        last_match_print = millis();
      }
      break;
    }
  }
  
  can_rx_count++;
}

/***************************************************
 * CAN DISPATCH
 ***************************************************/
void decodeTrijekt(const twai_message_t &m) {
  if (testMode) return;

  if (m.identifier == ID_AIM) {
    decodeAIM_770(m);
    return;
  }
  
  // Log other CAN IDs occasionally
  static uint32_t other_can_count = 0;
  if (other_can_count < 10) {
    Serial.printf("[CAN] Other ID: 0x%03X DLC=%d\n", m.identifier, m.data_length_code);
    other_can_count++;
  }
}

/***************************************************
 * HELPER FUNCTIONS
 ***************************************************/
const char* batt_color_hex(float v) {
  static int state = 2;
  switch (state) {
    case 2:
      if (v < BATT_GOOD_MIN - BATT_HYST) state = 1;
      break;
    case 1:
      if (v >= BATT_GOOD_MIN + BATT_HYST) state = 2;
      else if (v < BATT_WARN_MIN - BATT_HYST) state = 0;
      break;
    case 0:
    default:
      if (v >= BATT_WARN_MIN + BATT_HYST) state = 1;
      break;
  }
  return state == 2 ? COL_HEX_GOOD : (state == 1 ? COL_HEX_WARN : COL_HEX_BAD);
}

const char* motor_color_hex(float c) {
  static int state = 0;
  switch (state) {
    case 0:
      if (c > MOTOR_GOOD_MAX + MOTOR_HYST) state = 1;
      break;
    case 1:
      if (c <= MOTOR_GOOD_MAX - MOTOR_HYST) state = 0;
      else if (c > MOTOR_WARN_MAX + MOTOR_HYST) state = 2;
      break;
    case 2:
    default:
      if (c <= MOTOR_WARN_MAX - MOTOR_HYST) state = 1;
      break;
  }
  return state == 0 ? COL_HEX_GOOD : (state == 1 ? COL_HEX_WARN : COL_HEX_BAD);
}

/***************************************************
 * ESP-NOW
 ***************************************************/
void onEspNowSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // Silent - remove spam
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(50);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init failed");
    return;
  }
  esp_now_register_send_cb(onEspNowSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, broadcastAddr, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  
  Serial.println("[ESP-NOW] Initialized");
}

void sendEspNow() {
  DashPacket p{
    ecu.rpm, ecu.batt, ecu.motor,
    ecu.throttle_dk, ecu.throttle_pedal,
    (uint8_t)(funk_active ? 1 : 0),
    btn_left_on,
    btn_right_on,
    custom_marker_1,  // Include current marker values
    custom_marker_2,
    custom_marker_3
  };
  esp_now_send(broadcastAddr, (uint8_t*)&p, sizeof(p));
}

/***************************************************
 * STATISTICS
 ***************************************************/
void printStats() {
  uint32_t timeSinceUpdate = millis() - lastRpmUpdateMs;
  float success_rate = (packet_count + error_count > 0) ? 
                      (packet_count * 100.0 / (packet_count + error_count)) : 0.0;
  
  Serial.printf("[STATS] CAN_RX=%u, AIM_MSGS=%u, PACKETS=%u, ERRORS=%u, SUCCESS=%.1f%%\n",
               can_rx_count, aim_message_count, packet_count, error_count, success_rate);
  Serial.printf("[STATS] VALUES: RPM=%u, Batt=%.1fV, Motor=%.1fC, Lambda=%.3f, LastUpdate=%lums\n", 
               ecu.rpm, ecu.batt, ecu.motor, ecu.lambda, timeSinceUpdate);
  Serial.printf("[STATS] BUTTONS: L=0x%02X, R=0x%02X, COMBO=0x%02X\n",
               left_button_flags, right_button_flags, combo_button_flags);
}

/***************************************************
 * UI CREATION
 ***************************************************/
void create_sim_ui() {
  lv_obj_t *scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

  // TITLE
  label_title = lv_label_create(scr);
  lv_label_set_text(label_title, "RWB JANINE DASH");
  lv_obj_set_style_text_font(label_title, &ui_font_Hollow85, 0);
  lv_obj_set_style_text_color(label_title, lv_color_hex(LABEL_GREY_HEX), 0);
  lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 60);

  // SCALE
  label_scale = lv_label_create(scr);
  lv_label_set_text(label_scale, "RAUH Welt BEGRIFF");
  lv_obj_set_style_text_font(label_scale, &ui_font_t20, 0);
  lv_obj_set_style_text_color(label_scale, lv_color_hex(0xE6E5DF), 0);
  lv_obj_align(label_scale, LV_ALIGN_TOP_MID, 0, 20);

  // RPM LABEL
  label_rpm = lv_label_create(scr);
  lv_label_set_text(label_rpm, "0000");
  lv_obj_set_style_text_font(label_rpm, &ui_font_Hollow85, 0);
  lv_obj_set_style_text_color(label_rpm, lv_color_hex(0xE6E5DF), 0);
  lv_style_init(&style_rpm_label);
  lv_style_set_bg_opa(&style_rpm_label, LV_OPA_TRANSP);
  lv_style_set_border_opa(&style_rpm_label, LV_OPA_TRANSP);
  lv_style_set_outline_opa(&style_rpm_label, LV_OPA_TRANSP);
  lv_style_set_shadow_opa(&style_rpm_label, LV_OPA_TRANSP);
  lv_style_set_radius(&style_rpm_label, 0);
  lv_style_set_pad_all(&style_rpm_label, 0);
  lv_obj_set_width(label_rpm, LV_SIZE_CONTENT);
  lv_obj_set_height(label_rpm, LV_SIZE_CONTENT);
  lv_obj_add_style(label_rpm, &style_rpm_label, LV_PART_MAIN);
  lv_obj_align(label_rpm, LV_ALIGN_CENTER, 0, -50);

  // BUTTONS WITH EVENT HANDLERS
  btn_left = lv_btn_create(scr);
  lv_obj_add_flag(btn_left, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_size(btn_left, 140, 70);
  lv_obj_align(btn_left, LV_ALIGN_LEFT_MID, 10, -40);
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
  lv_obj_set_size(btn_right, 140, 70);
  lv_obj_align(btn_right, LV_ALIGN_RIGHT_MID, -10, -40);
  lv_obj_set_style_bg_color(btn_right, lv_color_hex(0x404040), LV_PART_MAIN);
  lv_obj_set_style_bg_color(btn_right, lv_color_hex(0x00FF00), LV_STATE_CHECKED);
  lv_obj_set_style_bg_opa(btn_right, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_radius(btn_right, 10, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_right, btn_right_event_handler, LV_EVENT_VALUE_CHANGED, NULL);
  btn_right_label = lv_label_create(btn_right);
  lv_label_set_text(btn_right_label, "mode 2");
  lv_obj_set_style_text_font(btn_right_label, &ui_font_Hollow38, 0);
  lv_obj_center(btn_right_label);

  // RPM BAR
  rpm_bar = lv_bar_create(scr);
  lv_bar_set_range(rpm_bar, 0, 800);
  lv_bar_set_value(rpm_bar, 0, LV_ANIM_OFF);
  static lv_style_t style_bar_anim;
  lv_style_init(&style_bar_anim);
  lv_style_set_anim_time(&style_bar_anim, 150);
  lv_obj_add_style(rpm_bar, &style_bar_anim, LV_PART_INDICATOR);
  lv_coord_t bar_width = LV_HOR_RES - 30;
  if (bar_width < 200) bar_width = 200;
  lv_obj_set_size(rpm_bar, bar_width, 15);
  lv_obj_align(rpm_bar, LV_ALIGN_CENTER, 0, 20);
  lv_obj_set_style_bg_color(rpm_bar, lv_color_hex(0x303030), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(rpm_bar, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_bg_color(rpm_bar, lv_color_hex(0x00FF40), LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(rpm_bar, LV_OPA_COVER, LV_PART_INDICATOR);

  // TICKS
  lv_obj_update_layout(scr);
  lv_area_t coords;
  lv_obj_get_coords(rpm_bar, &coords);
  int y_under = coords.y2 + 6;
  lv_coord_t bar_w = coords.x2 - coords.x1;
  for (int i = 0; i < 9; ++i) {
    label_rpm_ticks[i] = lv_label_create(scr);
    char txt[2];
    snprintf(txt, sizeof(txt), "%d", i);
    lv_label_set_text(label_rpm_ticks[i], txt);
    lv_obj_set_style_text_font(label_rpm_ticks[i], &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(label_rpm_ticks[i], lv_color_white(), 0);
    lv_obj_update_layout(label_rpm_ticks[i]);
    lv_coord_t label_w = lv_obj_get_width(label_rpm_ticks[i]);
    lv_coord_t tick_x = coords.x1 + (bar_w * i) / 8;
    lv_obj_set_pos(label_rpm_ticks[i], tick_x - (label_w / 2), y_under);
  }

  // DATA ROWS
  label_row1 = lv_label_create(scr);
  lv_label_set_text(label_row1, "Batt: --.-V   Motor: --.-C");
  lv_obj_set_style_text_font(label_row1, &ui_font_Hollow85, 0);
  lv_obj_set_style_text_color(label_row1, lv_color_hex(LABEL_GREY_HEX), 0);
  lv_label_set_recolor(label_row1, true);
  lv_obj_align(label_row1, LV_ALIGN_BOTTOM_MID, 0, -110);

  label_row2 = lv_label_create(scr);
  lv_label_set_text(label_row2, "DK: --.-%   Lambda: -.--");
  lv_obj_set_style_text_font(label_row2, &ui_font_Hollow85, 0);
  lv_obj_set_style_text_color(label_row2, lv_color_hex(LABEL_GREY_HEX), 0);
  lv_obj_align(label_row2, LV_ALIGN_BOTTOM_MID, 0, -40);

  label_row3 = lv_label_create(scr);
  lv_label_set_text(label_row3, "GP: --.-%   ESP-NOW: OFF");
  lv_obj_set_style_text_font(label_row3, &ui_font_Hollow38, 0);
  lv_obj_set_style_text_color(label_row3, lv_color_hex(LABEL_GREY_HEX), 0);
  lv_obj_align(label_row3, LV_ALIGN_BOTTOM_MID, 0, -5);
}

/***************************************************
 * UI UPDATE
 ***************************************************/
void ui_update_from_ecu() {
  char buf[128];

  // RPM smoothing and bar updates
  static float rpm_smooth = -1.0f;
  static int last_bar = -1;
  static int last_color_band = -1;
  const float RPM_UI_ALPHA = 0.20f;

  float rpm_raw = (float)ecu.rpm;
  if (rpm_raw > MAX_RPM) rpm_raw = MAX_RPM;
  if (rpm_raw < 0) rpm_raw = 0;

  if (rpm_smooth < 0.0f) {
    rpm_smooth = rpm_raw;
  } else {
    rpm_smooth += RPM_UI_ALPHA * (rpm_raw - rpm_smooth);
  }

  uint16_t rpm_display = (uint16_t)roundf(rpm_smooth);
  snprintf(buf, sizeof(buf), "%04u", rpm_display);
  lv_label_set_text(label_rpm, buf);

  int bar = rpm_display / 10;
  if (bar < 0) bar = 0;
  if (bar > 800) bar = 800;
  if (bar != last_bar) {
    lv_bar_set_value(rpm_bar, bar, LV_ANIM_ON);
    last_bar = bar;
  }

  int color_band;
  if (rpm_display < RPM_YELLOW_START) {
    color_band = 0;
  } else if (rpm_display < RPM_RED_START) {
    color_band = 1;
  } else {
    color_band = 2;
  }

  if (color_band != last_color_band) {
    lv_color_t col;
    switch (color_band) {
      case 0: col = lv_color_hex(0x00FF40); break;
      case 1: col = lv_color_hex(0xFFD000); break;
      default: col = lv_color_hex(0xFF0000); break;
    }
    lv_obj_set_style_bg_color(rpm_bar, col, LV_PART_INDICATOR);
    last_color_band = color_band;
  }

  // Button states
  btn_left_on = lv_obj_has_state(btn_left, LV_STATE_CHECKED);
  btn_right_on = lv_obj_has_state(btn_right, LV_STATE_CHECKED);

  // Check for combination button presses
  check_combination_buttons();

  // Update button labels based on active functions
  static uint8_t last_left_flags = 0;
  static uint8_t last_right_flags = 0;
  
  if (left_button_flags != last_left_flags) {
    if (left_button_flags & BTN_LEFT_SPORT_MODE) {
      lv_label_set_text(btn_left_label, "SPORT");
      lv_obj_set_style_bg_color(btn_left, lv_color_hex(0xFF0000), LV_STATE_CHECKED);
    } else if (left_button_flags & BTN_LEFT_LAUNCH_CONTROL) {
      lv_label_set_text(btn_left_label, "LAUNCH");
      lv_obj_set_style_bg_color(btn_left, lv_color_hex(0xFFFF00), LV_STATE_CHECKED);
    } else {
      lv_label_set_text(btn_left_label, "mode 1");
      lv_obj_set_style_bg_color(btn_left, lv_color_hex(0x00FF00), LV_STATE_CHECKED);
    }
    last_left_flags = left_button_flags;
  }
  
  if (right_button_flags != last_right_flags) {
    if (right_button_flags & BTN_RIGHT_ANTI_LAG) {
      lv_label_set_text(btn_right_label, "ANTI-LAG");
      lv_obj_set_style_bg_color(btn_right, lv_color_hex(0xFF5500), LV_STATE_CHECKED);
    } else if (right_button_flags & BTN_RIGHT_FLAT_SHIFT) {
      lv_label_set_text(btn_right_label, "FLAT SHIFT");
      lv_obj_set_style_bg_color(btn_right, lv_color_hex(0x00FFFF), LV_STATE_CHECKED);
    } else {
      lv_label_set_text(btn_right_label, "mode 2");
      lv_obj_set_style_bg_color(btn_right, lv_color_hex(0x00FF00), LV_STATE_CHECKED);
    }
    last_right_flags = right_button_flags;
  }

  // Data rows updates
  float batt_disp = roundf(ecu.batt * 10) / 10.0f;
  float motor_disp = roundf(ecu.motor * 10) / 10.0f;
  float dk_disp = roundf(ecu.throttle_dk * 10) / 10.0f;
  float gp_disp = roundf(ecu.throttle_pedal * 10) / 10.0f;
  float lambda_disp = ecu.lambda;

  const char* bCol = batt_color_hex(batt_disp);
  const char* mCol = motor_color_hex(motor_disp);

  // Row1
  static float last_batt = -999;
  static float last_motor = -999;
  static const char* last_bCol = nullptr;
  static const char* last_mCol = nullptr;
  bool row1_changed = fabs(batt_disp - last_batt) >= 0.1f || fabs(motor_disp - last_motor) >= 0.1f || bCol != last_bCol || mCol != last_mCol || last_bCol == nullptr;
  if (row1_changed) {
    snprintf(buf, sizeof(buf), "Batt: #%s %.1fV#   Motor: #%s %.1fC#", bCol, batt_disp, mCol, motor_disp);
    lv_label_set_text(label_row1, buf);
    last_batt = batt_disp;
    last_motor = motor_disp;
    last_bCol = bCol;
    last_mCol = mCol;
  }

  // Row2
  static float last_dk = -999.0f;
  static float last_lambda = -999.0f;
  bool row2_changed = fabs(dk_disp - last_dk) >= 0.5f || fabs(lambda_disp - last_lambda) >= 0.001f || last_dk < -900.0f;
  if (row2_changed) {
    snprintf(buf, sizeof(buf), "DK: %.1f%%   Lambda: %.2f", dk_disp, lambda_disp);
    lv_label_set_text(label_row2, buf);
    last_dk = dk_disp;
    last_lambda = lambda_disp;
  }

  // Row3
  static float last_gp = -999.0f;
  static bool last_funk = false;
  bool row3_changed = fabs(gp_disp - last_gp) >= 0.5f || funk_active != last_funk || last_gp < -900.0f;
  if (row3_changed) {
    snprintf(buf, sizeof(buf), "THROTTLE: %.1f%%   ESP-NOW: %s", gp_disp, funk_active ? "ON" : "OFF");
    lv_label_set_text(label_row3, buf);
    last_gp = gp_disp;
    last_funk = funk_active;
  }
}

/***************************************************
 * SETUP
 ***************************************************/
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== RWB JANINE DASH STARTING ===");
  Serial.println("[SYSTEM] Booting RWB Hollow85 dash (AIM protocol #1)...");

  Board* board = new Board();
  board->init();
  board->begin();

  lvgl_port_init(board->getLCD(), board->getTouch());

  lvgl_port_lock(-1);
  create_sim_ui();
  lvgl_port_unlock();

  can_ok = waveshare_twai_init();
  if (!can_ok) {
    Serial.println("[CAN] INIT FAILED!");
  } else {
    Serial.println("[CAN] Initialized successfully");
  }

  initEspNow();
  lastRpmUpdateMs = millis();
  last_stats_ms = millis();

  // Initialize ECU data
  ecu.rpm = 0;
  ecu.batt = 0.0f;
  ecu.motor = 0.0f;
  ecu.throttle_dk = 0.0f;
  ecu.throttle_pedal = 0.0f;
  ecu.lambda = 0.0f;
  ecu.lambda_valid = false;
  ecu.errors = 0;
  
  // Initialize marker bytes
  update_marker_bytes();
  
  Serial.println("[SYSTEM] Setup complete, starting main loop...");
}

/***************************************************
 * LOOP
 ***************************************************/
void loop() {
  static uint32_t lastUi = 0;
  static uint32_t lastEsp = 0;

  uint32_t now = millis();

  if (testMode) {
    // Simulation mode
    static int rpmSim = 0;
    static int dir = 1;
    int step = 3;
    rpmSim += dir * step;
    if (rpmSim >= 8000) { rpmSim = 8000; dir = -1; }
    if (rpmSim <= 0) { rpmSim = 0; dir = 1; }
    ecu.rpm = rpmSim;

    static float tb = 13.8f;
    static float tm = 80.0f;
    tb += ((random(136, 136) / 10.0f) - tb) * 0.02f;
    tm += ((float)random(105, 105) - tm) * 0.05f;
    ecu.batt = tb;
    ecu.motor = tm;
    ecu.throttle_dk = random(80, 80);
    ecu.throttle_pedal = ecu.throttle_dk;
    ecu.lambda = 1.00f;
    ecu.lambda_valid = true;
  } else {
    // CAN processing
    if (can_ok) {
      twai_message_t m;
      while (twai_receive(&m, 0) == ESP_OK) {
        decodeTrijekt(m);
      }
    }

    // RPM timeout
    if (now - lastRpmUpdateMs > RPM_TIMEOUT_MS) {
      ecu.rpm = 0;
    }
  }

  // UI update
  if (now - lastUi > UI_UPDATE_INTERVAL_MS) {
    lvgl_port_lock(-1);
    ui_update_from_ecu();
    lvgl_port_unlock();
    lastUi = now;
  }

  // ESP-NOW send
  if (now - lastEsp > ESPNOW_SEND_INTERVAL_MS) {
    sendEspNow();
    lastEsp = now;
  }

  // Statistics
  if (now - last_stats_ms > STATS_INTERVAL_MS) {
    printStats();
    last_stats_ms = now;
  }

  delay(1);
}
