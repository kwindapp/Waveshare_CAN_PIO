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
bool     funk_active     = false;   // placeholder if you later map status bits

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

/***************************************************
 * ECU DATA
 ***************************************************/
struct ECUData {
  uint16_t rpm;          // rpm
  float batt;            // V
  float motor;           // °C (bei dir Ersatz / Öl)
  float throttle_dk;     // %
  float throttle_pedal;  // %
  float lambda;          // λ
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
 * AIM full-packet reconstruction (35 bytes)
 ***************************************************/
static uint8_t aim_ring[35];
static uint8_t aim_packet[35];
static uint8_t aim_write_pos = 0;

// helpers: big-endian
static inline int16_t S16BE_bytes(const uint8_t *p) {
  return (int16_t)((uint16_t(p[0]) << 8) | uint16_t(p[1]));
}
static inline uint16_t U16BE_bytes(const uint8_t *p) {
  return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
}

/***************************************************
 * HELPERS
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
 * AIM PACKET → ECU DECODE
 ***************************************************/
void aim_decode_packet_to_ecu(const uint8_t *p) {
  // RPM (0–1)
  uint16_t rpm_raw = U16BE_bytes(&p[0]);
  if (rpm_raw > MAX_RPM) rpm_raw = MAX_RPM;
  ecu.rpm = rpm_raw;
  lastRpmUpdateMs = millis();

  // Motor temp / "Water" (8–9) 0.1°C/bit
  float waterC = S16BE_bytes(&p[8]) * 0.1f;
  ecu.motor = (ecu.motor < 0.01f)
                ? waterC
                : ecu.motor + SMOOTH_ALPHA_MOTOR * (waterC - ecu.motor);

  // Battery (12–13)
  int16_t batt_raw = S16BE_bytes(&p[12]);
  float battV = (batt_raw * 5.0f / 1024.0f); // voltage at ADC pin
  battV *= 3.0f;                             // your divider guess
  ecu.batt = (ecu.batt < 0.01f)
               ? battV
               : ecu.batt + SMOOTH_ALPHA_BATT * (battV - ecu.batt);

  // Throttle (14–15) 0.1% per bit
  float thr = U16BE_bytes(&p[14]) * 0.1f;
  ecu.throttle_dk     = thr;
  ecu.throttle_pedal  = thr;

  // Lambda (22–23) 0.001 λ per bit
  ecu.lambda = U16BE_bytes(&p[22]) * 0.001f;
}

/***************************************************
 * AIM PACKET ASSEMBLER FOR ID 0x770
 ***************************************************/
static uint32_t can_rx_count = 0;

void decodeAIM_770(const twai_message_t &m) {
  const uint8_t *data = m.data;

  for (int i = 0; i < m.data_length_code; ++i) {
    uint8_t b = data[i];

    // write byte into 35-byte ring buffer
    aim_ring[aim_write_pos] = b;
    int idx_last  = aim_write_pos;
    int idx_prev1 = (idx_last + 35 - 1) % 35;
    int idx_prev2 = (idx_last + 35 - 2) % 35;
    aim_write_pos = (aim_write_pos + 1) % 35;

    // look for marker sequence: FC, FB, FA (bytes 31,32,33)
    if (aim_ring[idx_prev2] == 0xFC &&
        aim_ring[idx_prev1] == 0xFB &&
        aim_ring[idx_last]  == 0xFA) {

      int start_idx = (idx_prev2 + 35 - 31) % 35;

      // rebuild ordered packet 0..34
      for (int j = 0; j < 35; ++j) {
        int src = (start_idx + j) % 35;
        aim_packet[j] = aim_ring[src];
      }

      if (can_rx_count < 5) {
        Serial.print("AIM PACKET RAW: ");
        for (int j = 0; j < 35; ++j) {
          if (aim_packet[j] < 0x10) Serial.print("0");
          Serial.print(aim_packet[j], HEX);
          Serial.print(" ");
        }
        Serial.println();
      }

      aim_decode_packet_to_ecu(aim_packet);
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
}

/***************************************************
 * ESP-NOW
 ***************************************************/
void onEspNowSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("ESP-NOW send: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(50);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) return;
  esp_now_register_send_cb(onEspNowSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, broadcastAddr, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

void sendEspNow() {
  DashPacket p{
    ecu.rpm, ecu.batt, ecu.motor,
    ecu.throttle_dk, ecu.throttle_pedal,
    funk_active ? 1 : 0
  };
  esp_now_send(broadcastAddr, (uint8_t*)&p, sizeof(p));
}

/***************************************************
 * UI CREATION
 ***************************************************/
void create_sim_ui() {
  lv_obj_t *scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

  lv_coord_t sw = LV_HOR_RES;   // screen width
  (void)sw;
  lv_coord_t sh = LV_VER_RES;   // screen height
  (void)sh;

  /********* TITLE (top center) *********/
  label_title = lv_label_create(scr);
  lv_label_set_text(label_title, "RWB JANINE DASH");
  lv_obj_set_style_text_font(label_title, &ui_font_Hollow85, 0);
  lv_obj_set_style_text_color(label_title, lv_color_hex(LABEL_GREY_HEX), 0);
  lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 60);

  /********* TOP SMALL TEXT (under title) *********/
  label_scale = lv_label_create(scr);
  lv_label_set_text(label_scale, "RAUH Welt BEGRIFF");
  lv_obj_set_style_text_font(label_scale, &ui_font_t20, 0);
  lv_obj_set_style_text_color(label_scale, lv_color_hex(0xE6E5DF), 0);
  lv_obj_align(label_scale, LV_ALIGN_TOP_MID, 0, 20);

  /********* BIG RPM (center-ish) *********/
  label_rpm = lv_label_create(scr);
  lv_label_set_text(label_rpm, "0000");
  lv_obj_set_style_text_font(label_rpm, &ui_font_Hollow85, 0);
  lv_obj_set_style_text_color(label_rpm, lv_color_white(), 0);

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

  /********* LEFT / RIGHT TOUCH BUTTONS (mid left/right) *********/
  btn_left = lv_btn_create(scr);
  lv_obj_add_flag(btn_left, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_size(btn_left, 140, 70);
  lv_obj_align(btn_left, LV_ALIGN_LEFT_MID, 10, -40);
  lv_obj_set_style_bg_color(btn_left, lv_color_hex(0x404040),
                            LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_color(btn_left, lv_color_hex(0x00FF00),
                            LV_PART_MAIN | LV_STATE_CHECKED);
  lv_obj_set_style_bg_opa(btn_left, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_radius(btn_left, 10, LV_PART_MAIN);

  btn_left_label = lv_label_create(btn_left);
  lv_label_set_text(btn_left_label, "mode 1");
  lv_obj_set_style_text_font(btn_left_label, &ui_font_Hollow38, 0);
  lv_obj_center(btn_left_label);

  btn_right = lv_btn_create(scr);
  lv_obj_add_flag(btn_right, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_size(btn_right, 140, 70);
  lv_obj_align(btn_right, LV_ALIGN_RIGHT_MID, -10, -40);
  lv_obj_set_style_bg_color(btn_right, lv_color_hex(0x404040),
                            LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_color(btn_right, lv_color_hex(0x00FF00),
                            LV_PART_MAIN | LV_STATE_CHECKED);
  lv_obj_set_style_bg_opa(btn_right, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_radius(btn_right, 10, LV_PART_MAIN);

  btn_right_label = lv_label_create(btn_right);
  lv_label_set_text(btn_right_label, "mode 2");
  lv_obj_set_style_text_font(btn_right_label, &ui_font_Hollow38, 0);
  lv_obj_center(btn_right_label);

  /********* RPM BAR (below RPM number) *********/
  rpm_bar = lv_bar_create(scr);
  lv_bar_set_range(rpm_bar, 0, 800);   // 0–8000 rpm -> /10
  lv_bar_set_value(rpm_bar, 0, LV_ANIM_OFF);

  // slower animation for smoother motion
  // slower animation for smoother motion (LVGL 8.x)
static lv_style_t style_bar_anim;
lv_style_init(&style_bar_anim);
lv_style_set_anim_time(&style_bar_anim, 150);   // ms
lv_obj_add_style(rpm_bar, &style_bar_anim, LV_PART_INDICATOR);


  lv_coord_t bar_width = LV_HOR_RES - 30;
  if (bar_width < 200) bar_width = 200;

  lv_obj_set_size(rpm_bar, bar_width, 15);
  lv_obj_align(rpm_bar, LV_ALIGN_CENTER, 0, 20);

  lv_obj_set_style_bg_color(rpm_bar, lv_color_hex(0x303030), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(rpm_bar, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_bg_color(rpm_bar, lv_color_hex(0x00FF40), LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(rpm_bar, LV_OPA_COVER, LV_PART_INDICATOR);

  /********* 0–8 TICK LABELS UNDER BAR *********/
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

  /********* BOTTOM ROWS *********/
  // Row 1 – Batt & Motor (Hollow85)
  label_row1 = lv_label_create(scr);
  lv_label_set_text(label_row1, "Batt: --.-V   Motor: --.-C");
  lv_obj_set_style_text_font(label_row1, &ui_font_Hollow85, 0);
  lv_obj_set_style_text_color(label_row1, lv_color_hex(LABEL_GREY_HEX), 0);
  lv_label_set_recolor(label_row1, true);
  lv_obj_align(label_row1, LV_ALIGN_BOTTOM_MID, 0, -110);

  // Row 2 – DK & Lambda (Hollow85)
  label_row2 = lv_label_create(scr);
  lv_label_set_text(label_row2, "DK: --.-%   Lambda: -.--");
  lv_obj_set_style_text_font(label_row2, &ui_font_Hollow85, 0);
  lv_obj_set_style_text_color(label_row2, lv_color_hex(LABEL_GREY_HEX), 0);
  lv_obj_align(label_row2, LV_ALIGN_BOTTOM_MID, 0, -40);

  // Row 3 – GP + Funk (small)
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

  /********* RPM label + bar (smoothed, animated) *********/
  static float rpm_smooth       = -1.0f;   // UI-only RPM
  static int   last_bar         = -1;
  static int   last_color_band  = -1;     // 0=green,1=yellow,2=red
  const  float RPM_UI_ALPHA     = 0.20f;  // 0..1, lower = smoother

  // 1) read & clamp raw RPM
  float rpm_raw = (float)ecu.rpm;
  if (rpm_raw > MAX_RPM) rpm_raw = MAX_RPM;
  if (rpm_raw < 0)       rpm_raw = 0;

  // 2) low-pass filter for display
  if (rpm_smooth < 0.0f) {
    // first run: jump to current
    rpm_smooth = rpm_raw;
  } else {
    rpm_smooth += RPM_UI_ALPHA * (rpm_raw - rpm_smooth);
  }

  // 3) integer value for display
  uint16_t rpm_display = (uint16_t)roundf(rpm_smooth);

  // fixed-width 4-digit text
  snprintf(buf, sizeof(buf), "%04u", rpm_display);
  lv_label_set_text(label_rpm, buf);

  // 4) bar value (0..8000 -> 0..800)
  int bar = rpm_display / 10;
  if (bar < 0)   bar = 0;
  if (bar > 800) bar = 800;

  if (bar != last_bar) {
    // LVGL animation between values, time set in create_sim_ui()
    lv_bar_set_value(rpm_bar, bar, LV_ANIM_ON);
    last_bar = bar;
  }

  // 5) Bar color by RPM
  int color_band;
  if (rpm_display < RPM_YELLOW_START) {
    color_band = 0; // green
  } else if (rpm_display < RPM_RED_START) {
    color_band = 1; // yellow
  } else {
    color_band = 2; // red
  }

  if (color_band != last_color_band) {
    lv_color_t col;
    switch (color_band) {
      case 0: col = lv_color_hex(0x00FF40); break; // green
      case 1: col = lv_color_hex(0xFFD000); break; // yellow
      default: col = lv_color_hex(0xFF0000); break; // red
    }

    lv_obj_set_style_bg_color(rpm_bar, col,
                              LV_PART_INDICATOR | LV_STATE_DEFAULT);
    last_color_band = color_band;
  }

  /********* Button states *********/
  btn_left_on  = lv_obj_has_state(btn_left,  LV_STATE_CHECKED);
  btn_right_on = lv_obj_has_state(btn_right, LV_STATE_CHECKED);

  /********* Common display values *********/
  float batt_disp   = roundf(ecu.batt  * 10) / 10.0f;
  float motor_disp  = roundf(ecu.motor * 10) / 10.0f;
  float dk_disp     = roundf(ecu.throttle_dk * 10) / 10.0f;
  float gp_disp     = roundf(ecu.throttle_pedal * 10) / 10.0f;
  float lambda_disp = ecu.lambda;

  const char* bCol = batt_color_hex(batt_disp);
  const char* mCol = motor_color_hex(motor_disp);

  /***** Row1: Batt + Motor (Hollow85) *****/
  static float last_batt   = -999;
  static float last_motor  = -999;
  static const char* last_bCol = nullptr;
  static const char* last_mCol = nullptr;

  bool row1_changed =
      fabs(batt_disp  - last_batt)  >= 0.1f ||
      fabs(motor_disp - last_motor) >= 0.1f ||
      bCol != last_bCol ||
      mCol != last_mCol ||
      last_bCol == nullptr;

  if (row1_changed) {
    snprintf(buf, sizeof(buf),
             "Batt: #%s %.1fV#   Motor: #%s %.1fC#",
             bCol, batt_disp, mCol, motor_disp);
    lv_label_set_text(label_row1, buf);

    last_batt  = batt_disp;
    last_motor = motor_disp;
    last_bCol  = bCol;
    last_mCol  = mCol;
  }

  /***** Row2: DK + Lambda (Hollow85) *****/
  static float last_dk      = -999.0f;
  static float last_lambda  = -999.0f;

  bool row2_changed =
      fabs(dk_disp     - last_dk)     >= 0.5f ||
      fabs(lambda_disp - last_lambda) >= 0.001f ||
      last_dk < -900.0f;

  if (row2_changed) {
    snprintf(buf, sizeof(buf),
             "DK: %.1f%%   Lambda: %.2f",
             dk_disp, lambda_disp);
    lv_label_set_text(label_row2, buf);

    last_dk      = dk_disp;
    last_lambda  = lambda_disp;
  }

  /***** Row3: GP + Funk (small) *****/
  static float last_gp  = -999.0f;
  static bool  last_funk = false;

  bool row3_changed =
      fabs(gp_disp - last_gp) >= 0.5f ||
      funk_active != last_funk ||
      last_gp < -900.0f;

  if (row3_changed) {
    snprintf(buf, sizeof(buf),
             "THROTTLE: %.1f%%   ESP-NOW: %s",
             gp_disp,
             funk_active ? "ON" : "OFF");
    lv_label_set_text(label_row3, buf);

    last_gp   = gp_disp;
    last_funk = funk_active;
  }
}

/***************************************************
 * SETUP
 ***************************************************/
void setup() {
  Serial.begin(115200);
  Serial.println("Booting RWB Hollow85 dash (AIM protocol #1)...");

  Board* board = new Board();
  board->init();
  board->begin();

  lvgl_port_init(board->getLCD(), board->getTouch());

  lvgl_port_lock(-1);
  create_sim_ui();
  lvgl_port_unlock();

  can_ok = waveshare_twai_init();
  if (!can_ok) {
    Serial.println("CAN init FAILED!");
  } else {
    Serial.println("CAN init OK");
  }

  initEspNow();
  lastRpmUpdateMs = millis();

  ecu.rpm            = 0;
  ecu.batt           = 0.0f;
  ecu.motor          = 0.0f;
  ecu.throttle_dk    = 0.0f;
  ecu.throttle_pedal = 0.0f;
  ecu.lambda         = 0.87f;
}

/***************************************************
 * LOOP
 ***************************************************/
void loop() {
  static uint32_t lastUi  = 0;
  static uint32_t lastEsp = 0;

  uint32_t now = millis();

  if (testMode) {
    // simulation
    static int rpmSim = 0;
    static int dir = 1;
    int step = 3;
    rpmSim += dir * step;
    if (rpmSim >= 8000) { rpmSim = 8000; dir = -1; }
    if (rpmSim <= 0)    { rpmSim = 0;    dir = 1;  }

    ecu.rpm = rpmSim;

    static float tb = 13.8f;
    static float tm = 80.0f;
    tb += ((random(136, 136) / 10.0f) - tb) * 0.02f;
    tm += ((float)random(105, 105) - tm) * 0.05f;

    ecu.batt  = tb;
    ecu.motor = tm;
    ecu.throttle_dk    = random(80, 80);
    ecu.throttle_pedal = ecu.throttle_dk;
    ecu.lambda         = 0.90f; // 0.85–0.95 test
  } else {
    if (can_ok) {
      twai_message_t m;
      while (twai_receive(&m, 0) == ESP_OK) {
        decodeTrijekt(m);
      }
    }

    if (now - lastRpmUpdateMs > RPM_TIMEOUT_MS) {
      ecu.rpm = 0;
    }
  }

  if (now - lastUi > UI_UPDATE_INTERVAL_MS) {
    lvgl_port_lock(-1);
    ui_update_from_ecu();
    lvgl_port_unlock();
    lastUi = now;
  }

  if (now - lastEsp > ESPNOW_SEND_INTERVAL_MS) {
    sendEspNow();
    lastEsp = now;
  }

  delay(1);
}
