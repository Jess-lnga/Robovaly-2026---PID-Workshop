#include "interface_usb.h"

#include <Arduino.h>

#include "ball_position.h"
#include "controller.h"
#include "interface_web.h"
#include "servo_cmd.h"

static const BaseType_t USB_TASK_CORE = 0;
static const uint32_t USB_TASK_DELAY_MS = 10;
static const uint32_t USB_CLIENT_TIMEOUT_MS = 3000;
static const size_t USB_LINE_MAX = 512;

static TaskHandle_t usb_task_handle = nullptr;
static bool usb_client_present = false;
static uint32_t usb_last_client_ms = 0;
static char usb_line[USB_LINE_MAX];
static size_t usb_line_len = 0;

static void usb_send(const String &json) {
  Serial.println(json);
}

static bool json_has_key(const String &json, const char *key) {
  String pattern = "\"" + String(key) + "\"";
  return json.indexOf(pattern) >= 0;
}

static String json_get_string(const String &json, const char *key, const char *fallback = "") {
  String pattern = "\"" + String(key) + "\"";
  int key_pos = json.indexOf(pattern);
  if (key_pos < 0) return String(fallback);
  int colon = json.indexOf(':', key_pos + pattern.length());
  if (colon < 0) return String(fallback);
  int first_quote = json.indexOf('"', colon + 1);
  if (first_quote < 0) return String(fallback);
  int second_quote = json.indexOf('"', first_quote + 1);
  if (second_quote < 0) return String(fallback);
  return json.substring(first_quote + 1, second_quote);
}

static float json_get_float(const String &json, const char *key, float fallback) {
  String pattern = "\"" + String(key) + "\"";
  int key_pos = json.indexOf(pattern);
  if (key_pos < 0) return fallback;
  int colon = json.indexOf(':', key_pos + pattern.length());
  if (colon < 0) return fallback;
  int end = colon + 1;
  while (end < (int)json.length() && json[end] != ',' && json[end] != '}') {
    end++;
  }
  return json.substring(colon + 1, end).toFloat();
}

static int json_get_int(const String &json, const char *key, int fallback) {
  return (int)lroundf(json_get_float(json, key, (float)fallback));
}

static bool json_get_bool(const String &json, const char *key, bool fallback) {
  String pattern = "\"" + String(key) + "\"";
  int key_pos = json.indexOf(pattern);
  if (key_pos < 0) return fallback;
  int colon = json.indexOf(':', key_pos + pattern.length());
  if (colon < 0) return fallback;
  int value_pos = colon + 1;
  while (value_pos < (int)json.length() && isspace((unsigned char)json[value_pos])) {
    value_pos++;
  }
  if (json.startsWith("true", value_pos)) return true;
  if (json.startsWith("false", value_pos)) return false;
  return json.substring(value_pos).toInt() != 0;
}

static void mark_usb_client_active(void) {
  usb_client_present = true;
  usb_last_client_ms = millis();
}

static void disconnect_usb_client(void) {
  bool was_present = usb_client_present;
  usb_client_present = false;
  usb_last_client_ms = 0;

  if (was_present && !wifi_interface_client_connected()) {
    set_controller_enabled(true);
  }
}

static void send_busy(const char *owner) {
  usb_send(String("{\"ok\":false,\"busy\":\"") + owner + "\"}");
}

static void send_state(void) {
  update_tof_distances();
  bool position_valid = compute_ball_position();

  float kp = 0.0f, ki = 0.0f, kd = 0.0f;
  get_controller_gains(&kp, &ki, &kd);

  float ab_min_alpha = 0.0f, ab_max_alpha = 0.0f;
  float ab_min_beta = 0.0f, ab_max_beta = 0.0f;
  get_alpha_beta_parameters(&ab_min_alpha, &ab_max_alpha, &ab_min_beta, &ab_max_beta);

  String json = "{";
  json += "\"ok\":true,";
  json += "\"connected\":" + String(usb_client_present ? "true" : "false") + ",";
  json += "\"wifi_client\":" + String(wifi_interface_client_connected() ? "true" : "false") + ",";
  json += "\"d1\":" + String(get_d1()) + ",";
  json += "\"d2\":" + String(get_d2()) + ",";
  json += "\"x\":" + String(position_valid ? get_ball_position() : -1) + ",";
  json += "\"v\":" + String(get_ball_speed()) + ",";
  json += "\"speed_valid\":" + String(is_ball_speed_valid() ? "true" : "false") + ",";
  json += "\"servo_angle\":" + String(get_controller_last_angle_deg()) + ",";
  json += "\"stabilization\":" + String(controller_is_enabled() ? "true" : "false") + ",";
  json += "\"controller_valid\":" + String(controller_last_update_was_valid() ? "true" : "false") + ",";
  json += "\"ball_stable\":" + String(controller_ball_is_stable() ? "true" : "false") + ",";
  json += "\"controller_idle\":" + String(controller_is_idle() ? "true" : "false") + ",";
  json += "\"ref\":" + String(get_controller_reference_mm()) + ",";
  json += "\"kp\":" + String(kp, 6) + ",";
  json += "\"ki\":" + String(ki, 6) + ",";
  json += "\"kd\":" + String(kd, 6) + ",";
  json += "\"table_length\":" + String(get_table_length_mm()) + ",";
  json += "\"servo_min\":" + String(get_servo_min_angle_deg()) + ",";
  json += "\"servo_max\":" + String(get_servo_max_angle_deg()) + ",";
  json += "\"servo_neutral\":" + String(get_servo_neutral_angle_deg()) + ",";
  json += "\"servo_theoretical_min\":" + String(get_servo_theoretical_min_angle_deg()) + ",";
  json += "\"servo_theoretical_max\":" + String(get_servo_theoretical_max_angle_deg()) + ",";
  json += "\"alpha_beta_min_alpha\":" + String(ab_min_alpha, 4) + ",";
  json += "\"alpha_beta_max_alpha\":" + String(ab_max_alpha, 4) + ",";
  json += "\"alpha_beta_min_beta\":" + String(ab_min_beta, 4) + ",";
  json += "\"alpha_beta_max_beta\":" + String(ab_max_beta, 4) + ",";
  json += "\"wifi_ssid\":\"" + String(get_wifi_ap_ssid()) + "\"";
  json += "}";
  usb_send(json);
}

static bool require_usb_client(void) {
  if (!usb_client_present) {
    usb_send("{\"ok\":false,\"error\":\"usb_not_connected\"}");
    return false;
  }
  mark_usb_client_active();
  return true;
}

static void handle_params(const String &line, bool save) {
  if (!require_usb_client()) return;

  if (json_has_key(line, "ref")) {
    set_controller_reference_mm(json_get_int(line, "ref", get_controller_reference_mm()));
  }

  float kp = 0.0f, ki = 0.0f, kd = 0.0f;
  get_controller_gains(&kp, &ki, &kd);
  if (json_has_key(line, "kp")) kp = json_get_float(line, "kp", kp);
  if (json_has_key(line, "ki")) ki = json_get_float(line, "ki", ki);
  if (json_has_key(line, "kd")) kd = json_get_float(line, "kd", kd);
  set_controller_gains(kp, ki, kd);

  bool saved = save ? save_persistent_controller_settings() : false;
  usb_send(String("{\"ok\":true,\"saved\":") + (saved ? "true" : "false") + "}");
}

static void handle_advanced(const String &line, bool save) {
  if (!require_usb_client()) return;

  float ab_min_alpha = 0.0f, ab_max_alpha = 0.0f;
  float ab_min_beta = 0.0f, ab_max_beta = 0.0f;
  get_alpha_beta_parameters(&ab_min_alpha, &ab_max_alpha, &ab_min_beta, &ab_max_beta);

  if (json_has_key(line, "ab_min_alpha")) ab_min_alpha = json_get_float(line, "ab_min_alpha", ab_min_alpha);
  if (json_has_key(line, "ab_max_alpha")) ab_max_alpha = json_get_float(line, "ab_max_alpha", ab_max_alpha);
  if (json_has_key(line, "ab_min_beta")) ab_min_beta = json_get_float(line, "ab_min_beta", ab_min_beta);
  if (json_has_key(line, "ab_max_beta")) ab_max_beta = json_get_float(line, "ab_max_beta", ab_max_beta);

  bool ok = set_alpha_beta_parameters(ab_min_alpha, ab_max_alpha, ab_min_beta, ab_max_beta);
  bool saved = ok && save ? save_persistent_advanced_settings() : false;

  get_alpha_beta_parameters(&ab_min_alpha, &ab_max_alpha, &ab_min_beta, &ab_max_beta);
  String json = "{";
  json += "\"ok\":" + String(ok ? "true" : "false") + ",";
  json += "\"saved\":" + String(saved ? "true" : "false") + ",";
  json += "\"alpha_beta_min_alpha\":" + String(ab_min_alpha, 4) + ",";
  json += "\"alpha_beta_max_alpha\":" + String(ab_max_alpha, 4) + ",";
  json += "\"alpha_beta_min_beta\":" + String(ab_min_beta, 4) + ",";
  json += "\"alpha_beta_max_beta\":" + String(ab_max_beta, 4);
  json += "}";
  usb_send(json);
}

static void handle_command(const String &line) {
  String cmd = json_get_string(line, "cmd");

  if (cmd == "connect") {
    if (wifi_interface_has_priority()) {
      send_busy("wifi");
      return;
    }
    mark_usb_client_active();
    set_controller_enabled(false);
    usb_send("{\"ok\":true,\"connected\":true}");
    return;
  }

  if (cmd == "disconnect") {
    disconnect_usb_client();
    usb_send("{\"ok\":true,\"connected\":false}");
    return;
  }

  if (cmd == "heartbeat") {
    if (usb_client_present) mark_usb_client_active();
    usb_send(String("{\"ok\":true,\"connected\":") + (usb_client_present ? "true" : "false") + "}");
    return;
  }

  if (cmd == "state") {
    if (usb_client_present) mark_usb_client_active();
    send_state();
    return;
  }

  if (cmd == "control") {
    if (!require_usb_client()) return;
    if (json_has_key(line, "stabilization")) {
      set_controller_enabled(json_get_bool(line, "stabilization", controller_is_enabled()));
    }
    if (json_has_key(line, "angle")) {
      set_controller_manual_angle(json_get_int(line, "angle", get_controller_last_angle_deg()));
    }
    send_state();
    return;
  }

  if (cmd == "params") {
    handle_params(line, false);
    return;
  }

  if (cmd == "params_save") {
    handle_params(line, true);
    return;
  }

  if (cmd == "params_reload") {
    if (!require_usb_client()) return;
    bool ok = load_persistent_controller_settings();
    usb_send(String("{\"ok\":") + (ok ? "true" : "false") + "}");
    return;
  }

  if (cmd == "advanced_set") {
    handle_advanced(line, false);
    return;
  }

  if (cmd == "advanced_save") {
    handle_advanced(line, true);
    return;
  }

  if (cmd == "wifi_set") {
    if (!require_usb_client()) return;
    String ssid = json_get_string(line, "ssid");
    bool ok = set_wifi_ap_ssid(ssid.c_str());
    usb_send(String("{\"ok\":") + (ok ? "true" : "false") + ",\"wifi_ssid\":\"" + String(get_wifi_ap_ssid()) + "\"}");
    return;
  }

  usb_send("{\"ok\":false,\"error\":\"unknown_command\"}");
}

static void read_serial_commands(void) {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;

    if (ch == '\n') {
      usb_line[usb_line_len] = '\0';
      if (usb_line_len > 0) {
        handle_command(String(usb_line));
      }
      usb_line_len = 0;
      continue;
    }

    if (usb_line_len < USB_LINE_MAX - 1) {
      usb_line[usb_line_len++] = ch;
    } else {
      usb_line_len = 0;
      usb_send("{\"ok\":false,\"error\":\"line_too_long\"}");
    }
  }
}

static void usb_task(void *pv) {
  (void)pv;

  for (;;) {
    read_serial_commands();
    if (usb_client_present && millis() - usb_last_client_ms > USB_CLIENT_TIMEOUT_MS) {
      disconnect_usb_client();
    }
    vTaskDelay(pdMS_TO_TICKS(USB_TASK_DELAY_MS));
  }
}

bool launch_interface_usb(void) {
  if (usb_task_handle != nullptr) {
    return true;
  }

  BaseType_t created = xTaskCreatePinnedToCore(
      usb_task, "TaskUSB", 4096, nullptr, 1, &usb_task_handle, USB_TASK_CORE);

  if (created != pdPASS) {
    usb_task_handle = nullptr;
    return false;
  }

  return true;
}

bool usb_interface_client_connected(void) {
  return usb_client_present;
}

bool usb_interface_has_priority(void) {
  return usb_client_present;
}
