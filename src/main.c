#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define INPUT_GPIO_PIN GPIO_NUM_35

#define OUTPUT_PIN_27 GPIO_NUM_27
#define OUTPUT_PIN_14 GPIO_NUM_14
#define PWM_PIN_26 GPIO_NUM_26

#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_DUTY_RES LEDC_TIMER_10_BIT // 10-bit resolution (0 to 1023)
#define PWM_FREQUENCY 5000             // 5 kHz
#define PWM_MAX_DUTY ((1 << 10) - 1)
#define PWM_MAX_DUTY_PERCENT                                                   \
  75.0f // 75% limit (protect 9V motor under 12V supply)

#define WIFI_SSID "ESP32-RPM-Controller"
#define WIFI_PASS "12345678" // Minimum 8 chars for WPA2

#define PULSE_BUFFER_SIZE 15
#define ZERO_SPEED_TIMEOUT_US                                                  \
  5000000LL // 5.0s timeout (supports slow speeds down to 12 RPM)
#define PULSES_PER_REV 1

// RPM measurement variables
static portMUX_TYPE isr_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile int64_t last_pulse_time_us = 0;
static volatile int64_t pulse_intervals[PULSE_BUFFER_SIZE] = {0};
static volatile uint32_t pulse_head = 0;
static volatile uint32_t pulse_count = 0;
static volatile uint32_t pulse_raw_count = 0;
static volatile bool is_new_pulse = false;

// Shared controller state protected by a mutex
static SemaphoreHandle_t s_state_mutex = NULL;
static float s_target_rpm = 150.0f;
static float s_current_rpm = 0.0f;
static float s_current_pwm = 0.0f;
static uint32_t s_pulse_count_telemetry = 0;
static float s_kp = 0.05f;
static float s_ki = 0.02f;
static float s_kd = 0.005f;
static uint8_t motor_sense = 0;

// GPIO ISR Handler: triggers on falling edge (1 rotation per pulse)
static void IRAM_ATTR gpio_isr_handler(void *arg) {
  int64_t now = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&isr_mux);
  if (last_pulse_time_us > 0) {
    int64_t diff = now - last_pulse_time_us;
    // Debounce: ignore glitches shorter than 1000us (max ~60,000 RPM)
    if (diff > 120000) {
      pulse_raw_count++;
      pulse_intervals[pulse_head] = diff;
      pulse_head = (pulse_head + 1) % PULSE_BUFFER_SIZE;
      if (pulse_count < PULSE_BUFFER_SIZE) {
        pulse_count++;
      }
      last_pulse_time_us = now;
      is_new_pulse = true;

      // printf("Registered pulse: %lli | Count: %lu | Time: %lli \n", diff,
      //        pulse_count, last_pulse_time_us);
    }
  } else {
    // First edge detected: just record timestamp to establish reference
    last_pulse_time_us = now;
  }
  portEXIT_CRITICAL_ISR(&isr_mux);
}

// Function to update PWM duty cycle (clamped between 0% and
// PWM_MAX_DUTY_PERCENT)
static void set_pwm_duty_percentage(float percentage) {
  if (percentage < 0.0f)
    percentage = 0.0f;
  if (percentage > PWM_MAX_DUTY_PERCENT)
    percentage = PWM_MAX_DUTY_PERCENT;

  uint32_t duty = (uint32_t)((percentage / 100.0f) * PWM_MAX_DUTY);
  ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty);
  ledc_update_duty(PWM_MODE, PWM_CHANNEL);
}

// Embedded Webpage HTML
static const char HTML_PAGE[] =
    "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' "
    "content='width=device-width,initial-scale=1.0'>"
    "<title>ESP32 RPM Controller</title>"
    "<style>"
    "body{font-family:Segoe "
    "UI,Roboto,sans-serif;background:#0d1117;color:#c9d1d9;margin:0;padding:"
    "20px;display:flex;justify-content:center;}"
    ".card{background:#161b22;border:1px solid "
    "#30363d;border-radius:12px;padding:24px;width:100%;max-width:440px;box-"
    "shadow:0 8px 24px rgba(0,0,0,0.5);}"
    "h1{font-size:22px;margin:0 0 16px;color:#58a6ff;text-align:center;}"
    ".grid{display:grid;grid-template-columns:1fr "
    "1fr;gap:12px;margin-bottom:16px;}"
    ".stat{background:#21262d;padding:12px;border-radius:8px;border:1px solid "
    "#30363d;}"
    ".label{font-size:12px;color:#8b949e;text-transform:uppercase;font-weight:"
    "600;margin-bottom:6px;}"
    ".val{font-size:22px;font-weight:700;color:#f0f6fc;margin-top:4px;}"
    ".unit{font-size:14px;color:#8b949e;font-weight:normal;}"
    ".form{display:flex;gap:8px;margin-bottom:16px;}"
    "input{flex:1;background:#0d1117;border:1px solid "
    "#30363d;color:#fff;padding:10px "
    "14px;border-radius:6px;font-size:16px;outline:none;}"
    "input:focus{border-color:#58a6ff;}"
    "button{background:#238636;color:#fff;border:none;padding:10px "
    "18px;border-radius:6px;cursor:pointer;font-weight:600;font-size:14px;}"
    "button:hover{background:#2ea043;}"
    ".divider{height:1px;background:#30363d;margin:16px 0;}"
    ".pid-grid{display:grid;grid-template-columns:1fr 1fr "
    "1fr;gap:8px;margin-bottom:12px;}"
    ".pid-box{display:flex;flex-direction:column;}"
    ".pid-box label{font-size:11px;color:#8b949e;text-transform:uppercase;font-"
    "weight:600;margin-bottom:4px;}"
    ".pid-box input{padding:8px 10px;font-size:14px;}"
    ".btn-pid{background:#1f6feb;width:100%;padding:10px;border-radius:6px;"
    "color:#fff;border:none;cursor:pointer;font-weight:600;font-size:14px;}"
    ".btn-pid:hover{background:#388bfd;}"
    ".toast{text-align:center;font-size:12px;min-height:18px;margin-top:8px;"
    "font-weight:500;}"
    ".badge{font-size:12px;padding:3px "
    "10px;border-radius:12px;background:#30363d;display:inline-block;margin-"
    "top:"
    "14px;}"
    "</style></head><body>"
    "<div class='card'>"
    "<h1>⚡ RPM Controller</h1>"
    "<div class='grid'>"
    "<div class='stat'><div class='label'>Target RPM</div><div class='val' "
    "id='trpm'>---</div></div>"
    "<div class='stat'><div class='label'>Current RPM</div><div class='val' "
    "id='crpm'>---</div></div>"
    "<div class='stat'><div class='label'>PWM Duty</div><div class='val' "
    "id='cpwm'>--- <span class='unit'>%</span></div></div>"
    "<div class='stat'><div class='label'>Pulse Count</div><div class='val' "
    "id='ccnt'>---</div></div>"
    "</div>"
    "<div class='label'>Set Target RPM</div>"
    "<div class='form'>"
    "<input type='number' id='rpmInput' min='0' max='10000' placeholder='e.g. "
    "150' />"
    "<button onclick='setTarget()'>Set</button>"
    "</div>"
    "<div class='divider'></div>"
    "<div class='label'>PID Gains Tuning</div>"
    "<div class='pid-grid'>"
    "<div class='pid-box'><label>Kp</label><input type='number' id='kpInput' "
    "step='0.001' min='0' /></div>"
    "<div class='pid-box'><label>Ki</label><input type='number' id='kiInput' "
    "step='0.001' min='0' /></div>"
    "<div class='pid-box'><label>Kd</label><input type='number' id='kdInput' "
    "step='0.001' min='0' /></div>"
    "</div>"
    "<button class='btn-pid' onclick='setPid()'>Update Gains</button>"
    "<button class='btn-pid' onclick='toggleSense()'>Revert Sense</button>"
    "<div id='toast' class='toast'></div>"
    "<div style='text-align:center;'><span class='badge'>Connected via SoftAP: "
    "192.168.4.1</span></div>"
    "</div>"
    "<script>"
    "let pidLoaded=false;"
    "async function pollData(){"
    "try{"
    "const res=await fetch('/api/status');"
    "if(res.ok){"
    "const d=await res.json();"
    "document.getElementById('trpm').innerText=d.target.toFixed(0);"
    "document.getElementById('crpm').innerText=d.rpm.toFixed(1);"
    "document.getElementById('cpwm').innerHTML=d.pwm.toFixed(1)+' <span "
    "class=\"unit\">%</span>';"
    "document.getElementById('ccnt').innerText=d.count;"
    "if(!pidLoaded){"
    "document.getElementById('kpInput').value=d.kp;"
    "document.getElementById('kiInput').value=d.ki;"
    "document.getElementById('kdInput').value=d.kd;"
    "pidLoaded=true;"
    "}"
    "}"
    "}catch(e){}"
    "}"
    "async function setTarget(){"
    "const val=parseFloat(document.getElementById('rpmInput').value);"
    "if(isNaN(val)||val<0)return alert('Please enter a valid RPM value');"
    "await fetch('/api/target?val='+val,{method:'POST'});"
    "document.getElementById('rpmInput').value='';"
    "pollData();"
    "}"
    "async function setPid(){"
    "const kp=parseFloat(document.getElementById('kpInput').value);"
    "const ki=parseFloat(document.getElementById('kiInput').value);"
    "const kd=parseFloat(document.getElementById('kdInput').value);"
    "if(isNaN(kp)||isNaN(ki)||isNaN(kd)||kp<0||ki<0||kd<0)return alert('Enter "
    "valid non-negative numbers for Kp, Ki, Kd');"
    "const res=await "
    "fetch(`/api/pid?kp=${kp}&ki=${ki}&kd=${kd}`,{method:'POST'});"
    "const toast=document.getElementById('toast');"
    "if(res.ok){"
    "toast.style.color='#3fb950';"
    "toast.innerText='✓ PID gains updated successfully!';"
    "}else{"
    "toast.style.color='#f85149';"
    "toast.innerText='✗ Error updating PID gains';"
    "}"
    "setTimeout(()=>{toast.innerText='';},3000);"
    "pollData();"
    "}"
    "async function toggleSense(){"
    "fetch(`/api/sense`,{method:'POST'});"
    "}"
    "setInterval(pollData,500);pollData();"
    "</script></body></html>";

// GET / - Serve web interface
static esp_err_t root_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, HTML_PAGE, HTTPD_RESP_USE_STRLEN);
}

// GET /api/status - Return telemetry in JSON
static esp_err_t status_get_handler(httpd_req_t *req) {
  char json_buf[192];
  float trpm = 0, crpm = 0, cpwm = 0;
  uint32_t count = 0;
  float kp = 0, ki = 0, kd = 0;

  if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    trpm = s_target_rpm;
    crpm = s_current_rpm;
    cpwm = s_current_pwm;
    count = s_pulse_count_telemetry;
    kp = s_kp;
    ki = s_ki;
    kd = s_kd;
    xSemaphoreGive(s_state_mutex);
  }

  snprintf(json_buf, sizeof(json_buf),
           "{\"target\":%.1f,\"rpm\":%.1f,\"pwm\":%.1f,\"count\":%lu,\"kp\":%."
           "4f,\"ki\":%.4f,\"kd\":%.4f}",
           trpm, crpm, cpwm, (unsigned long)count, kp, ki, kd);

  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json_buf, HTTPD_RESP_USE_STRLEN);
}

// POST /api/target?val=1500 - Update target RPM
static esp_err_t target_post_handler(httpd_req_t *req) {
  char query[64] = {0};
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    char val_str[16] = {0};
    if (httpd_query_key_value(query, "val", val_str, sizeof(val_str)) ==
        ESP_OK) {
      float new_target = atof(val_str);
      if (new_target >= 0.0f) {
        if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          s_target_rpm = new_target;
          xSemaphoreGive(s_state_mutex);
        }
      }
    }
  }
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

// POST /api/pid?kp=0.05&ki=0.02&kd=0.005 - Update PID gains
static esp_err_t pid_post_handler(httpd_req_t *req) {
  char query[128] = {0};
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    char kp_str[16] = {0};
    char ki_str[16] = {0};
    char kd_str[16] = {0};

    float new_kp = -1.0f, new_ki = -1.0f, new_kd = -1.0f;

    if (httpd_query_key_value(query, "kp", kp_str, sizeof(kp_str)) == ESP_OK) {
      new_kp = atof(kp_str);
    }
    if (httpd_query_key_value(query, "ki", ki_str, sizeof(ki_str)) == ESP_OK) {
      new_ki = atof(ki_str);
    }
    if (httpd_query_key_value(query, "kd", kd_str, sizeof(kd_str)) == ESP_OK) {
      new_kd = atof(kd_str);
    }

    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (new_kp >= 0.0f)
        s_kp = new_kp;
      if (new_ki >= 0.0f)
        s_ki = new_ki;
      if (new_kd >= 0.0f)
        s_kd = new_kd;
      xSemaphoreGive(s_state_mutex);
    }
  }
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t sense_post_handler(httpd_req_t *req) {
  motor_sense ^= 0x01;
  gpio_set_level(OUTPUT_PIN_27, 1 - motor_sense);
  gpio_set_level(OUTPUT_PIN_14, motor_sense);

  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

// Start HTTP server
static httpd_handle_t start_webserver(void) {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.stack_size = 8192;
  httpd_handle_t server = NULL;

  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
    };
    httpd_register_uri_handler(server, &root_uri);

    httpd_uri_t status_uri = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = status_get_handler,
    };
    httpd_register_uri_handler(server, &status_uri);

    httpd_uri_t target_uri = {
        .uri = "/api/target",
        .method = HTTP_POST,
        .handler = target_post_handler,
    };
    httpd_register_uri_handler(server, &target_uri);

    httpd_uri_t pid_uri = {
        .uri = "/api/pid",
        .method = HTTP_POST,
        .handler = pid_post_handler,
    };
    httpd_register_uri_handler(server, &pid_uri);

    httpd_uri_t sense_uri = {
        .uri = "/api/sense",
        .method = HTTP_POST,
        .handler = sense_post_handler,
    };
    httpd_register_uri_handler(server, &sense_uri);
  }
  return server;
}

// Initialize Wi-Fi in Access Point (SoftAP) mode
static void wifi_init_softap(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  wifi_config_t wifi_config = {
      .ap =
          {
              .ssid = WIFI_SSID,
              .ssid_len = strlen(WIFI_SSID),
              .password = WIFI_PASS,
              .max_connection = 4,
              .authmode = WIFI_AUTH_WPA2_PSK,
          },
  };

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  printf("\n============================================\n");
  printf("Wi-Fi SoftAP started!\n");
  printf("SSID: %s\nPassword: %s\nIP: 192.168.4.1\n", WIFI_SSID, WIFI_PASS);
  printf("============================================\n\n");
}

void app_main(void) {
  // Initialize NVS (needed by Wi-Fi)
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Create state mutex
  s_state_mutex = xSemaphoreCreateMutex();

  // Configure GPIO 35 as input with falling-edge interrupt
  gpio_config_t in_conf = {
      .pin_bit_mask = (1ULL << INPUT_GPIO_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_NEGEDGE,
  };
  gpio_config(&in_conf);

  // Install GPIO ISR service and attach handler
  gpio_install_isr_service(0);
  gpio_isr_handler_add(INPUT_GPIO_PIN, gpio_isr_handler,
                       (void *)INPUT_GPIO_PIN);

  // Configure GPIO 27 and 14 as standard digital outputs
  gpio_config_t out_conf = {
      .pin_bit_mask = (1ULL << OUTPUT_PIN_27) | (1ULL << OUTPUT_PIN_14),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&out_conf);

  // Set pin states: 27 -> HIGH, 14 -> LOW
  gpio_set_level(OUTPUT_PIN_27, 1);
  gpio_set_level(OUTPUT_PIN_14, 0);

  // Configure LEDC PWM Timer
  ledc_timer_config_t ledc_timer = {.speed_mode = PWM_MODE,
                                    .timer_num = PWM_TIMER,
                                    .duty_resolution = PWM_DUTY_RES,
                                    .freq_hz = PWM_FREQUENCY,
                                    .clk_cfg = LEDC_AUTO_CLK};
  ledc_timer_config(&ledc_timer);

  // Configure LEDC PWM Channel for GPIO 26
  ledc_channel_config_t ledc_channel = {.speed_mode = PWM_MODE,
                                        .channel = PWM_CHANNEL,
                                        .timer_sel = PWM_TIMER,
                                        .intr_type = LEDC_INTR_DISABLE,
                                        .gpio_num = PWM_PIN_26,
                                        .duty = 0,
                                        .hpoint = 0};
  ledc_channel_config(&ledc_channel);

  // Start Wi-Fi AP & Web Server
  wifi_init_softap();
  start_webserver();

  // Controller configuration
  float duty_cycle_percent = 0.0f; // Initial PWM duty cycle (0.0% to 100.0%)
  set_pwm_duty_percentage(duty_cycle_percent);

  float current_rpm = 0.0f;
  float integral = 0.0f;
  float prev_rpm = 0.0f;
  int64_t last_pid_time_us = esp_timer_get_time();

  while (1) {
    int64_t last_time = 0;
    uint32_t count = 0;
    uint32_t raw_count = 0;
    int64_t intervals[PULSE_BUFFER_SIZE] = {0};
    bool has_new_pulse = false;

    portENTER_CRITICAL(&isr_mux);
    last_time = last_pulse_time_us;
    count = pulse_count;
    raw_count = pulse_raw_count;
    for (int i = 0; i < PULSE_BUFFER_SIZE; i++) {
      intervals[i] = pulse_intervals[i];
    }
    has_new_pulse = is_new_pulse;
    is_new_pulse = false;
    portEXIT_CRITICAL(&isr_mux);

    int64_t now = esp_timer_get_time();

    // If no pulse received within zero-speed timeout (5.0s for slow RPMs),
    // consider motor stopped
    if (last_time == 0 || (now - last_time) >= ZERO_SPEED_TIMEOUT_US) {
      current_rpm = 0.0f;
      portENTER_CRITICAL(&isr_mux);
      last_pulse_time_us = 0;
      pulse_count = 0;
      pulse_head = 0;
      for (int i = 0; i < PULSE_BUFFER_SIZE; i++) {
        pulse_intervals[i] = 0;
      }
      portEXIT_CRITICAL(&isr_mux);
    } else if (has_new_pulse && count >= PULSE_BUFFER_SIZE) {
      // Calculate mean RPM after 15 pulses have been measured
      int64_t sum_intervals = 0;
      for (int i = 0; i < PULSE_BUFFER_SIZE; i++) {
        sum_intervals += intervals[i];
      }

      if (sum_intervals > 0) {
        float mean_interval_us =
            (float)sum_intervals / (float)PULSE_BUFFER_SIZE;
        float calculated_rpm =
            (60.0f * 1000000.0f) / (mean_interval_us * PULSES_PER_REV);

        // Reject impossible initial spikes (e.g. > 10,000 RPM)
        if (calculated_rpm <= 10000.0f) {
          current_rpm = calculated_rpm;
        }
      }
    }

    float rpm = current_rpm;

    // Calculate actual dt for PID controller
    int64_t now_pid_time = esp_timer_get_time();
    float dt = (float)(now_pid_time - last_pid_time_us) / 1000000.0f;
    last_pid_time_us = now_pid_time;
    if (dt <= 0.0f || dt > 1.0f) {
      dt = 0.1f; // fallback to 100ms nominal update rate
    }

    // Fetch current target RPM and PID gains safely
    float target = 0.0f;
    float kp = 0.0f, ki = 0.0f, kd = 0.0f;
    if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      target = s_target_rpm;
      kp = s_kp;
      ki = s_ki;
      kd = s_kd;
      // Update telemetry shared with web server
      s_current_rpm = rpm;
      s_current_pwm = duty_cycle_percent;
      s_pulse_count_telemetry = raw_count;
      xSemaphoreGive(s_state_mutex);
    }

    // PID controller calculation
    float error = target - rpm;

    if (target <= 0.0f) {
      integral = 0.0f;
      duty_cycle_percent = 0.0f;
    } else {
      // Proportional term
      float p_term = kp * error;

      // Integral term with anti-windup clamping
      integral += error * dt;
      if (ki > 0.0f) {
        float max_integral = PWM_MAX_DUTY_PERCENT / ki;
        if (integral > max_integral) {
          integral = max_integral;
        } else if (integral < 0.0f) {
          integral = 0.0f; // Only forward motor drive
        }
      } else {
        integral = 0.0f;
      }
      float i_term = ki * integral;

      // Derivative term on measurement (avoids derivative kick on target
      // change)
      float d_term = 0.0f;
      if (dt > 0.0f) {
        d_term = -kd * (rpm - prev_rpm) / dt;
      }

      duty_cycle_percent = p_term + i_term + d_term;

      // Clamp duty cycle between 0% and PWM_MAX_DUTY_PERCENT (75% to protect 9V
      // motor under 12V supply)
      if (duty_cycle_percent > PWM_MAX_DUTY_PERCENT) {
        duty_cycle_percent = PWM_MAX_DUTY_PERCENT;
      }
      if (duty_cycle_percent < 0.0f) {
        duty_cycle_percent = 0.0f;
      }
    }

    prev_rpm = rpm;

    // Apply updated duty cycle to PWM output
    set_pwm_duty_percentage(duty_cycle_percent);

    printf("Target: %.0f | RPM: %.1f | Error: %.1f | PWM: %.1f%% | Kp: %.3f "
           "Ki: %.3f Kd: %.3f | Count: %lu\n",
           target, rpm, error, duty_cycle_percent, kp, ki, kd,
           (unsigned long)raw_count);

    vTaskDelay(pdMS_TO_TICKS(100)); // 100ms update rate
  }
}