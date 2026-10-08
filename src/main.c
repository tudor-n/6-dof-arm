// ESP32 "dummy executor": receives joint angles over USB serial, drives PCA9685.
// Protocol (115200 8N1, newline-terminated):
//   J a0 a1 a2 a3 a4 a5   -> set target servo angles in degrees (0..180)
//   GET                   -> replies "S a0 ... a5" with current angles
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "ARM";

#define I2C_SDA        21
#define I2C_SCL        22
#define I2C_NUM        0
#define PCA_ADDR       0x40
#define PCA_MODE1      0x00
#define PCA_PRESCALE   0xFE
#define LED0_ON_L      0x06
#define SERVOMIN       150   // 0 deg   (same as your code)
#define SERVOMAX       500   // 180 deg
#define N              6
#define MAX_DPS        180.0f   // firmware-side speed safety cap
#define LOOP_US        20000    // 50 Hz

// Power-on pose (matches your go_to_start_pose)
static float cur[N] = {90.0, 16.0, 10.0, 72.0, 90.0, 100.0};
static float tgt[N] = {90.0, 16.0, 10.0, 72.0, 90.0, 100.0};
static uint8_t relaxed = 0;   // bit i = channel i has PWM cut (limp, ~0 current)
static int force_push = 0;
static int engaged = 0;       // 0 after boot: all servos limp until the server sends an R command (Live ON)
static float lim_lo[N] = {0, 0, 0, 0, 0, 0}, lim_hi[N] = {180, 180, 180, 180, 180, 180};  // set by server: L i lo hi

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static esp_err_t pca_write(const uint8_t *buf, size_t len) {
    return i2c_master_write_to_device(I2C_NUM, PCA_ADDR, buf, len, pdMS_TO_TICKS(100));
}
static void pca_reg(uint8_t reg, uint8_t val) { uint8_t b[2] = {reg, val}; pca_write(b, 2); }

static void i2c_init(void) {
    i2c_config_t c = {
        .mode = I2C_MODE_MASTER, .sda_io_num = I2C_SDA, .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE, .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM, &c));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, c.mode, 0, 0, 0));
}

static void pca_init(float freq) {
    float p = 25000000.0f / 4096.0f / freq - 1.0f;
    uint8_t prescale = (uint8_t)floorf(p + 0.5f);
    pca_reg(PCA_MODE1, 0x10);          // sleep
    pca_reg(PCA_PRESCALE, prescale);
    pca_reg(PCA_MODE1, 0x00);          // wake
    vTaskDelay(pdMS_TO_TICKS(5));
    pca_reg(PCA_MODE1, 0x20);          // auto-increment on
}

// All 6 channels in ONE I2C transaction
static void pca_push(void) {
    uint8_t buf[1 + 4 * N];
    buf[0] = LED0_ON_L;
    for (int i = 0; i < N; i++) {
        uint16_t pulse = (uint16_t)(SERVOMIN + (cur[i] / 180.0f) * (SERVOMAX - SERVOMIN));
        buf[1 + 4 * i] = 0;
        buf[2 + 4 * i] = 0;
        buf[3 + 4 * i] = pulse & 0xFF;
        buf[4 + 4 * i] = (pulse >> 8) & 0xFF;
        if (relaxed & (1 << i)) { buf[3 + 4 * i] = 0; buf[4 + 4 * i] = 0x10; }   // OFF_H bit4 = full OFF
    }
    if (pca_write(buf, sizeof(buf)) != ESP_OK) ESP_LOGW(TAG, "i2c write failed");
}

static void reply(const char *s) { uart_write_bytes(UART_NUM_0, s, strlen(s)); }

static void handle_line(char *s) {
    if (s[0] == 'J' && s[1] == ' ') {
        float v[N];
        if (sscanf(s + 2, "%f %f %f %f %f %f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) == 6)
            for (int i = 0; i < N; i++) {
                float nv = clampf(v[i], lim_lo[i], lim_hi[i]);
                if (fabsf(nv - tgt[i]) > 0.05f) { if (engaged) relaxed &= ~(1 << i); }   // moving a channel wakes it
                tgt[i] = nv;
            }
    } else if (s[0] == 'L' && s[1] == ' ') {          // L <ch> <lo> <hi>: hard per-channel limits
        int i; float lo, hi;
        if (sscanf(s + 2, "%d %f %f", &i, &lo, &hi) == 3 && i >= 0 && i < N && lo <= hi) {
            lim_lo[i] = clampf(lo, 0, 180); lim_hi[i] = clampf(hi, 0, 180);
            tgt[i] = clampf(tgt[i], lim_lo[i], lim_hi[i]);
        }
    } else if (s[0] == 'R' && s[1] == ' ') {          // R <bitmask>: cut PWM on those channels
        relaxed = (uint8_t)(atoi(s + 2) & 0x3F); force_push = 1; engaged = 1;
    } else if (strncmp(s, "GET", 3) == 0) {
        char out[96];
        snprintf(out, sizeof(out), "S %.1f %.1f %.1f %.1f %.1f %.1f E%d\n",
                 cur[0], cur[1], cur[2], cur[3], cur[4], cur[5], engaged);
        reply(out);
    }
}

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_WARN);
    uart_driver_install(UART_NUM_0, 512, 0, 0, NULL, 0);
    i2c_init();
    pca_init(50.0f);
    relaxed = 0x3F;
    pca_push();                         // all channels OFF: servos stay limp until the UI engages them

    char line[128]; int len = 0;
    int64_t last = esp_timer_get_time();
    uint8_t rx[64];
    
    while (1) {
        // Read incoming serial commands
        int n = uart_read_bytes(UART_NUM_0, rx, sizeof(rx), pdMS_TO_TICKS(5));
        for (int i = 0; i < n; i++) {
            char c = (char)rx[i];
            if (c == '\n' || c == '\r') { if (len) { line[len] = 0; handle_line(line); len = 0; } }
            else if (len < (int)sizeof(line) - 1) line[len++] = c;
            else len = 0;
        }
        
        // Timer loop to push updates to the servos at 50Hz
        int64_t now = esp_timer_get_time();
        if (now - last >= LOOP_US) {
            float dt = (now - last) / 1e6f, maxstep = MAX_DPS * dt;
            last = now;
            int dirty = 0;
            
            for (int i = 0; i < N; i++) {
                float d = tgt[i] - cur[i];
                
                // 1. Widen deadband to 0.3° to stop MG996R from endlessly hunting/buzzing
                if (fabsf(d) > 0.3f) { 
                    float step;
                    
                    if (i == 5) {
                        // Let the server strictly dictate the gripper speed without lag
                        step = clampf(d, -maxstep, maxstep); 
                    } else {
                        // 2. Smooth the ESP32's catch-up logic to prevent serial-stutter
                        step = clampf(d * 0.15f, -maxstep, maxstep); 
                    }
                    
                    // 3. Minimum push threshold to ensure it doesn't stall out just before the target
                    if (step > 0 && step < 0.15f) step = 0.15f;
                    if (step < 0 && step > -0.15f) step = -0.15f;
                    
                    cur[i] += step; 
                    dirty = 1; 
                }
            }
            if (dirty || force_push) { pca_push(); force_push = 0; }
        }
    }
}