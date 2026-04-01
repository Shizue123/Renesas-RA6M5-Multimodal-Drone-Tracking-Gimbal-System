#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* WiFi credentials */
static const char *WIFI_SSID     = "Lzy";
static const char *WIFI_PASSWORD = "12345678";

/* Cloud endpoint */
static const char *CLOUD_URL = "http://10.220.189.61:8000/api/device/upload";

/* WebCloud expected defaults (must exist in backend camera registry) */
static const char *DEFAULT_DEVICE_ID = "DRONE-001";
static const char *DEFAULT_LOCATION  = "学校操场";
static const char *DEFAULT_CAMERA_ID = "CAM_NORTH_01";

/* OpenMV 输出分辨率中心点，用于 err 坐标转绝对像素 */
#define OPENMV_CENTER_X 120.0f
#define OPENMV_CENTER_Y 120.0f
#define OPENMV_MAX_X    239.0f
#define OPENMV_MAX_Y    239.0f

#define PAN_RAW_MAX     4095.0f
#define TILT_RAW_MIN    1080.0f
#define TILT_RAW_MAX    2457.0f

/* Serial2 pins and baud (RA6M5 SCI3) */
#define SERIAL2_RXD   16
#define SERIAL2_TXD   17
#define SERIAL2_BAUD  115200

/* Buffer and scheduler limits */
#define FRAME_BUF_SIZE            256
#define JSON_LINE_BUF_SIZE        512
#define MIN_UPLOAD_INTERVAL_MS    80
#define WIFI_RECONNECT_INTERVAL   5000
#define MAX_CONSECUTIVE_ERRORS    20
#define ERROR_BACKOFF_MS          5000
#define HTTP_TIMEOUT_MS           800
#define UPLOAD_QUEUE_LENGTH       1

static char frame_buf[FRAME_BUF_SIZE];
static int frame_idx = 0;
static bool in_frame = false;

static char json_line_buf[JSON_LINE_BUF_SIZE];
static int json_line_idx = 0;
static bool in_json_line = false;

static unsigned long last_upload_ms = 0;
static unsigned long last_reconnect_ms = 0;
static unsigned int consecutive_errs = 0;
static bool wifi_connect_started = false;

static const char *wifi_status_to_text(wl_status_t status)
{
    switch (status)
    {
        case WL_NO_SHIELD:
            return "WL_NO_SHIELD";
        case WL_IDLE_STATUS:
            return "WL_IDLE_STATUS";
        case WL_NO_SSID_AVAIL:
            return "WL_NO_SSID_AVAIL";
        case WL_SCAN_COMPLETED:
            return "WL_SCAN_COMPLETED";
        case WL_CONNECTED:
            return "WL_CONNECTED";
        case WL_CONNECT_FAILED:
            return "WL_CONNECT_FAILED";
        case WL_CONNECTION_LOST:
            return "WL_CONNECTION_LOST";
        case WL_DISCONNECTED:
            return "WL_DISCONNECTED";
        default:
            return "WL_UNKNOWN";
    }
}

static void wifi_debug_scan(void)
{
    int network_count = WiFi.scanNetworks();
    Serial.printf("[GW] WiFi scan done: %d network(s) found\n", network_count);

    for (int index = 0; index < network_count; ++index)
    {
        Serial.printf(
            "[GW]   SSID=%s RSSI=%d CH=%d ENC=%d\n",
            WiFi.SSID(index).c_str(),
            WiFi.RSSI(index),
            WiFi.channel(index),
            (int)WiFi.encryptionType(index)
        );
    }

    WiFi.scanDelete();
}

typedef struct
{
    char json[JSON_LINE_BUF_SIZE];
} upload_packet_t;

static QueueHandle_t g_upload_queue = NULL;

static const char *normalize_status(const char *raw)
{
    if (raw == NULL)
    {
        return "VISION_STATUS_LOST";
    }

    if ((strcmp(raw, "VISION_STATUS_LOCKED") == 0) || (strcmp(raw, "VISION_LOCK") == 0))
    {
        return "VISION_STATUS_LOCKED";
    }

    if ((strcmp(raw, "VISION_STATUS_PREDICTED") == 0) || (strcmp(raw, "COASTING") == 0))
    {
        return "VISION_STATUS_PREDICTED";
    }

    return "VISION_STATUS_LOST";
}

static float clampf_local(float value, float min_v, float max_v)
{
    if (value < min_v)
    {
        return min_v;
    }

    if (value > max_v)
    {
        return max_v;
    }

    return value;
}

static float raw_to_pan_deg(float raw_value)
{
    return clampf_local(raw_value, 0.0f, PAN_RAW_MAX) * 270.0f / PAN_RAW_MAX;
}

static float raw_to_tilt_deg(float raw_value)
{
    float clamped = clampf_local(raw_value, TILT_RAW_MIN, TILT_RAW_MAX);
    return (clamped - TILT_RAW_MIN) * 180.0f / (TILT_RAW_MAX - TILT_RAW_MIN);
}

static float compute_radar_dist_mm(float radar_x_mm, float radar_y_mm, float reported_dist_mm)
{
    if (reported_dist_mm > 0.0f)
    {
        return reported_dist_mm;
    }

    if ((radar_x_mm != 0.0f) || (radar_y_mm != 0.0f))
    {
        return sqrtf((radar_x_mm * radar_x_mm) + (radar_y_mm * radar_y_mm));
    }

    return 0.0f;
}

static const char *status_from_ra_state(const char *state)
{
    if (state == NULL)
    {
        return "VISION_STATUS_LOST";
    }

    if (strcmp(state, "VISION_LOCK") == 0)
    {
        return "VISION_STATUS_LOCKED";
    }

    if (strcmp(state, "COASTING") == 0)
    {
        return "VISION_STATUS_PREDICTED";
    }

    if (strcmp(state, "RADAR_SLEW") == 0)
    {
        return "VISION_STATUS_PREDICTED";
    }

    return "VISION_STATUS_LOST";
}

static void wifi_connect_nonblocking(void)
{
    if (WiFi.status() == WL_CONNECTED)
    {
        return;
    }

    Serial.printf("[GW] WiFi connect start: %s\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true, true);
    delay(100);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    wifi_connect_started = true;
    last_reconnect_ms = millis();
}

static void wifi_check_reconnect(void)
{
    static bool connected_announced = false;

    if (WiFi.status() == WL_CONNECTED)
    {
        if (!connected_announced)
        {
            Serial.printf("[GW] WiFi connected, IP=%s\n", WiFi.localIP().toString().c_str());
            connected_announced = true;
        }
        return;
    }

    connected_announced = false;

    if (!wifi_connect_started)
    {
        wifi_connect_nonblocking();
        return;
    }

    unsigned long now = millis();
    if ((now - last_reconnect_ms) < WIFI_RECONNECT_INTERVAL)
    {
        return;
    }

    last_reconnect_ms = now;
    Serial.printf(
        "[GW] WiFi reconnect attempt, status=%s(%d), target=%s\n",
        wifi_status_to_text(WiFi.status()),
        (int)WiFi.status(),
        WIFI_SSID
    );
    wifi_debug_scan();
    WiFi.disconnect(true, true);
    delay(100);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

static bool parse_legacy_frame(const char *raw, JsonDocument &doc)
{
    char buf[FRAME_BUF_SIZE];
    strncpy(buf, raw, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *tokens[8];
    int count = 0;
    char *ctx = NULL;
    char *tok = strtok_r(buf, ",", &ctx);

    while ((tok != NULL) && (count < 8))
    {
        tokens[count++] = tok;
        tok = strtok_r(NULL, ",", &ctx);
    }

    if (count < 8)
    {
        Serial.printf("[GW] Legacy frame parse error: fields=%d\n", count);
        return false;
    }

    doc["device_id"]  = tokens[0];
    doc["location"]   = DEFAULT_LOCATION;
    doc["camera_id"]  = tokens[1];
    doc["radar_dist"] = atof(tokens[2]);
    doc["gimbal_pan"] = atof(tokens[3]);
    doc["gimbal_tilt"] = atof(tokens[4]);
    doc["target_x"]   = atof(tokens[5]);
    doc["target_y"]   = atof(tokens[6]);
    doc["status"]     = normalize_status(tokens[7]);

    if ((doc["camera_id"].as<const char *>() == NULL) || (strlen(doc["camera_id"].as<const char *>()) == 0))
    {
        doc["camera_id"] = DEFAULT_CAMERA_ID;
    }

    return true;
}

static bool normalize_json_line(const char *raw_json, JsonDocument &doc)
{
    StaticJsonDocument<768> input;
    DeserializationError err = deserializeJson(input, raw_json);
    if (err)
    {
        Serial.printf("[GW] JSON parse error: %s\n", err.c_str());
        return false;
    }

    if (input["device_id"].is<const char *>())
    {
        float err_x = input["target_err_x"] | 0.0f;
        float err_y = input["target_err_y"] | 0.0f;
        float abs_x = input["target_x"] | (OPENMV_CENTER_X + err_x);
        float abs_y = input["target_y"] | (OPENMV_CENTER_Y + err_y);
        float pan_field = input["gimbal_pan"] | 0.0f;
        float tilt_field = input["gimbal_tilt"] | 0.0f;
        bool has_pan_deg = !input["gimbal_pan_deg"].isNull();
        bool has_tilt_deg = !input["gimbal_tilt_deg"].isNull();
        bool has_pan_raw = !input["gimbal_pan_raw"].isNull();
        bool has_tilt_raw = !input["gimbal_tilt_raw"].isNull();
        float pan_raw = has_pan_raw ? (input["gimbal_pan_raw"] | 0.0f) : ((pan_field > 270.0f) ? pan_field : 0.0f);
        float tilt_raw = has_tilt_raw ? (input["gimbal_tilt_raw"] | 0.0f) : ((tilt_field > 180.0f) ? tilt_field : 0.0f);
        float pan_deg = has_pan_deg ? (input["gimbal_pan_deg"] | 0.0f) : (((pan_field > 0.0f) && (pan_field <= 270.0f)) ? pan_field : raw_to_pan_deg(pan_raw));
        float tilt_deg = has_tilt_deg ? (input["gimbal_tilt_deg"] | 0.0f) : (((tilt_field > 0.0f) && (tilt_field <= 180.0f)) ? tilt_field : raw_to_tilt_deg(tilt_raw));
        float radar_x_mm = input["radar_x_mm"] | 0.0f;
        float radar_y_mm = input["radar_y_mm"] | 0.0f;
        float radar_dist_mm = compute_radar_dist_mm(radar_x_mm, radar_y_mm, input["radar_dist"] | 0.0f);
        const char *location = input["location"] | DEFAULT_LOCATION;
        const char *camera_id = input["camera_id"] | DEFAULT_CAMERA_ID;

        if ((location == NULL) || (strlen(location) == 0))
        {
            location = DEFAULT_LOCATION;
        }

        if ((camera_id == NULL) || (strlen(camera_id) == 0))
        {
            camera_id = DEFAULT_CAMERA_ID;
        }

        doc["device_id"] = input["device_id"] | DEFAULT_DEVICE_ID;
        doc["location"] = location;
        doc["camera_id"] = camera_id;
        doc["radar_dist"] = radar_dist_mm;
        doc["radar_x_mm"] = radar_x_mm;
        doc["radar_y_mm"] = radar_y_mm;
        doc["gimbal_pan"] = pan_deg;
        doc["gimbal_tilt"] = tilt_deg;
        doc["gimbal_pan_raw"] = pan_raw;
        doc["gimbal_tilt_raw"] = tilt_raw;
        doc["target_x"] = clampf_local(abs_x, 0.0f, OPENMV_MAX_X);
        doc["target_y"] = clampf_local(abs_y, 0.0f, OPENMV_MAX_Y);
        doc["status"] = normalize_status(input["status"] | "LOST");
        doc["uart3_drop"] = input["uart3_drop"] | 0;
        doc["uart3_sent"] = input["uart3_sent"] | 0;
        doc["uart3_q"] = input["uart3_q"] | 0;
        doc["uart3_q_peak"] = input["uart3_q_peak"] | 0;
        doc["uart3_reject"] = input["uart3_reject"] | 0;
        doc["uart3_skip"] = input["uart3_skip"] | 0;
        doc["servo_drop"] = input["servo_drop"] | 0;
        doc["uart3_ovf_reset"] = input["uart3_ovf_reset"] | 0;
        doc["servo_uart_open_err"] = input["servo_uart_open_err"] | 0;
        doc["servo_uart_write_err"] = input["servo_uart_write_err"] | 0;
        doc["servo_uart_tx"] = input["servo_uart_tx"] | 0;
        doc["servo_uart_to"] = input["servo_uart_to"] | 0;
        
        doc["feedforward_pan"] = input["feedforward_pan"] | 0.0f;
        doc["filtered_error"] = input["filtered_error"] | 0.0f;
        doc["radar_vx"] = input["radar_vx"] | 0.0f;

        doc["tracking_mode"] = input["tracking_mode"] | "STATE_SEARCHING";
        doc["hover_active"] = input["hover_active"] | 0;

        doc["timestamp"] = input["timestamp"] | millis();
        return true;
    }

    float pan_field = input["gimbal_pan"] | 0.0f;
    float tilt_field = input["gimbal_tilt"] | 0.0f;
    bool has_pan_deg = !input["gimbal_pan_deg"].isNull();
    bool has_tilt_deg = !input["gimbal_tilt_deg"].isNull();
    bool has_pan_raw = !input["gimbal_pan_raw"].isNull();
    bool has_tilt_raw = !input["gimbal_tilt_raw"].isNull();
    float pan_raw = has_pan_raw ? (input["gimbal_pan_raw"] | 0.0f) : (input["target_angle"] | ((pan_field > 270.0f) ? pan_field : 0.0f));
    float tilt_raw = has_tilt_raw ? (input["gimbal_tilt_raw"] | 0.0f) : ((tilt_field > 180.0f) ? tilt_field : 0.0f);
    float pan_deg = has_pan_deg ? (input["gimbal_pan_deg"] | 0.0f) : (((pan_field > 0.0f) && (pan_field <= 270.0f)) ? pan_field : raw_to_pan_deg(pan_raw));
    float tilt_deg = has_tilt_deg ? (input["gimbal_tilt_deg"] | 0.0f) : (((tilt_field > 0.0f) && (tilt_field <= 180.0f) && !has_tilt_raw) ? tilt_field : raw_to_tilt_deg(tilt_raw));
    float radar_x_mm = input["radar_x_mm"] | 0.0f;
    float radar_y_mm = input["radar_y_mm"] | 0.0f;
    float radar_dist_mm = compute_radar_dist_mm(radar_x_mm, radar_y_mm, input["radar_dist"] | 0.0f);

    doc["device_id"] = DEFAULT_DEVICE_ID;
    doc["location"] = DEFAULT_LOCATION;
    doc["camera_id"] = DEFAULT_CAMERA_ID;
    doc["radar_dist"] = radar_dist_mm;
    doc["radar_x_mm"] = radar_x_mm;
    doc["radar_y_mm"] = radar_y_mm;
    doc["gimbal_pan"] = pan_deg;
    doc["gimbal_tilt"] = tilt_deg;
    doc["gimbal_pan_raw"] = pan_raw;
    doc["gimbal_tilt_raw"] = tilt_raw;
    doc["target_x"] = clampf_local(input["target_x"] | OPENMV_CENTER_X, 0.0f, OPENMV_MAX_X);
    doc["target_y"] = clampf_local(input["target_y"] | OPENMV_CENTER_Y, 0.0f, OPENMV_MAX_Y);
    
    doc["feedforward_pan"] = input["feedforward_pan"] | 0.0f;
    doc["filtered_error"] = input["filtered_error"] | 0.0f;
    doc["radar_vx"] = input["radar_vx"] | 0.0f;

    doc["tracking_mode"] = input["tracking_mode"] | "STATE_SEARCHING";
    doc["hover_active"] = input["hover_active"] | 0;

    doc["status"] = status_from_ra_state(input["state"] | "SEARCH");
    doc["vision_active"] = input["vision_active"] | 0;
    doc["timestamp"] = input["timestamp"] | millis();
    return true;
}

static bool read_serial_packet(JsonDocument &doc)
{
    while (Serial2.available())
    {
        char ch = (char)Serial2.read();

        // 简化：如果遇到$，也可以直接忽略（丢弃旧格式）
        if (ch == '$')
        {
            in_frame = true;
            frame_idx = 0;
            continue;
        }

        if (!in_frame && ch == '{')
        {
            in_json_line = true;
            json_line_idx = 0;
            json_line_buf[json_line_idx++] = ch;
            continue;
        }

        if (in_frame)
        {
            if (ch == '#')
            {
                frame_buf[frame_idx] = '\0';
                in_frame = false;
                return parse_legacy_frame(frame_buf, doc);
            }

            if (ch == '\r' || ch == '\n')
            {
                continue;
            }

            if (frame_idx < (FRAME_BUF_SIZE - 1))
            {
                frame_buf[frame_idx++] = ch;
            }
            else
            {
                Serial.println("[GW] Legacy frame overflow, discard");
                in_frame = false;
                frame_idx = 0;
            }
            continue;
        }

        if (in_json_line)
        {
            if (ch == '\n' || ch == '\r')
            {
                if (json_line_idx > 0)
                {
                    json_line_buf[json_line_idx] = '\0';
                    in_json_line = false;
                    return normalize_json_line(json_line_buf, doc);
                }

                in_json_line = false;
                json_line_idx = 0;
                continue;
            }

            if (json_line_idx < (JSON_LINE_BUF_SIZE - 1))
            {
                json_line_buf[json_line_idx++] = ch;
            }
            else
            {
                Serial.println("[GW] JSON line overflow, discard");
                in_json_line = false;
                json_line_idx = 0;
            }
        }
    }

    return false;
}

static bool upload_json(const JsonDocument &doc)
{
    if (WiFi.status() != WL_CONNECTED)
    {
        return false;
    }

    HTTPClient http;
    http.begin(CLOUD_URL);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(HTTP_TIMEOUT_MS);

    String payload;
    serializeJson(doc, payload);

    int code = http.POST(payload);
    http.end();

    if (code >= 200 && code < 300)
    {
        consecutive_errs = 0;
        return true;
    }

    consecutive_errs++;
    Serial.printf("[GW] Upload failed: HTTP %d (errs=%u)\n", code, consecutive_errs);
    return false;
}

static void queue_upload_document(const JsonDocument &doc)
{
    if (g_upload_queue == NULL)
    {
        return;
    }

    upload_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    serializeJson(doc, pkt.json, sizeof(pkt.json));
    pkt.json[sizeof(pkt.json) - 1] = '\0';

    (void)xQueueOverwrite(g_upload_queue, &pkt);
}

static void upload_task(void *arg)
{
    (void)arg;
    upload_packet_t pkt;
    unsigned long backoff_until = 0;

    for (;;)
    {
        if (xQueueReceive(g_upload_queue, &pkt, pdMS_TO_TICKS(30)) == pdTRUE)
        {
            unsigned long now = millis();
            if (now < backoff_until)
            {
                continue;
            }

            if (WiFi.status() != WL_CONNECTED)
            {
                continue;
            }

            StaticJsonDocument<768> doc;
            if (deserializeJson(doc, pkt.json) == DeserializationError::Ok)
            {
                if (upload_json(doc))
                {
                    last_upload_ms = millis();
                }
                else if (consecutive_errs >= MAX_CONSECUTIVE_ERRORS)
                {
                    backoff_until = millis() + ERROR_BACKOFF_MS;
                    consecutive_errs = 0;
                }
            }
        }
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("\n=== ESP32 Async Gateway v2.0 ===");

    Serial2.begin(SERIAL2_BAUD, SERIAL_8N1, SERIAL2_RXD, SERIAL2_TXD);
    Serial.printf("[GW] Serial2 opened: %d baud (RXD=%d TXD=%d)\n", SERIAL2_BAUD, SERIAL2_RXD, SERIAL2_TXD);

    wifi_connect_nonblocking();

    g_upload_queue = xQueueCreate(UPLOAD_QUEUE_LENGTH, sizeof(upload_packet_t));
    if (g_upload_queue != NULL)
    {
        xTaskCreatePinnedToCore(upload_task, "upload_task", 6144, NULL, 1, NULL, 1);
    }

    Serial.println("[GW] Gateway ready");
}

void loop()
{
    wifi_check_reconnect();

    StaticJsonDocument<768> doc;
    if (read_serial_packet(doc))
    {
        unsigned long now = millis();
        if ((now - last_upload_ms) >= MIN_UPLOAD_INTERVAL_MS)
        {
            queue_upload_document(doc);
        }
    }

    delay(1);
}
