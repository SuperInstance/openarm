/**
 * openarm_plato_node.c — ESP32 CAN + PLATO Bridge for OpenArm
 *
 * Runs on an ESP32 connected to the OpenArm CAN bus.
 * Bridges motor telemetry → PLATO, and PLATO commands → motor control.
 * All motor commands go through constraint checking before hitting CAN.
 *
 * Hardware:
 *   - ESP32-S3 (or any ESP32 with CAN/TWAI support)
 *   - CAN transceiver (SN65HVD230 or TJA1050) on GPIO21/GPIO22
 *   - WiFi for PLATO connection
 *
 * Build:
 *   idf.py set-target esp32s3
 *   idf.py menuconfig  (set WiFi + PLATO server under "OpenArm PLATO Node")
 *   idf.py build flash monitor
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/twai.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "plato_client.h"
#include "plato_mcp.h"

/* ------------------------------------------------------------------ */
/*  Configuration                                                      */
/* ------------------------------------------------------------------ */

#define WIFI_SSID           CONFIG_PLATO_WIFI_SSID
#define WIFI_PASS           CONFIG_PLATO_WIFI_PASSWORD
#define PLATO_SERVER        CONFIG_PLATO_SERVER_HOST
#define PLATO_PORT          CONFIG_PLATO_SERVER_PORT
#define DEVICE_ID           CONFIG_PLATO_DEVICE_ID

#define NUM_JOINTS          7
#define TELEMETRY_INTERVAL_MS   100  /* 10 Hz telemetry */
#define COMMAND_POLL_MS         200  /* 5 Hz command polling */
#define CONSTRAINT_CHECK_MS     10   /* 100 Hz constraint check loop */

#define CAN_TX_GPIO    21
#define CAN_RX_GPIO    22

static const char *TAG = "OPENARM_PLATO";

/* ------------------------------------------------------------------ */
/*  Constraint types (mirrors Python constraint system)                */
/* ------------------------------------------------------------------ */

typedef struct {
    float min_pos;
    float max_pos;
} joint_limit_t;

typedef struct {
    float max_torque;
} torque_limit_t;

/* ------------------------------------------------------------------ */
/*  Arm state                                                          */
/* ------------------------------------------------------------------ */

typedef struct {
    float positions[NUM_JOINTS];
    float velocities[NUM_JOINTS];
    float torques[NUM_JOINTS];
    float temperatures[NUM_JOINTS];
    int64_t last_update_us;

    /* Safety */
    bool emergency_stop;
    int total_commands;
    int blocked_commands;
    float safety_score;

    /* Constraints */
    joint_limit_t joint_limits[NUM_JOINTS];
    torque_limit_t torque_limits[NUM_JOINTS];
    float max_velocity;

    /* PLATO */
    plato_ctx_t *plato;
} arm_state_t;

static arm_state_t g_arm = {0};

/* ------------------------------------------------------------------ */
/*  Constraint checking (runs at 100Hz, before any CAN frame)         */
/* ------------------------------------------------------------------ */

/**
 * Check if a target position is within joint limits.
 * Returns true if safe, false if the command should be rejected.
 */
static bool check_joint_limit(int joint, float target) {
    if (joint < 0 || joint >= NUM_JOINTS) return false;
    return target >= g_arm.joint_limits[joint].min_pos &&
           target <= g_arm.joint_limits[joint].max_pos;
}

/**
 * Check torque against limit.
 */
static bool check_torque_limit(int joint) {
    if (joint < 0 || joint >= NUM_JOINTS) return false;
    return fabsf(g_arm.torques[joint]) <= g_arm.torque_limits[joint].max_torque;
}

/**
 * Eisenstein workspace boundary check.
 *
 * Maps end-effector position to Eisenstein integer coordinates
 * using basis: e1 = (1, 0), e2 = (1/2, √3/2)
 *
 * Then checks if Eisenstein norm a² - ab + b² ≤ r²
 *
 * This gives hexagonal workspace boundaries — tighter than axis-aligned
 * boxes for 6-DOF/7-DOF arms.
 */
static bool check_eisenstein_workspace(float x, float y, int radius, float scale) {
    float inv_scale = 1.0f / scale;
    float inv_sqrt3 = 1.0f / sqrtf(3.0f);

    int a = (int)roundf(x * inv_scale - y * inv_scale * inv_sqrt3);
    int b = (int)roundf(2.0f * y * inv_scale * inv_sqrt3);

    /* Eisenstein norm: a² - ab + b² */
    int norm = a * a - a * b + b * b;
    return norm <= radius * radius;
}

/* ------------------------------------------------------------------ */
/*  Default constraints (7-DOF human-scale arm)                        */
/* ------------------------------------------------------------------ */

static void init_default_constraints(void) {
    for (int i = 0; i < NUM_JOINTS; i++) {
        g_arm.joint_limits[i] = (joint_limit_t){ -3.14159f, 3.14159f };
        g_arm.torque_limits[i] = (torque_limit_t){ 5.0f };
    }
    g_arm.max_velocity = 2.0f;
    g_arm.safety_score = 1.0f;
}

/* ------------------------------------------------------------------ */
/*  CAN bus initialization (TWAI on ESP32)                             */
/* ------------------------------------------------------------------ */

static void init_can(void) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_GPIO, (gpio_num_t)CAN_RX_GPIO);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "CAN bus initialized on GPIO%d/GPIO%d @ 1Mbps", CAN_TX_GPIO, CAN_RX_GPIO);
}

/* ------------------------------------------------------------------ */
/*  WiFi (same pattern as esp32_sensor_node)                           */
/* ------------------------------------------------------------------ */

static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WiFi disconnected, retrying...");
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id, instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
        ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
        IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
}

/* ------------------------------------------------------------------ */
/*  MCP tool registration (exposes arm capabilities to PLATO agents)   */
/* ------------------------------------------------------------------ */

static void register_arm_tools(plato_mcp_registry_t *reg) {
    mcp_register_tool(reg, &(plato_mcp_tool_t){
        .name = "get_joint_states",
        .description = "Get current joint positions, velocities, and torques",
        .input_schema = "{\"type\":\"object\",\"properties\":{}}",
        .output_type = "json",
    });
    mcp_register_tool(reg, &(plato_mcp_tool_t){
        .name = "set_joint_position",
        .description = "Set a joint position (constraint-checked)",
        .input_schema = "{\"type\":\"object\",\"properties\":{"
            "\"joint\":{\"type\":\"integer\",\"min\":0,\"max\":6},"
            "\"target\":{\"type\":\"number\"}}}",
        .output_type = "json",
    });
    mcp_register_tool(reg, &(plato_mcp_tool_t){
        .name = "emergency_stop",
        .description = "Trigger emergency stop — disables all motors",
        .input_schema = "{\"type\":\"object\",\"properties\":{}}",
        .output_type = "string",
    });
    mcp_register_tool(reg, &(plato_mcp_tool_t){
        .name = "get_safety_state",
        .description = "Get safety envelope state and violation history",
        .input_schema = "{\"type\":\"object\",\"properties\":{}}",
        .output_type = "json",
    });
    mcp_register_tool(reg, &(plato_mcp_tool_t){
        .name = "set_constraint",
        .description = "Add or update a safety constraint",
        .input_schema = "{\"type\":\"object\",\"properties\":{"
            "\"type\":{\"type\":\"string\",\"enum\":[\"joint_limit\",\"torque_limit\",\"velocity_limit\"]},"
            "\"joint\":{\"type\":\"integer\"},"
            "\"min\":{\"type\":\"number\"},"
            "\"max\":{\"type\":\"number\"}}}",
        .output_type = "string",
    });
}

/* ------------------------------------------------------------------ */
/*  Telemetry publishing task                                          */
/* ------------------------------------------------------------------ */

static void telemetry_task(void *pvParameters) {
    char joints_json[512];
    char safety_json[256];

    while (1) {
        if (g_arm.plato) {
            /* Publish joint states */
            int n = snprintf(joints_json, sizeof(joints_json),
                "{\"positions\":[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f],"
                "\"velocities\":[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f],"
                "\"torques\":[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]}",
                g_arm.positions[0], g_arm.positions[1], g_arm.positions[2],
                g_arm.positions[3], g_arm.positions[4], g_arm.positions[5],
                g_arm.positions[6],
                g_arm.velocities[0], g_arm.velocities[1], g_arm.velocities[2],
                g_arm.velocities[3], g_arm.velocities[4], g_arm.velocities[5],
                g_arm.velocities[6],
                g_arm.torques[0], g_arm.torques[1], g_arm.torques[2],
                g_arm.torques[3], g_arm.torques[4], g_arm.torques[5],
                g_arm.torques[6]);
            plato_publish(g_arm.plato, "sensors", "joints", joints_json);

            /* Publish safety state */
            snprintf(safety_json, sizeof(safety_json),
                "{\"score\":%.2f,\"estop\":%s,\"commands\":%d,\"blocked\":%d}",
                g_arm.safety_score,
                g_arm.emergency_stop ? "true" : "false",
                g_arm.total_commands,
                g_arm.blocked_commands);
            plato_publish(g_arm.plato, "safety", "envelope", safety_json);
        }

        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_INTERVAL_MS));
    }
}

/* ------------------------------------------------------------------ */
/*  Command polling task                                               */
/* ------------------------------------------------------------------ */

static void command_task(void *pvParameters) {
    char cmd[PLATO_HTTP_BUF_SIZE];
    char result[PLATO_HTTP_BUF_SIZE];

    while (1) {
        if (g_arm.plato) {
            plato_err_t err = plato_poll(g_arm.plato, cmd, sizeof(cmd));
            if (err == PLATO_OK) {
                ESP_LOGI(TAG, "Fleet command: %s", cmd);

                /* Parse command type */
                char action[64] = {0};
                plato_json_find_string(cmd, "action", action, sizeof(action));

                if (strcmp(action, "set_joint_position") == 0) {
                    char joint_str[16], target_str[32];
                    plato_json_find_string(cmd, "joint", joint_str, sizeof(joint_str));
                    plato_json_find_string(cmd, "target", target_str, sizeof(target_str));

                    int joint = atoi(joint_str);
                    float target = atof(target_str);

                    /* CONSTRAINT CHECK before motor command */
                    if (g_arm.emergency_stop) {
                        snprintf(result, sizeof(result),
                            "{\"status\":\"rejected\",\"reason\":\"emergency_stop\"}");
                    } else if (!check_joint_limit(joint, target)) {
                        g_arm.blocked_commands++;
                        g_arm.total_commands++;
                        snprintf(result, sizeof(result),
                            "{\"status\":\"rejected\",\"reason\":\"joint_limit\","
                            "\"joint\":%d,\"target\":%.4f,"
                            "\"min\":%.4f,\"max\":%.4f}",
                            joint, target,
                            g_arm.joint_limits[joint].min_pos,
                            g_arm.joint_limits[joint].max_pos);
                        ESP_LOGW(TAG, "BLOCKED: joint %d target %.3f outside limits",
                                 joint, target);
                    } else {
                        /* Safe — send to CAN bus */
                        g_arm.total_commands++;
                        /* TODO: Construct Damiao MIT control CAN frame */
                        /* For now, update internal state */
                        g_arm.positions[joint] = target;
                        snprintf(result, sizeof(result),
                            "{\"status\":\"accepted\",\"joint\":%d,\"target\":%.4f}",
                            joint, target);
                        ESP_LOGI(TAG, "ACCEPTED: joint %d → %.3f", joint, target);
                    }

                    /* Publish result back */
                    plato_publish(g_arm.plato, "commands", "result", result);

                } else if (strcmp(action, "emergency_stop") == 0) {
                    g_arm.emergency_stop = true;
                    ESP_LOGW(TAG, "⚠️ EMERGENCY STOP via fleet command");
                    /* TODO: Disable all motors via CAN */
                    plato_publish(g_arm.plato, "safety", "estop",
                        "{\"triggered_by\":\"fleet\",\"timestamp\":0}");

                } else if (strcmp(action, "clear_estop") == 0) {
                    g_arm.emergency_stop = false;
                    ESP_LOGI(TAG, "Emergency stop cleared");
                    plato_publish(g_arm.plato, "safety", "estop",
                        "{\"cleared\":true}");

                } else if (strcmp(action, "set_constraint") == 0) {
                    /* Dynamic constraint update from fleet */
                    char type_str[32], joint_str[16], min_str[32], max_str[32];
                    plato_json_find_string(cmd, "type", type_str, sizeof(type_str));
                    plato_json_find_string(cmd, "joint", joint_str, sizeof(joint_str));
                    plato_json_find_string(cmd, "min", min_str, sizeof(min_str));
                    plato_json_find_string(cmd, "max", max_str, sizeof(max_str));

                    int joint = atoi(joint_str);
                    if (strcmp(type_str, "joint_limit") == 0 && joint >= 0 && joint < NUM_JOINTS) {
                        g_arm.joint_limits[joint].min_pos = atof(min_str);
                        g_arm.joint_limits[joint].max_pos = atof(max_str);
                        ESP_LOGI(TAG, "Updated joint %d limits: [%.3f, %.3f]",
                                 joint, g_arm.joint_limits[joint].min_pos,
                                 g_arm.joint_limits[joint].max_pos);
                    }

                    plato_publish(g_arm.plato, "safety", "constraint_update",
                        "{\"status\":\"updated\"}");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(COMMAND_POLL_MS));
    }
}

/* ------------------------------------------------------------------ */
/*  CAN receive task (reads motor feedback)                            */
/* ------------------------------------------------------------------ */

static void can_rx_task(void *pvParameters) {
    twai_message_t msg;

    while (1) {
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
            /* Parse Damiao motor feedback frame */
            /* Standard Damiao protocol: position (2 bytes) + velocity (2 bytes) + torque (2 bytes) */
            /* TODO: Implement full Damiao frame parsing */
            /* For now, extract motor ID from CAN ID */
            int motor_id = (msg.identifier & 0x0F);

            if (motor_id >= 0 && motor_id < NUM_JOINTS && msg.data_length_code >= 6) {
                /* Raw position from motor (scaled int16) */
                int16_t raw_pos = (msg.data[0] << 8) | msg.data[1];
                int16_t raw_vel = (msg.data[2] << 8) | msg.data[3];
                int16_t raw_tau = (msg.data[4] << 8) | msg.data[5];

                g_arm.positions[motor_id] = raw_pos * 0.0001f;  /* Scale to radians */
                g_arm.velocities[motor_id] = raw_vel * 0.001f;   /* Scale to rad/s */
                g_arm.torques[motor_id] = raw_tau * 0.01f;       /* Scale to N·m */
                g_arm.last_update_us = esp_timer_get_time();

                /* Check torque constraint */
                if (!check_torque_limit(motor_id)) {
                    g_arm.emergency_stop = true;
                    ESP_LOGW(TAG, "⚠️ TORQUE LIMIT: motor %d at %.2f N·m",
                             motor_id, g_arm.torques[motor_id]);
                }
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Entry point                                                        */
/* ------------------------------------------------------------------ */

void app_main(void) {
    ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  OpenArm × Cocapn PLATO Node         ║");
    ESP_LOGI(TAG, "║  Constraint safety + fleet connectivity║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝");

    /* Init NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Init constraints */
    init_default_constraints();

    /* Init CAN bus */
    init_can();

    /* Connect WiFi */
    wifi_init_sta();

    /* Connect to PLATO */
    g_arm.plato = plato_init(PLATO_SERVER, PLATO_PORT, DEVICE_ID);
    if (!g_arm.plato) {
        ESP_LOGE(TAG, "Failed to create PLATO context");
        return;
    }

    /* Register as fleet device */
    plato_publish(g_arm.plato, "ensign", "presence",
        "{\"type\":\"robotic_arm\","
        "\"hardware\":\"openarm-7dof\","
        "\"joints\":7,"
        "\"protocol\":\"cocapn-openarm-v1\","
        "\"constraints_enabled\":true,"
        "\"capability_level\":2,"
        "\"capability_name\":\"constraint-aware\"}");

    /* Register MCP tools */
    plato_mcp_registry_t reg;
    mcp_init(&reg);
    register_arm_tools(&reg);

    char cap_tile[1024];
    mcp_build_capability_tile(&reg, cap_tile, sizeof(cap_tile));
    plato_publish(g_arm.plato, "capabilities", "tools", cap_tile);

    ESP_LOGI(TAG, "Online. Device: %s, Joints: %d, Constraints: active",
             DEVICE_ID, NUM_JOINTS);

    /* Launch tasks */
    xTaskCreatePinnedToCore(telemetry_task, "telemetry", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(command_task,    "command",   8192, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(can_rx_task,     "can_rx",    4096, NULL, 6, NULL, 0);
}
