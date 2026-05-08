#ifndef PLATO_MCP_H
#define PLATO_MCP_H

#include "plato_client.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  MCP Tool definition                                                */
/* ------------------------------------------------------------------ */

/**
 * plato_mcp_tool_t - Describes a single MCP tool (sensor or actuator).
 *
 * Tools are the bridge between PLATO room capabilities and agent commands.
 * Each sensor or actuator on the device registers as one tool.
 */
typedef struct {
    const char *name;        /* Tool name (e.g. "read_temperature")   */
    const char *description; /* Human-readable description            */
    const char *input_schema;/* JSON string: { "type":"object", ... } */
    const char *output_type; /* "string", "number", "boolean", "json" */
} plato_mcp_tool_t;

/* ------------------------------------------------------------------ */
/*  Device capability levels (Turbo-Shell)                             */
/* ------------------------------------------------------------------ */

typedef enum {
    PLATO_CAP_RAW      = 0,  /* Level 0: Raw sensor readings only        */
    PLATO_CAP_CONDITIONED = 1, /* Level 1: Thresholds, filtering         */
    PLATO_CAP_SMART    = 2,  /* Level 2: Context-aware decisions         */
    PLATO_CAP_AUTONOMOUS = 3,/* Level 3: Autonomous agent behavior       */
    PLATO_CAP_ENSIGN   = 4,  /* Level 4: Fleet-level coordination        */
} plato_cap_level_t;

/* ------------------------------------------------------------------ */
/*  MCP Registry                                                       */
/* ------------------------------------------------------------------ */

typedef struct {
    plato_mcp_tool_t tools[PLATO_MAX_TOOLS];
    size_t           tool_count;
    plato_cap_level_t cap_level;
    char             capability_json[PLATO_MAX_ANSWER];
} plato_mcp_registry_t;

/**
 * mcp_init() - Initialize the MCP registry for a device.
 * Sets default capability level and clears the tool list.
 */
void mcp_init(plato_mcp_registry_t *reg);

/**
 * mcp_register_tool() - Add a tool definition to the registry.
 * Returns PLATO_OK on success, PLATO_ERR_MEM if tool_count would exceed max.
 */
plato_err_t mcp_register_tool(plato_mcp_registry_t *reg, const plato_mcp_tool_t *tool);

/**
 * mcp_build_capability_tile() - Generate the capability JSON tile
 * for publishing to PLATO.
 *
 * Format:
 * {
 *   "level": <int>,
 *   "level_name": "<name>",
 *   "tools": [ { "name":"...", "description":"...", "input_schema":... }, ... ]
 * }
 *
 * @out_buf:  Buffer to receive the JSON string
 * @buf_size: Size of out_buf
 */
void mcp_build_capability_tile(plato_mcp_registry_t *reg,
                                char *out_buf, size_t buf_size);

/**
 * mcp_handle_command() - Parse and execute an MCP command JSON.
 *
 * Expected JSON:
 * {
 *   "tool": "<tool_name>",
 *   "arguments": { ... },
 *   "intelligence": "<new behavior code (optional)>"
 * }
 *
 * If "intelligence" is present, the device stores it as upgraded behavior
 * and advances its capability level (embodiment / turbo-shell).
 *
 * @cmd_json:    Raw command JSON string
 * @reg:         MCP registry (may be updated by embodiment)
 * @result_buf:  Buffer for the result JSON
 * @result_size: Size of result_buf
 *
 * Returns PLATO_OK on successful execution or intelligence receipt,
 * PLATO_ERR_PARSE if the command was malformed.
 */
plato_err_t mcp_handle_command(const char *cmd_json,
                                plato_mcp_registry_t *reg,
                                char *result_buf, size_t result_size);

/**
 * mcp_get_capability_name() - Return string name for a cap level.
 */
const char *mcp_get_capability_name(plato_cap_level_t level);

#ifdef __cplusplus
}
#endif

#endif /* PLATO_MCP_H */
