#include "plato_mcp.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ------------------------------------------------------------------ */
/*  Capability level names                                             */
/* ------------------------------------------------------------------ */

static const char *cap_names[] = {
    "raw",
    "conditioned",
    "smart",
    "autonomous",
    "ensign"
};

const char *mcp_get_capability_name(plato_cap_level_t level)
{
    if ((size_t)level < sizeof(cap_names) / sizeof(cap_names[0]))
        return cap_names[level];
    return "unknown";
}

/* ------------------------------------------------------------------ */
/*  Registry                                                           */
/* ------------------------------------------------------------------ */

void mcp_init(plato_mcp_registry_t *reg)
{
    memset(reg, 0, sizeof(*reg));
    reg->cap_level = PLATO_CAP_RAW;
}

plato_err_t mcp_register_tool(plato_mcp_registry_t *reg,
                               const plato_mcp_tool_t *tool)
{
    if (reg->tool_count >= PLATO_MAX_TOOLS)
        return PLATO_ERR_MEM;

    size_t idx = reg->tool_count++;
    reg->tools[idx] = *tool;
    return PLATO_OK;
}

void mcp_build_capability_tile(plato_mcp_registry_t *reg,
                                char *out_buf, size_t buf_size)
{
    size_t offset = 0;
    int n;

    n = snprintf(out_buf + offset, buf_size - offset,
        "{\"level\":%d,\"level_name\":\"%s\",\"tools\":[",
        (int)reg->cap_level,
        mcp_get_capability_name(reg->cap_level));
    if (n > 0) offset += (size_t)n;

    for (size_t i = 0; i < reg->tool_count; i++) {
        if (i > 0) {
            n = snprintf(out_buf + offset, buf_size - offset, ",");
            if (n > 0) offset += (size_t)n;
        }
        n = snprintf(out_buf + offset, buf_size - offset,
            "{\"name\":\"%s\",\"description\":\"%s\",\"input_schema\":%s}",
            reg->tools[i].name,
            reg->tools[i].description,
            reg->tools[i].input_schema);
        if (n > 0) offset += (size_t)n;
        if (offset >= buf_size) break;
    }

    n = snprintf(out_buf + offset, buf_size - offset, "]}");
    if (n > 0) offset += (size_t)n;

    out_buf[buf_size - 1] = '\0';
}

/* ------------------------------------------------------------------ */
/*  Command handler                                                     */
/* ------------------------------------------------------------------ */

static int simple_strcmp(const char *a, const char *b)
{
    return strcmp(a, b);
}

plato_err_t mcp_handle_command(const char *cmd_json,
                                plato_mcp_registry_t *reg,
                                char *result_buf, size_t result_size)
{
    if (!cmd_json || !reg || !result_buf || result_size == 0)
        return PLATO_ERR_PARSE;

    /* ---- Check for intelligence upgrade (embodiment) ---- */
    char intelligence[PLATO_MAX_ANSWER];
    if (plato_json_find_string(cmd_json, "intelligence",
                                intelligence, sizeof(intelligence)) == PLATO_OK) {
        /* Store the intelligence as our new behavior */
        strncpy(reg->capability_json, intelligence, sizeof(reg->capability_json) - 1);
        reg->capability_json[sizeof(reg->capability_json) - 1] = '\0';

        /* Advance capability level (capped at ensign) */
        if (reg->cap_level < PLATO_CAP_ENSIGN) {
            reg->cap_level = (plato_cap_level_t)((int)reg->cap_level + 1);
        }

        snprintf(result_buf, result_size,
            "{\"status\":\"upgraded\",\"new_level\":\"%s\"}",
            mcp_get_capability_name(reg->cap_level));
        return PLATO_OK;
    }

    /* ---- Extract tool name ---- */
    char tool_name[64];
    if (plato_json_find_string(cmd_json, "tool",
                                tool_name, sizeof(tool_name)) != PLATO_OK) {
        snprintf(result_buf, result_size,
            "{\"status\":\"error\",\"message\":\"missing tool field\"}");
        return PLATO_ERR_PARSE;
    }

    /* ---- Find and "execute" the tool ---- */
    /* In a real implementation, each tool would have a handler function.
     * Here we return success with the tool name echoed back.
     * The firmware example overrides this with real sensor reads.
     */
    for (size_t i = 0; i < reg->tool_count; i++) {
        if (simple_strcmp(reg->tools[i].name, tool_name) == 0) {
            snprintf(result_buf, result_size,
                "{\"status\":\"ok\",\"tool\":\"%s\",\"result\":\"executed\"}",
                tool_name);
            return PLATO_OK;
        }
    }

    snprintf(result_buf, result_size,
        "{\"status\":\"error\",\"message\":\"unknown tool: %s\"}", tool_name);
    return PLATO_ERR_PARSE;
}
