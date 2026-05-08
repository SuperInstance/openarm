#ifndef PLATO_CLIENT_H
#define PLATO_CLIENT_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  Configuration                                                      */
/* ------------------------------------------------------------------ */

#define PLATO_DEFAULT_PORT    8847
#define PLATO_MAX_ROOM_NAME   64
#define PLATO_MAX_DOMAIN      64
#define PLATO_MAX_QUESTION    256
#define PLATO_MAX_ANSWER      1024
#define PLATO_MAX_RESPONSE    2048
#define PLATO_MAX_TOOLS       16
#define PLATO_HTTP_BUF_SIZE   4096

/* ------------------------------------------------------------------ */
/*  Error codes                                                        */
/* ------------------------------------------------------------------ */

typedef enum {
    PLATO_OK          = 0,
    PLATO_ERR_CONNECT = -1,
    PLATO_ERR_SOCKET  = -2,
    PLATO_ERR_SEND    = -3,
    PLATO_ERR_RECV    = -4,
    PLATO_ERR_PARSE   = -5,
    PLATO_ERR_TIMEOUT = -6,
    PLATO_ERR_MEM     = -7,
} plato_err_t;

/* ------------------------------------------------------------------ */
/*  Handle / context                                                   */
/* ------------------------------------------------------------------ */

typedef struct plato_ctx plato_ctx_t;

/**
 * plato_init() - Create and connect a PLATO client context.
 * @server_ip:  NUL-terminated IPv4 string (e.g. "192.168.1.100")
 * @port:       TCP port (use PLATO_DEFAULT_PORT if unsure)
 * @device_id:  Unique identifier for this device (room name in PLATO)
 *
 * Returns a heap-allocated context on success, NULL on failure.
 * Caller must free with plato_destroy().
 */
plato_ctx_t *plato_init(const char *server_ip, uint16_t port, const char *device_id);

/**
 * plato_destroy() - Free a PLATO client context.
 */
void plato_destroy(plato_ctx_t *ctx);

/**
 * plato_publish() - POST a tile to the PLATO server.
 * @domain:   Domain string (e.g. "sensors")
 * @question: Question/path string (e.g. "temperature")
 * @answer:   Answer/value string (e.g. "{\"celsius\": 23.5}")
 *
 * Returns PLATO_OK on success, or a negative error code.
 */
plato_err_t plato_publish(plato_ctx_t *ctx, const char *domain,
                          const char *question, const char *answer);

/**
 * plato_fetch() - GET tiles from a PLATO room.
 * @room_name: Room to query (e.g. "fleet-controller")
 * @out_buf:   Buffer to receive JSON response
 * @buf_size:  Size of out_buf in bytes
 *
 * Returns PLATO_OK on success, or a negative error code.
 * On success, out_buf contains the raw JSON response.
 */
plato_err_t plato_fetch(plato_ctx_t *ctx, const char *room_name,
                        char *out_buf, size_t buf_size);

/**
 * plato_poll() - Poll the PLATO server for pending commands.
 * Calls plato_fetch on a well-known command room.
 *
 * @out_cmd:  Buffer to receive the raw command JSON
 * @cmd_size: Size of out_cmd in bytes
 *
 * Returns PLATO_OK if a command was received, PLATO_ERR_PARSE if none,
 * or another negative code on transport error.
 */
plato_err_t plato_poll(plato_ctx_t *ctx, char *out_cmd, size_t cmd_size);

/**
 * plato_json_find_string() - Minimal JSON string-value extractor.
 *
 * Scans @json for the first occurrence of:
 *     "key":"value"
 * or  "key": "value"
 *
 * Copies the value (without surrounding quotes) into @out_val
 * up to @val_size bytes. Returns PLATO_OK if found, PLATO_ERR_PARSE if not.
 *
 * This is intentionally minimal — no nesting, no arrays, no escapes.
 * Fine for the structured, flat JSON PLATO tiles use.
 */
plato_err_t plato_json_find_string(const char *json, const char *key,
                                   char *out_val, size_t val_size);

#ifdef __cplusplus
}
#endif

#endif /* PLATO_CLIENT_H */
