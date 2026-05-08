#include "plato_client.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* ------------------------------------------------------------------ */
/*  Socket abstraction layer                                           */
/*  (Platform-specific — wraps BSD sockets / lwIP / ESP-IDF net)      */
/* ------------------------------------------------------------------ */

#ifdef ESP_PLATFORM
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#define PLATO_SOCKET_TYPE  int
#define PLATO_INVALID_SOCK -1
#define PLATO_CLOSE(fd)    close(fd)
#elif defined(PICO_SDK)
#include "pico/cyw43_arch.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#define PLATO_SOCKET_TYPE  int
#define PLATO_INVALID_SOCK -1
#define PLATO_CLOSE(fd)    close(fd)
#else
/* POSIX desktop fallback */
#include <sys/socket.h>
#include <netdb.h>
#include <fcntl.h>
#include <errno.h>
#define PLATO_SOCKET_TYPE  int
#define PLATO_INVALID_SOCK -1
#define PLATO_CLOSE(fd)    close(fd)
#endif

/* ------------------------------------------------------------------ */
/*  Internal helpers                                                   */
/* ------------------------------------------------------------------ */

struct plato_ctx {
    char    server_ip[16];
    uint16_t port;
    char    device_id[PLATO_MAX_ROOM_NAME];
    char    http_buf[PLATO_HTTP_BUF_SIZE];
};

static int plato_tcp_connect(const char *host, uint16_t port)
{
    struct sockaddr_in addr;
    PLATO_SOCKET_TYPE fd;
    struct hostent *he;

    he = gethostbyname(host);
    if (!he) return PLATO_INVALID_SOCK;

    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return PLATO_INVALID_SOCK;

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        PLATO_CLOSE(fd);
        return PLATO_INVALID_SOCK;
    }
    return fd;
}

static int plato_http_request(plato_ctx_t *ctx, const char *method,
                              const char *path, const char *body,
                              char *out_buf, size_t out_size)
{
    PLATO_SOCKET_TYPE fd = plato_tcp_connect(ctx->server_ip, ctx->port);
    if (fd < 0) return PLATO_ERR_CONNECT;

    /* Build HTTP request */
    char *buf = ctx->http_buf;
    int n;

    if (body) {
        n = snprintf(buf, sizeof(ctx->http_buf),
            "%s %s HTTP/1.1\r\n"
            "Host: %s:%u\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %zu\r\n"
            "Connection: close\r\n"
            "\r\n"
            "%s",
            method, path,
            ctx->server_ip, ctx->port,
            strlen(body),
            body);
    } else {
        n = snprintf(buf, sizeof(ctx->http_buf),
            "%s %s HTTP/1.1\r\n"
            "Host: %s:%u\r\n"
            "Connection: close\r\n"
            "\r\n",
            method, path,
            ctx->server_ip, ctx->port);
    }

    if (n < 0 || (size_t)n >= sizeof(ctx->http_buf)) {
        PLATO_CLOSE(fd);
        return PLATO_ERR_MEM;
    }

    /* Send */
    size_t sent = 0, total = (size_t)n;
    while (sent < total) {
        int w = send(fd, buf + sent, total - sent, 0);
        if (w < 0) { PLATO_CLOSE(fd); return PLATO_ERR_SEND; }
        sent += (size_t)w;
    }

    /* Receive response */
    size_t received = 0;
    while (received < out_size - 1) {
        int r = recv(fd, out_buf + received, out_size - 1 - received, 0);
        if (r < 0) { PLATO_CLOSE(fd); return PLATO_ERR_RECV; }
        if (r == 0) break; /* connection closed */
        received += (size_t)r;
    }
    out_buf[received] = '\0';

    PLATO_CLOSE(fd);

    /* Strip HTTP headers: find first "\r\n\r\n" */
    char *header_end = strstr(out_buf, "\r\n\r\n");
    if (header_end) {
        header_end += 4; /* skip past the blank line */
        size_t body_len = received - (size_t)(header_end - out_buf);
        memmove(out_buf, header_end, body_len);
        out_buf[body_len] = '\0';
    }

    return PLATO_OK;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

plato_ctx_t *plato_init(const char *server_ip, uint16_t port,
                         const char *device_id)
{
    plato_ctx_t *ctx = (plato_ctx_t *)calloc(1, sizeof(plato_ctx_t));
    if (!ctx) return NULL;

    strncpy(ctx->server_ip, server_ip, sizeof(ctx->server_ip) - 1);
    ctx->server_ip[sizeof(ctx->server_ip) - 1] = '\0';
    ctx->port = port ? port : PLATO_DEFAULT_PORT;
    strncpy(ctx->device_id, device_id, sizeof(ctx->device_id) - 1);
    ctx->device_id[sizeof(ctx->device_id) - 1] = '\0';

    return ctx;
}

void plato_destroy(plato_ctx_t *ctx)
{
    if (ctx) free(ctx);
}

plato_err_t plato_publish(plato_ctx_t *ctx, const char *domain,
                           const char *question, const char *answer)
{
    /*
     * PLATO tile POST format (JSON):
     * {
     *   "room": "<device_id>",
     *   "domain": "<domain>",
     *   "question": "<question>",
     *   "answer": <answer>
     * }
     */
    char body[PLATO_HTTP_BUF_SIZE];
    int n = snprintf(body, sizeof(body),
        "{\"room\":\"%s\",\"domain\":\"%s\",\"question\":\"%s\",\"answer\":%s}",
        ctx->device_id, domain, question, answer);
    if (n < 0 || (size_t)n >= sizeof(body)) return PLATO_ERR_MEM;

    char path[PLATO_MAX_ROOM_NAME + 32];
    snprintf(path, sizeof(path), "/submit");

    char resp[PLATO_HTTP_BUF_SIZE];
    return (plato_err_t)plato_http_request(ctx, "POST", path, body, resp, sizeof(resp));
}

plato_err_t plato_fetch(plato_ctx_t *ctx, const char *room_name,
                         char *out_buf, size_t buf_size)
{
    char path[PLATO_MAX_ROOM_NAME + 32];
    snprintf(path, sizeof(path), "/room/%s", room_name);

    return (plato_err_t)plato_http_request(ctx, "GET", path, NULL,
                                           out_buf, buf_size);
}

plato_err_t plato_poll(plato_ctx_t *ctx, char *out_cmd, size_t cmd_size)
{
    /*
     * Poll the device's own command room.
     * Convention: commands are posted to "<device_id>/commands".
     */
    char room[PLATO_MAX_ROOM_NAME + 16];
    snprintf(room, sizeof(room), "%s/commands", ctx->device_id);

    plato_err_t err = plato_fetch(ctx, room, out_cmd, cmd_size);
    if (err != PLATO_OK) return err;

    /* If the response is an empty array or "[]", no command pending */
    if (strlen(out_cmd) == 0 || strcmp(out_cmd, "[]") == 0 ||
        strcmp(out_cmd, "{}") == 0) {
        return PLATO_ERR_PARSE;
    }
    return PLATO_OK;
}

/* ------------------------------------------------------------------ */
/*  Minimal JSON string-value extractor                                */
/* ------------------------------------------------------------------ */

plato_err_t plato_json_find_string(const char *json, const char *key,
                                    char *out_val, size_t val_size)
{
    if (!json || !key || !out_val || val_size == 0)
        return PLATO_ERR_PARSE;

    /* Build the search pattern: "key": */
    char pattern[PLATO_MAX_DOMAIN + 8];
    int n = snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    if (n < 0 || (size_t)n >= sizeof(pattern)) return PLATO_ERR_PARSE;

    const char *p = strstr(json, pattern);
    if (!p) return PLATO_ERR_PARSE;

    /* Advance past the pattern and any whitespace */
    p += strlen(pattern);
    while (*p == ' ' || *p == '\t') p++;

    /* Expect opening quote */
    if (*p != '"') return PLATO_ERR_PARSE;
    p++;

    /* Copy until closing quote */
    size_t i = 0;
    while (*p && *p != '"' && i < val_size - 1) {
        if (*p == '\\' && *(p+1)) {
            p++; /* skip escape char */
        }
        out_val[i++] = *p++;
    }
    out_val[i] = '\0';

    return (i > 0) ? PLATO_OK : PLATO_ERR_PARSE;
}
