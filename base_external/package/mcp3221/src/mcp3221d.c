// base_external/package/mcp3221/src/mcp3221d.c
#define _GNU_SOURCE

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sqlite3.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

typedef struct {
    int bus;
    int addr;
    double vref;
    unsigned period_ms;
    const char *db_path;
    int http_port;
} config_t;

typedef struct {
    int64_t ts_unix_ms;
    uint16_t raw12;
    double vref;
    double vadc;
    double ph;          // NAN if unknown
    int status;         // 0 ok, 1 warn, 2 fault
    char faults[128];
} reading_t;

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static reading_t g_latest;

static int64_t now_unix_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static void iso8601_utc_from_ms(int64_t ms, char *out, size_t out_sz)
{
    time_t sec = (time_t)(ms / 1000);
    struct tm tm;
    gmtime_r(&sec, &tm);
    int msec = (int)(ms % 1000);
    snprintf(out, out_sz,
             "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec, msec);
}

static void msleep(unsigned ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

static int mkdir_p(const char *dir, mode_t mode)
{
    char tmp[256];
    snprintf(tmp, sizeof(tmp), "%s", dir);
    size_t len = strlen(tmp);
    if (len == 0) return -1;
    if (tmp[len - 1] == '/') tmp[len - 1] = '\0';

    for (char *p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = '\0';
            if (mkdir(tmp, mode) != 0 && errno != EEXIST) return -1;
            *p = '/';
        }
    }
    if (mkdir(tmp, mode) != 0 && errno != EEXIST) return -1;
    return 0;
}

static int read_mcp3221_raw12(int bus, int addr, uint16_t *raw12_out)
{
    char dev[32];
    snprintf(dev, sizeof(dev), "/dev/i2c-%d", bus);

    int fd = open(dev, O_RDWR);
    if (fd < 0) return -1;

    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        close(fd);
        return -1;
    }

    uint8_t buf[2] = {0};
    ssize_t n = read(fd, buf, 2);
    close(fd);
    if (n != 2) return -1;

    uint16_t v16 = ((uint16_t)buf[0] << 8) | buf[1];
    *raw12_out = (v16 >> 4) & 0x0FFF;
    return 0;
}

static int db_init(sqlite3 **db, const char *path)
{
    // Ensure parent dir exists (path like /var/lib/mcp3221/mcp3221.sqlite)
    char dir[256];
    snprintf(dir, sizeof(dir), "%s", path);
    char *slash = strrchr(dir, '/');
    if (slash) {
        *slash = '\0';
        if (mkdir_p(dir, 0755) != 0) return -1;
    }

    if (sqlite3_open(path, db) != SQLITE_OK) return -1;

    const char *sql =
        "PRAGMA journal_mode=WAL;"
        "CREATE TABLE IF NOT EXISTS readings ("
        "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  ts_unix_ms INTEGER NOT NULL,"
        "  raw12 INTEGER NOT NULL,"
        "  vref REAL NOT NULL,"
        "  vadc REAL NOT NULL,"
        "  ph REAL,"
        "  status INTEGER NOT NULL,"
        "  faults TEXT"
        ");"
        "CREATE INDEX IF NOT EXISTS idx_readings_ts ON readings(ts_unix_ms);";

    char *err = NULL;
    if (sqlite3_exec(*db, sql, NULL, NULL, &err) != SQLITE_OK) {
        sqlite3_free(err);
        return -1;
    }
    return 0;
}

static int db_insert_reading(sqlite3 *db, const reading_t *r)
{
    const char *sql =
        "INSERT INTO readings(ts_unix_ms, raw12, vref, vadc, ph, status, faults) "
        "VALUES(?,?,?,?,?,?,?);";
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &st, NULL) != SQLITE_OK) return -1;

    sqlite3_bind_int64(st, 1, r->ts_unix_ms);
    sqlite3_bind_int(st, 2, (int)r->raw12);
    sqlite3_bind_double(st, 3, r->vref);
    sqlite3_bind_double(st, 4, r->vadc);
    if (isnan(r->ph)) sqlite3_bind_null(st, 5);
    else sqlite3_bind_double(st, 5, r->ph);
    sqlite3_bind_int(st, 6, r->status);
    sqlite3_bind_text(st, 7, r->faults, -1, SQLITE_TRANSIENT);

    int rc = sqlite3_step(st);
    sqlite3_finalize(st);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

static void *sampler_thread(void *arg)
{
    const config_t *cfg = (const config_t *)arg;

    sqlite3 *db = NULL;
    if (db_init(&db, cfg->db_path) != 0) {
        fprintf(stderr, "db_init failed for %s\n", cfg->db_path);
        return NULL;
    }

    for (;;) {
        reading_t r;
        memset(&r, 0, sizeof(r));
        r.ts_unix_ms = now_unix_ms();
        r.vref = cfg->vref;
        r.ph = NAN;
        r.status = 0;
        r.faults[0] = '\0';

        uint16_t raw12 = 0;
        if (read_mcp3221_raw12(cfg->bus, cfg->addr, &raw12) != 0) {
            r.status = 2;
            snprintf(r.faults, sizeof(r.faults), "i2c_read_failed");
        } else {
            r.raw12 = raw12;
            r.vadc = ((double)raw12 * cfg->vref) / 4095.0;
        }

        (void)db_insert_reading(db, &r);

        pthread_mutex_lock(&g_lock);
        g_latest = r;
        pthread_mutex_unlock(&g_lock);

        msleep(cfg->period_ms);
    }

    sqlite3_close(db);
    return NULL;
}

static void http_send(int cfd, const char *ctype, const char *body)
{
    char hdr[256];
    size_t blen = strlen(body);
    int n = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: %s\r\n"
        "Cache-Control: no-store\r\n"
        "Content-Length: %zu\r\n"
        "\r\n",
        ctype, blen);
    (void)write(cfd, hdr, (size_t)n);
    (void)write(cfd, body, blen);
}

static void http_send_404(int cfd)
{
    const char *body = "404\n";
    char hdr[256];
    int n = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 404 Not Found\r\n"
        "Content-Type: text/plain\r\n"
        "Content-Length: %zu\r\n"
        "\r\n",
        strlen(body));
    (void)write(cfd, hdr, (size_t)n);
    (void)write(cfd, body, strlen(body));
}

static void build_latest_json(char *out, size_t out_sz)
{
    reading_t r;
    pthread_mutex_lock(&g_lock);
    r = g_latest;
    pthread_mutex_unlock(&g_lock);

    char ts[64];
    iso8601_utc_from_ms(r.ts_unix_ms, ts, sizeof(ts));

    const char *status_s = (r.status == 0) ? "ok" : (r.status == 1) ? "warn" : "fault";

    snprintf(out, out_sz,
        "{"
        "\"ts\":\"%s\","
        "\"ts_unix_ms\":%lld,"
        "\"raw_adc\":%u,"
        "\"vref\":%.6f,"
        "\"vadc\":%.6f,"
        "\"ph\":null,"
        "\"status\":\"%s\","
        "\"faults\":\"%s\""
        "}",
        ts, (long long)r.ts_unix_ms, r.raw12, r.vref, r.vadc, status_s, r.faults);
}

static void build_health_json(char *out, size_t out_sz)
{
    int64_t ms = now_unix_ms();
    char ts[64];
    iso8601_utc_from_ms(ms, ts, sizeof(ts));
    snprintf(out, out_sz, "{\"status\":\"ok\",\"ts\":\"%s\"}", ts);
}

static void serve_client(int cfd)
{
    char req[1024];
    ssize_t n = read(cfd, req, sizeof(req) - 1);
    if (n <= 0) return;
    req[n] = '\0';

    char method[8] = {0};
    char path[256] = {0};
    if (sscanf(req, "%7s %255s", method, path) != 2) {
        http_send_404(cfd);
        return;
    }

    if (strcmp(method, "GET") != 0) {
        http_send_404(cfd);
        return;
    }

    if (strcmp(path, "/api/v1/health") == 0) {
        char json[256];
        build_health_json(json, sizeof(json));
        http_send(cfd, "application/json", json);
        return;
    }

    if (strcmp(path, "/api/v1/latest") == 0) {
        char json[512];
        build_latest_json(json, sizeof(json));
        http_send(cfd, "application/json", json);
        return;
    }

    if (strcmp(path, "/") == 0) {
        const char *html =
            "<!doctype html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>MCP3221</title>"
            "<style>body{font-family:sans-serif;margin:2rem}.v{font-size:3rem}</style>"
            "</head><body>"
            "<h1>MCP3221 Monitor</h1>"
            "<div>Time: <span id='ts'>-</span></div>"
            "<div>Status: <span id='st'>-</span></div>"
            "<div class='v'><span id='vadc'>-</span> V</div>"
            "<div>Raw: <span id='raw'>-</span></div>"
            "<script>"
            "async function tick(){"
            " const r=await fetch('/api/v1/latest',{cache:'no-store'});"
            " const j=await r.json();"
            " document.getElementById('ts').textContent=j.ts;"
            " document.getElementById('st').textContent=j.status;"
            " document.getElementById('vadc').textContent=(j.vadc ?? '-');"
            " document.getElementById('raw').textContent=(j.raw_adc ?? '-');"
            "}"
            "tick(); setInterval(tick,1000);"
            "</script>"
            "</body></html>";
        http_send(cfd, "text/html", html);
        return;
    }

    http_send_404(cfd);
}

static void *http_thread(void *arg)
{
    const config_t *cfg = (const config_t *)arg;

    int sfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sfd < 0) {
        perror("socket");
        return NULL;
    }

    int yes = 1;
    setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)cfg->http_port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sfd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        perror("bind");
        close(sfd);
        return NULL;
    }

    if (listen(sfd, 8) != 0) {
        perror("listen");
        close(sfd);
        return NULL;
    }

    for (;;) {
        int cfd = accept(sfd, NULL, NULL);
        if (cfd < 0) continue;
        serve_client(cfd);
        close(cfd);
    }

    close(sfd);
    return NULL;
}

static void usage(const char *argv0)
{
    fprintf(stderr,
            "Usage: %s [--bus N] [--addr 0x4d] [--vref 3.3] [--period-ms 1000]\n"
            "          [--db /var/lib/mcp3221/mcp3221.sqlite] [--port 8080]\n",
            argv0);
}

int main(int argc, char **argv)
{
    config_t cfg = {
        .bus = 1,
        .addr = 0x4d,
        .vref = 3.3,
        .period_ms = 1000,
        .db_path = "/var/lib/mcp3221/mcp3221.sqlite",
        .http_port = 8080,
    };

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--bus") && i + 1 < argc) cfg.bus = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--addr") && i + 1 < argc) cfg.addr = (int)strtol(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--vref") && i + 1 < argc) cfg.vref = atof(argv[++i]);
        else if (!strcmp(argv[i], "--period-ms") && i + 1 < argc) cfg.period_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--db") && i + 1 < argc) cfg.db_path = argv[++i];
        else if (!strcmp(argv[i], "--port") && i + 1 < argc) cfg.http_port = atoi(argv[++i]);
        else { usage(argv[0]); return 2; }
    }

    pthread_mutex_lock(&g_lock);
    memset(&g_latest, 0, sizeof(g_latest));
    g_latest.ts_unix_ms = now_unix_ms();
    g_latest.vref = cfg.vref;
    g_latest.ph = NAN;
    snprintf(g_latest.faults, sizeof(g_latest.faults), "no_data_yet");
    pthread_mutex_unlock(&g_lock);

    pthread_t th_samp, th_http;

    if (pthread_create(&th_samp, NULL, sampler_thread, &cfg) != 0) {
        fprintf(stderr, "pthread_create sampler failed\n");
        return 1;
    }

    if (pthread_create(&th_http, NULL, http_thread, &cfg) != 0) {
        fprintf(stderr, "pthread_create http failed\n");
        return 1;
    }

    for (;;) pause();
    return 0;
}
