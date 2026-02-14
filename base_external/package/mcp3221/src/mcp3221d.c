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

    // Control loop / simulation
    unsigned ctrl_period_ms; // e.g. 100 ms
} config_t;

typedef struct {
    int64_t ts_unix_ms;
    uint16_t raw12;
    double vref;
    double vadc;

    // Sprint 3: can be simulated or real (depending on sim_enabled and calibration)
    double ph; // NAN if unknown

    int status; // 0 ok, 1 warn, 2 fault
    char faults[128];
} reading_t;

typedef struct {
    // Control enable and setpoint
    int enabled;     // 0/1
    double setpoint; // target pH

    // PI controller parameters
    double kp;
    double ki;
    double u_max; // saturation on actuator command
    double i_min; // integral clamp
    double i_max; // integral clamp

    // State
    double integ; // integral state
    double u;     // last actuator output

    // Plant parameters (simple first order with drift)
    double ph;     // plant state (simulated, or mirrored from measured when sim disabled)
    double ph_eq;  // equilibrium pH when u = 0
    double k_leak; // drift rate back to equilibrium (1/s)
    double k_u;    // actuator gain (pH/s at u=1)

    // Optional measurement noise
    double noise_sigma; // pH std-dev

    // Simulation switch and measured pH
    int sim_enabled; // 1 = simulated plant, 0 = use real measured pH
    double ph_meas;  // latest measured pH (from ADC conversion), NAN if unavailable
} control_t;

typedef struct {
    int enabled;     // 0/1
    double slope;    // pH per volt
    double offset;   // pH at 0V
} ph_cal_t;

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static reading_t g_latest;
static control_t g_ctrl;
static ph_cal_t g_cal;

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

static double clamp(double x, double lo, double hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
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

// pH conversion from ADC voltage (linear calibration for Sprint 3)
static double ph_from_vadc(double v)
{
    ph_cal_t cal;
    pthread_mutex_lock(&g_lock);
    cal = g_cal;
    pthread_mutex_unlock(&g_lock);

    if (!cal.enabled) return NAN;
    return cal.slope * v + cal.offset;
}

// Box-Muller transform for approximate Gaussian noise
static double randn01(void)
{
    double u1 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
    double u2 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

/* ----------------------------- SQLite helpers ----------------------------- */

static int db_open_only(sqlite3 **db, const char *path)
{
    // Ensure parent dir exists (path like /var/lib/mcp3221/mcp3221.sqlite)
    char dir[256];
    snprintf(dir, sizeof(dir), "%s", path);
    char *slash = strrchr(dir, '/');
    if (slash) {
        *slash = '\0';
        if (mkdir_p(dir, 0755) != 0) return -1;
    }

    if (sqlite3_open(path, db) != SQLITE_OK) {
        fprintf(stderr, "sqlite3_open(%s) failed: %s\n", path, sqlite3_errmsg(*db));
        return -1;
    }

    // Wait for locks rather than failing immediately
    sqlite3_busy_timeout(*db, 2000);
    return 0;
}

static int db_schema_init_once(sqlite3 *db)
{
    // DELETE journal mode is the most compatible on embedded setups
    const char *sql =
        "PRAGMA journal_mode=DELETE;"
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
        "CREATE INDEX IF NOT EXISTS idx_readings_ts ON readings(ts_unix_ms);"
        "CREATE TABLE IF NOT EXISTS control_log ("
        "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "  ts_unix_ms INTEGER NOT NULL,"
        "  enabled INTEGER NOT NULL,"
        "  setpoint REAL NOT NULL,"
        "  ph REAL NOT NULL,"
        "  u REAL NOT NULL,"
        "  sim_enabled INTEGER NOT NULL,"
        "  ph_meas REAL"
        ");"
        "CREATE INDEX IF NOT EXISTS idx_ctrl_ts ON control_log(ts_unix_ms);";

    char *err = NULL;
    if (sqlite3_exec(db, sql, NULL, NULL, &err) != SQLITE_OK) {
        fprintf(stderr, "sqlite3_exec schema init failed: %s\n", err ? err : "(null)");
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

static int db_insert_control(sqlite3 *db, int64_t ts_unix_ms, const control_t *c)
{
    const char *sql =
        "INSERT INTO control_log(ts_unix_ms, enabled, setpoint, ph, u, sim_enabled, ph_meas) "
        "VALUES(?,?,?,?,?,?,?);";
    sqlite3_stmt *st = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &st, NULL) != SQLITE_OK) return -1;

    sqlite3_bind_int64(st, 1, ts_unix_ms);
    sqlite3_bind_int(st, 2, c->enabled);
    sqlite3_bind_double(st, 3, c->setpoint);
    sqlite3_bind_double(st, 4, c->ph);
    sqlite3_bind_double(st, 5, c->u);
    sqlite3_bind_int(st, 6, c->sim_enabled);
    if (isnan(c->ph_meas)) sqlite3_bind_null(st, 7);
    else sqlite3_bind_double(st, 7, c->ph_meas);

    int rc = sqlite3_step(st);
    sqlite3_finalize(st);
    return (rc == SQLITE_DONE) ? 0 : -1;
}

/* ----------------------------- Threads ----------------------------- */

static void *sampler_thread(void *arg)
{
    const config_t *cfg = (const config_t *)arg;

    sqlite3 *db = NULL;
    if (db_open_only(&db, cfg->db_path) != 0) {
        fprintf(stderr, "db_open_only failed for %s\n", cfg->db_path);
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

        double phm = ph_from_vadc(r.vadc);

        pthread_mutex_lock(&g_lock);
        g_ctrl.ph_meas = phm;

        // Expose pH to UI/API: sim plant pH if sim enabled, otherwise measured pH
        r.ph = g_ctrl.sim_enabled ? g_ctrl.ph : phm;

        g_latest = r;
        pthread_mutex_unlock(&g_lock);

        (void)db_insert_reading(db, &r);

        msleep(cfg->period_ms);
    }

    sqlite3_close(db);
    return NULL;
}

static void *control_thread(void *arg)
{
    const config_t *cfg = (const config_t *)arg;

    sqlite3 *db = NULL;
    if (db_open_only(&db, cfg->db_path) != 0) {
        fprintf(stderr, "db_open_only failed (control thread) for %s\n", cfg->db_path);
        return NULL;
    }

    const double dt = (double)cfg->ctrl_period_ms / 1000.0;

    for (;;) {
        int64_t ts = now_unix_ms();

        pthread_mutex_lock(&g_lock);
        control_t c = g_ctrl; // copy under lock
        pthread_mutex_unlock(&g_lock);

        // Choose measurement source
        double ph_meas = c.sim_enabled ? c.ph : c.ph_meas;

        // Add noise only to the measurement signal
        if (c.noise_sigma > 0.0 && !isnan(ph_meas)) {
            ph_meas += c.noise_sigma * randn01();
        }

        // PI compute
        if (!c.sim_enabled && isnan(ph_meas)) {
            // No valid measured pH yet: safe behavior
            c.u = 0.0;
            c.integ *= 0.98;
        } else if (c.enabled) {
            double e = c.setpoint - ph_meas;
            c.integ = clamp(c.integ + e * dt, c.i_min, c.i_max);
            c.u = clamp(c.kp * e + c.ki * c.integ, -c.u_max, c.u_max);
        } else {
            c.u = 0.0;
            c.integ *= 0.98;
            if (fabs(c.integ) < 1e-6) c.integ = 0.0;
        }

        // Plant update
        if (c.sim_enabled) {
            // ph_dot = -k_leak*(ph - ph_eq) + k_u*u
            double ph_dot = (-c.k_leak * (c.ph - c.ph_eq)) + (c.k_u * c.u);
            c.ph = c.ph + dt * ph_dot;
            c.ph = clamp(c.ph, 0.0, 14.0);
        } else {
            // In real mode, mirror plant state to measured value (if available)
            if (!isnan(c.ph_meas)) c.ph = clamp(c.ph_meas, 0.0, 14.0);
        }

        pthread_mutex_lock(&g_lock);
        g_ctrl = c;
        pthread_mutex_unlock(&g_lock);

        (void)db_insert_control(db, ts, &c);

        msleep(cfg->ctrl_period_ms);
    }

    sqlite3_close(db);
    return NULL;
}

/* ----------------------------- HTTP helpers ----------------------------- */

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

static void http_send_400(int cfd, const char *msg)
{
    char body[256];
    snprintf(body, sizeof(body), "400 %s\n", msg ? msg : "");
    char hdr[256];
    int n = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 400 Bad Request\r\n"
        "Content-Type: text/plain\r\n"
        "Content-Length: %zu\r\n"
        "\r\n",
        strlen(body));
    (void)write(cfd, hdr, (size_t)n);
    (void)write(cfd, body, strlen(body));
}

static void build_health_json(char *out, size_t out_sz)
{
    int64_t ms = now_unix_ms();
    char ts[64];
    iso8601_utc_from_ms(ms, ts, sizeof(ts));
    snprintf(out, out_sz, "{\"status\":\"ok\",\"ts\":\"%s\"}", ts);
}

static void build_latest_json(char *out, size_t out_sz)
{
    reading_t r;
    control_t c;

    pthread_mutex_lock(&g_lock);
    r = g_latest;
    c = g_ctrl;
    pthread_mutex_unlock(&g_lock);

    char ts[64];
    iso8601_utc_from_ms(r.ts_unix_ms, ts, sizeof(ts));
    const char *status_s = (r.status == 0) ? "ok" : (r.status == 1) ? "warn" : "fault";

    char ph_buf[32];
    if (isnan(r.ph)) snprintf(ph_buf, sizeof(ph_buf), "null");
    else snprintf(ph_buf, sizeof(ph_buf), "%.3f", r.ph);

    snprintf(out, out_sz,
        "{"
        "\"ts\":\"%s\","
        "\"ts_unix_ms\":%lld,"
        "\"raw_adc\":%u,"
        "\"vref\":%.6f,"
        "\"vadc\":%.6f,"
        "\"ph\":%s,"
        "\"status\":\"%s\","
        "\"faults\":\"%s\","
        "\"control\":{"
          "\"enabled\":%s,"
          "\"setpoint\":%.3f,"
          "\"u\":%.3f,"
          "\"sim_enabled\":%s"
        "}"
        "}",
        ts, (long long)r.ts_unix_ms, r.raw12, r.vref, r.vadc,
        ph_buf,
        status_s, r.faults,
        c.enabled ? "true" : "false", c.setpoint, c.u, c.sim_enabled ? "true" : "false");
}

static void build_control_state_json(char *out, size_t out_sz)
{
    control_t c;
    ph_cal_t cal;
    pthread_mutex_lock(&g_lock);
    c = g_ctrl;
    cal = g_cal;
    pthread_mutex_unlock(&g_lock);

    char ph_meas_buf[32];
    if (isnan(c.ph_meas)) snprintf(ph_meas_buf, sizeof(ph_meas_buf), "null");
    else snprintf(ph_meas_buf, sizeof(ph_meas_buf), "%.3f", c.ph_meas);

    snprintf(out, out_sz,
        "{"
        "\"enabled\":%s,"
        "\"setpoint\":%.3f,"
        "\"u\":%.3f,"
        "\"sim_enabled\":%s,"
        "\"ph_meas\":%s,"
        "\"pi\":{"
          "\"kp\":%.6f,"
          "\"ki\":%.6f,"
          "\"u_max\":%.3f,"
          "\"integ\":%.6f,"
          "\"i_min\":%.3f,"
          "\"i_max\":%.3f"
        "},"
        "\"plant\":{"
          "\"ph\":%.3f,"
          "\"ph_eq\":%.3f,"
          "\"k_leak\":%.6f,"
          "\"k_u\":%.6f,"
          "\"noise_sigma\":%.6f"
        "},"
        "\"cal\":{"
          "\"enabled\":%s,"
          "\"slope\":%.6f,"
          "\"offset\":%.6f"
        "}"
        "}",
        c.enabled ? "true" : "false", c.setpoint, c.u,
        c.sim_enabled ? "true" : "false",
        ph_meas_buf,
        c.kp, c.ki, c.u_max, c.integ, c.i_min, c.i_max,
        c.ph, c.ph_eq, c.k_leak, c.k_u, c.noise_sigma,
        cal.enabled ? "true" : "false", cal.slope, cal.offset);
}

// Minimal query parsing: expects "...?key=value&key2=value2"
static const char *query_find(const char *path, const char *key)
{
    const char *q = strchr(path, '?');
    if (!q) return NULL;
    q++;

    size_t klen = strlen(key);
    const char *p = q;
    while (*p) {
        if (strncmp(p, key, klen) == 0 && p[klen] == '=') {
            return p + klen + 1;
        }
        const char *amp = strchr(p, '&');
        if (!amp) break;
        p = amp + 1;
    }
    return NULL;
}

static int parse_double_param(const char *path, const char *key, double *out)
{
    const char *v = query_find(path, key);
    if (!v) return -1;
    char *end = NULL;
    double d = strtod(v, &end);
    if (end == v) return -1;
    *out = d;
    return 0;
}

static int parse_int_param(const char *path, const char *key, int *out)
{
    const char *v = query_find(path, key);
    if (!v) return -1;
    char *end = NULL;
    long x = strtol(v, &end, 10);
    if (end == v) return -1;
    *out = (int)x;
    return 0;
}

/* ----------------------------- HTTP routing ----------------------------- */

static void serve_client(int cfd)
{
    char req[1024];
    ssize_t n = read(cfd, req, sizeof(req) - 1);
    if (n <= 0) return;
    req[n] = '\0';

    char method[8] = {0};
    char path[512] = {0};
    if (sscanf(req, "%7s %511s", method, path) != 2) {
        http_send_404(cfd);
        return;
    }

    if (strcmp(method, "GET") != 0) {
        http_send_404(cfd);
        return;
    }

    char path_only[512];
    snprintf(path_only, sizeof(path_only), "%s", path);
    char *qmark = strchr(path_only, '?');
    if (qmark) *qmark = '\0';

    if (strcmp(path_only, "/api/v1/health") == 0) {
        char json[256];
        build_health_json(json, sizeof(json));
        http_send(cfd, "application/json", json);
        return;
    }

    if (strcmp(path_only, "/api/v1/latest") == 0) {
        char json[896];
        build_latest_json(json, sizeof(json));
        http_send(cfd, "application/json", json);
        return;
    }

    if (strcmp(path_only, "/api/v1/control/state") == 0) {
        char json[1024];
        build_control_state_json(json, sizeof(json));
        http_send(cfd, "application/json", json);
        return;
    }

    // /api/v1/control/enable?enabled=0|1
    if (strcmp(path_only, "/api/v1/control/enable") == 0) {
        int en = 0;
        if (parse_int_param(path, "enabled", &en) != 0) {
            http_send_400(cfd, "missing enabled=0|1");
            return;
        }
        pthread_mutex_lock(&g_lock);
        g_ctrl.enabled = (en != 0);
        pthread_mutex_unlock(&g_lock);

        char json[128];
        snprintf(json, sizeof(json), "{\"enabled\":%s}", (en != 0) ? "true" : "false");
        http_send(cfd, "application/json", json);
        return;
    }

    // /api/v1/control/setpoint?value=6.5
    if (strcmp(path_only, "/api/v1/control/setpoint") == 0) {
        double sp = 0.0;
        if (parse_double_param(path, "value", &sp) != 0) {
            http_send_400(cfd, "missing value=<setpoint>");
            return;
        }
        if (sp < 0.0 || sp > 14.0) {
            http_send_400(cfd, "setpoint out of range 0..14");
            return;
        }
        pthread_mutex_lock(&g_lock);
        g_ctrl.setpoint = sp;
        pthread_mutex_unlock(&g_lock);

        char json[128];
        snprintf(json, sizeof(json), "{\"setpoint\":%.3f}", sp);
        http_send(cfd, "application/json", json);
        return;
    }

    // /api/v1/control/tune?kp=2.0&ki=0.5&umax=0.4
    if (strcmp(path_only, "/api/v1/control/tune") == 0) {
        double kp = 0.0, ki = 0.0, umax = 0.0;
        int have = 0;
        if (parse_double_param(path, "kp", &kp) == 0) have |= 1;
        if (parse_double_param(path, "ki", &ki) == 0) have |= 2;
        if (parse_double_param(path, "umax", &umax) == 0) have |= 4;

        if (have == 0) {
            http_send_400(cfd, "provide kp=.. and/or ki=.. and/or umax=..");
            return;
        }

        pthread_mutex_lock(&g_lock);
        if (have & 1) g_ctrl.kp = kp;
        if (have & 2) g_ctrl.ki = ki;
        if (have & 4) g_ctrl.u_max = fabs(umax);
        pthread_mutex_unlock(&g_lock);

        char json[256];
        pthread_mutex_lock(&g_lock);
        control_t c = g_ctrl;
        pthread_mutex_unlock(&g_lock);
        snprintf(json, sizeof(json), "{\"kp\":%.6f,\"ki\":%.6f,\"u_max\":%.3f}", c.kp, c.ki, c.u_max);
        http_send(cfd, "application/json", json);
        return;
    }

    // /api/v1/control/plant?pHeq=7.5&kleak=0.05&ku=0.2&noise=0.01
    if (strcmp(path_only, "/api/v1/control/plant") == 0) {
        double pheq = 0.0, kleak = 0.0, ku = 0.0, noise = 0.0;
        int have = 0;
        if (parse_double_param(path, "pHeq", &pheq) == 0) have |= 1;
        if (parse_double_param(path, "kleak", &kleak) == 0) have |= 2;
        if (parse_double_param(path, "ku", &ku) == 0) have |= 4;
        if (parse_double_param(path, "noise", &noise) == 0) have |= 8;

        if (have == 0) {
            http_send_400(cfd, "provide pHeq=.. and/or kleak=.. and/or ku=.. and/or noise=..");
            return;
        }

        pthread_mutex_lock(&g_lock);
        if (have & 1) g_ctrl.ph_eq = clamp(pheq, 0.0, 14.0);
        if (have & 2) g_ctrl.k_leak = fabs(kleak);
        if (have & 4) g_ctrl.k_u = ku;
        if (have & 8) g_ctrl.noise_sigma = fabs(noise);
        pthread_mutex_unlock(&g_lock);

        char json[256];
        pthread_mutex_lock(&g_lock);
        control_t c = g_ctrl;
        pthread_mutex_unlock(&g_lock);
        snprintf(json, sizeof(json),
                 "{\"ph_eq\":%.3f,\"k_leak\":%.6f,\"k_u\":%.6f,\"noise_sigma\":%.6f}",
                 c.ph_eq, c.k_leak, c.k_u, c.noise_sigma);
        http_send(cfd, "application/json", json);
        return;
    }

    // NEW: /api/v1/control/sim?enabled=0|1
    if (strcmp(path_only, "/api/v1/control/sim") == 0) {
        int en = 0;
        if (parse_int_param(path, "enabled", &en) != 0) {
            http_send_400(cfd, "missing enabled=0|1");
            return;
        }
        pthread_mutex_lock(&g_lock);
        g_ctrl.sim_enabled = (en != 0);
        pthread_mutex_unlock(&g_lock);

        char json[128];
        snprintf(json, sizeof(json), "{\"sim_enabled\":%s}", (en != 0) ? "true" : "false");
        http_send(cfd, "application/json", json);
        return;
    }

    // NEW: /api/v1/control/cal?enabled=0|1&slope=-5.7&offset=21.0
    if (strcmp(path_only, "/api/v1/control/cal") == 0) {
        int en = 0;
        int have_en = (parse_int_param(path, "enabled", &en) == 0);

        double slope = 0.0, offset = 0.0;
        int have_slope = (parse_double_param(path, "slope", &slope) == 0);
        int have_offset = (parse_double_param(path, "offset", &offset) == 0);

        if (!have_en && !have_slope && !have_offset) {
            http_send_400(cfd, "provide enabled=0|1 and optionally slope=..&offset=..");
            return;
        }

        pthread_mutex_lock(&g_lock);
        if (have_en) g_cal.enabled = (en != 0);
        if (have_slope) g_cal.slope = slope;
        if (have_offset) g_cal.offset = offset;
        ph_cal_t cal = g_cal;
        pthread_mutex_unlock(&g_lock);

        char json[256];
        snprintf(json, sizeof(json),
                 "{\"enabled\":%s,\"slope\":%.6f,\"offset\":%.6f}",
                 cal.enabled ? "true" : "false", cal.slope, cal.offset);
        http_send(cfd, "application/json", json);
        return;
    }

    if (strcmp(path_only, "/") == 0) {
        const char *html =
            "<!doctype html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>MCP3221 + pH Control</title>"
            "<style>"
            "body{font-family:sans-serif;margin:2rem}"
            ".grid{display:grid;grid-template-columns:1fr 1fr;gap:1rem;max-width:900px}"
            ".card{border:1px solid #ddd;border-radius:12px;padding:1rem}"
            ".big{font-size:3rem}"
            "label{display:block;margin-top:.5rem}"
            "input{font-size:1rem;padding:.3rem .4rem;width:10rem}"
            "button{font-size:1rem;padding:.4rem .6rem;margin-right:.5rem;margin-top:.5rem}"
            ".kv{display:grid;grid-template-columns:140px 1fr;row-gap:.25rem}"
            ".muted{color:#555;font-size:.9rem}"
            "</style>"
            "</head><body>"
            "<h1>MCP3221 + pH Control (PI)</h1>"
            "<div class='grid'>"
            "<div class='card'>"
            "<h2>Sensor</h2>"
            "<div class='kv'>"
            "<div>Time:</div><div><span id='ts'>-</span></div>"
            "<div>Status:</div><div><span id='st'>-</span></div>"
            "</div>"
            "<div class='big'><span id='vadc'>-</span> V</div>"
            "<div>Raw: <span id='raw'>-</span></div>"
            "<div>pH (display): <span id='ph'>-</span></div>"
            "</div>"
            "<div class='card'>"
            "<h2>Control</h2>"
            "<div class='kv'>"
            "<div>Enabled:</div><div><span id='en'>-</span></div>"
            "<div>Simulation:</div><div><span id='sim'>-</span></div>"
            "<div>Setpoint:</div><div><span id='sp'>-</span></div>"
            "<div>Measured pH:</div><div><span id='phm'>-</span></div>"
            "<div>Actuator u:</div><div><span id='u'>-</span></div>"
            "</div>"

            "<label>Setpoint <input id='sp_in' type='number' step='0.01' min='0' max='14'></label>"
            "<div>"
            "<button onclick='setEnable(1)'>Enable</button>"
            "<button onclick='setEnable(0)'>Disable</button>"
            "<button onclick='applySP()'>Apply Setpoint</button>"
            "</div>"

            "<div>"
            "<button onclick='setSim(1)'>Sim On</button>"
            "<button onclick='setSim(0)'>Sim Off</button>"
            "</div>"

            "<div class='muted' style='margin-top:.5rem'>"
            "Sim toggle: /api/v1/control/sim?enabled=0|1<br>"
            "Calibration: /api/v1/control/cal?enabled=1&slope=..&offset=.."
            "</div>"
            "</div>"
            "</div>"
            "<script>"
            "async function setEnable(v){ await fetch('/api/v1/control/enable?enabled='+v); }"
            "async function setSim(v){ await fetch('/api/v1/control/sim?enabled='+v); }"
            "async function applySP(){"
            " const v=document.getElementById('sp_in').value;"
            " if(v==='') return;"
            " await fetch('/api/v1/control/setpoint?value='+encodeURIComponent(v));"
            "}"
            "function fmt(x, d){"
            " if(x===null||x===undefined) return '-';"
            " if(typeof x==='number') return x.toFixed(d);"
            " return x;"
            "}"
            "async function tick(){"
            " const r=await fetch('/api/v1/latest',{cache:'no-store'});"
            " const j=await r.json();"
            " document.getElementById('ts').textContent=j.ts;"
            " document.getElementById('st').textContent=j.status;"
            " document.getElementById('vadc').textContent=fmt(j.vadc,6);"
            " document.getElementById('raw').textContent=(j.raw_adc ?? '-');"
            " document.getElementById('ph').textContent=(j.ph===null?'-':fmt(j.ph,2));"
            " if(j.control){"
            "  document.getElementById('en').textContent=j.control.enabled ? 'true':'false';"
            "  document.getElementById('sim').textContent=j.control.sim_enabled ? 'true':'false';"
            "  document.getElementById('sp').textContent=fmt(j.control.setpoint,2);"
            "  document.getElementById('u').textContent=fmt(j.control.u,3);"
            " }"
            " const cs=await (await fetch('/api/v1/control/state',{cache:'no-store'})).json();"
            " document.getElementById('phm').textContent=(cs.ph_meas===null?'-':fmt(cs.ph_meas,2));"
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
            "          [--db /var/lib/mcp3221/mcp3221.sqlite] [--port 8080]\n"
            "          [--ctrl-period-ms 100]\n"
            "          [--sp 6.5] [--enable 0|1] [--sim 0|1]\n"
            "          [--kp 2.0] [--ki 0.5] [--u-max 0.4]\n"
            "          [--ph0 7.5] [--ph-eq 7.5] [--k-leak 0.05] [--k-u 0.2] [--noise 0.01]\n"
            "          [--cal-enable 0|1] [--cal-slope X] [--cal-offset Y]\n",
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
        .ctrl_period_ms = 100,
    };

    srand((unsigned)now_unix_ms());

    // Defaults: tap-water-ish plant and conservative PI
    pthread_mutex_lock(&g_lock);
    memset(&g_latest, 0, sizeof(g_latest));
    g_latest.ts_unix_ms = now_unix_ms();
    g_latest.vref = cfg.vref;
    g_latest.ph = NAN;
    snprintf(g_latest.faults, sizeof(g_latest.faults), "no_data_yet");

    memset(&g_ctrl, 0, sizeof(g_ctrl));
    g_ctrl.enabled = 0;
    g_ctrl.setpoint = 6.50;

    g_ctrl.kp = 2.0;
    g_ctrl.ki = 0.5;
    g_ctrl.u_max = 0.40;

    g_ctrl.i_min = -2.0;
    g_ctrl.i_max =  2.0;

    g_ctrl.integ = 0.0;
    g_ctrl.u = 0.0;

    g_ctrl.ph = 7.50;
    g_ctrl.ph_eq = 7.50;
    g_ctrl.k_leak = 0.05;
    g_ctrl.k_u = 0.20;
    g_ctrl.noise_sigma = 0.01;

    g_ctrl.sim_enabled = 1;
    g_ctrl.ph_meas = NAN;

    memset(&g_cal, 0, sizeof(g_cal));
    g_cal.enabled = 0;   // start disabled, enable when you have calibration
    g_cal.slope = 0.0;
    g_cal.offset = 0.0;
    pthread_mutex_unlock(&g_lock);

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--bus") && i + 1 < argc) cfg.bus = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--addr") && i + 1 < argc) cfg.addr = (int)strtol(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--vref") && i + 1 < argc) cfg.vref = atof(argv[++i]);
        else if (!strcmp(argv[i], "--period-ms") && i + 1 < argc) cfg.period_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--db") && i + 1 < argc) cfg.db_path = argv[++i];
        else if (!strcmp(argv[i], "--port") && i + 1 < argc) cfg.http_port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--ctrl-period-ms") && i + 1 < argc) cfg.ctrl_period_ms = (unsigned)atoi(argv[++i]);

        else if (!strcmp(argv[i], "--sp") && i + 1 < argc) {
            double sp = atof(argv[++i]);
            pthread_mutex_lock(&g_lock);
            g_ctrl.setpoint = clamp(sp, 0.0, 14.0);
            pthread_mutex_unlock(&g_lock);
        } else if (!strcmp(argv[i], "--enable") && i + 1 < argc) {
            int en = atoi(argv[++i]);
            pthread_mutex_lock(&g_lock);
            g_ctrl.enabled = (en != 0);
            pthread_mutex_unlock(&g_lock);
        } else if (!strcmp(argv[i], "--sim") && i + 1 < argc) {
            int en = atoi(argv[++i]);
            pthread_mutex_lock(&g_lock);
            g_ctrl.sim_enabled = (en != 0);
            pthread_mutex_unlock(&g_lock);
        } else if (!strcmp(argv[i], "--kp") && i + 1 < argc) {
            double kp = atof(argv[++i]);
            pthread_mutex_lock(&g_lock);
            g_ctrl.kp = kp;
            pthread_mutex_unlock(&g_lock);
        } else if (!strcmp(argv[i], "--ki") && i + 1 < argc) {
            double ki = atof(argv[++i]);
            pthread_mutex_lock(&g_lock);
            g_ctrl.ki = ki;
            pthread_mutex_unlock(&g_lock);
        } else if (!strcmp(argv[i], "--u-max") && i + 1 < argc) {
            double umax = atof(argv[++i]);
            pthread_mutex_lock(&g_lock);
            g_ctrl.u_max = fabs(umax);
            pthread_mutex_unlock(&g_lock);
        } else if (!strcmp(argv[i], "--ph0") && i + 1 < argc) {
            double ph0 = atof(argv[++i]);
            pthread_mutex_lock(&g_lock);
            g_ctrl.ph = clamp(ph0, 0.0, 14.0);
            pthread_mutex_unlock(&g_lock);
        } else if (!strcmp(argv[i], "--ph-eq") && i + 1 < argc) {
            double pheq = atof(argv[++i]);
            pthread_mutex_lock(&g_lock);
            g_ctrl.ph_eq = clamp(pheq, 0.0, 14.0);
            pthread_mutex_unlock(&g_lock);
        } else if (!strcmp(argv[i], "--k-leak") && i + 1 < argc) {
            double kleak = atof(argv[++i]);
            pthread_mutex_lock(&g_lock);
            g_ctrl.k_leak = fabs(kleak);
            pthread_mutex_unlock(&g_lock);
        } else if (!strcmp(argv[i], "--k-u") && i + 1 < argc) {
            double ku = atof(argv[++i]);
            pthread_mutex_lock(&g_lock);
            g_ctrl.k_u = ku;
            pthread_mutex_unlock(&g_lock);
        } else if (!strcmp(argv[i], "--noise") && i + 1 < argc) {
            double ns = atof(argv[++i]);
            pthread_mutex_lock(&g_lock);
            g_ctrl.noise_sigma = fabs(ns);
            pthread_mutex_unlock(&g_lock);
        } else if (!strcmp(argv[i], "--cal-enable") && i + 1 < argc) {
            int en = atoi(argv[++i]);
            pthread_mutex_lock(&g_lock);
            g_cal.enabled = (en != 0);
            pthread_mutex_unlock(&g_lock);
        } else if (!strcmp(argv[i], "--cal-slope") && i + 1 < argc) {
            double s = atof(argv[++i]);
            pthread_mutex_lock(&g_lock);
            g_cal.slope = s;
            pthread_mutex_unlock(&g_lock);
        } else if (!strcmp(argv[i], "--cal-offset") && i + 1 < argc) {
            double o = atof(argv[++i]);
            pthread_mutex_lock(&g_lock);
            g_cal.offset = o;
            pthread_mutex_unlock(&g_lock);
        } else {
            usage(argv[0]);
            return 2;
        }
    }

    // Update latest vref after CLI parsing
    pthread_mutex_lock(&g_lock);
    g_latest.vref = cfg.vref;
    pthread_mutex_unlock(&g_lock);

    // Init DB schema once in main before threads start
    sqlite3 *db0 = NULL;
    if (db_open_only(&db0, cfg.db_path) != 0) {
        fprintf(stderr, "db_open_only failed in main for %s\n", cfg.db_path);
        return 1;
    }
    if (db_schema_init_once(db0) != 0) {
        fprintf(stderr, "db_schema_init_once failed for %s\n", cfg.db_path);
        sqlite3_close(db0);
        return 1;
    }
    sqlite3_close(db0);

    pthread_t th_samp, th_http, th_ctrl;

    if (pthread_create(&th_ctrl, NULL, control_thread, &cfg) != 0) {
        fprintf(stderr, "pthread_create control failed\n");
        return 1;
    }
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
