#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/fatal.h>
#include <zephyr/spinlock.h>
#include <zephyr/retention/bootmode.h>
#include <zephyr/settings/settings.h>   // BONDING: load saved pairing keys
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/services/cts_client.h>
#include <bluetooth/services/ancs_client.h>   // ANCS: iPhone notifications
#include <hal/nrf_gpio.h>
#include <lvgl.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>

#define DB_LED_PIN    (32 + 11)
#define DRV_EN_PIN    (32 + 2)    // haptic held OFF
#define TP_RST_PIN    (29)        // P0.29 touch reset (active low)

// Interrupt inputs (port-relative numbers for the Zephyr GPIO API)
#define TP_INT_PORT_PIN 3         // P0.03  touch interrupt (active low)
#define PB2_PORT_PIN    24        // P0.24
#define PB1_PORT_PIN    10        // P1.10
#define INT2_PORT_PIN   9         // P1.09  ACCEL_INT2
#define PB1_PIN         (32 + PB1_PORT_PIN)
#define PB2_PIN         (PB2_PORT_PIN)

#define LSM_ADDR      0x6A
#define TOUCH_ADDR    0x15

// ---- Clock settings ----
#define USE_24H          0        // 0 = 12-hour with AM/PM, 1 = 24-hour
#define RESYNC_MINUTES   10

// ---- Raise-to-wake settings ----
#define FACE_AXIS        2
#define FACE_SIGN        1
#define VIEW_ON_MG       600
#define VIEW_OFF_MG      400
#define IGNORE_ABOVE_MG  900
#define RAISE_DELTA_MG   250
#define SETTLE_MS        200
#define RAISE_WINDOW_MS  1500
#define AWAY_DIM_MS      2000
#define ON_TIMEOUT_MS    6000
#define DIM_HOLD_MS      6000

// ---- Brightness + fades ----
#define BRIGHT_FULL      0x80
#define BRIGHT_DIM       0x40
#define FADE_IN_MS       150
#define FADE_DIM_MS      300
#define FADE_OFF_MS      300

#define TOUCH_NO_AUTOSLEEP 0

// ---- Notifications (ANCS) ----
#define NOTIF_SHOW_MS    8000     // a new notification stays on screen this long
#define NOTIF_TITLE_MAX  18       // characters before "..." (28 px font)
#define NOTIF_MSG_MAX    110      // characters before "..." (about 5 wrapped lines)
#define NOTIF_WRAP_W     330      // message width in pixels
// FIX -22: the SDK's ANCS client rejects anything above its own per-field limit
#define ATTR_APP_LEN     BT_ANCS_ATTR_DATA_MAX
#define ATTR_TITLE_LEN   BT_ANCS_ATTR_DATA_MAX
#define ATTR_MSG_LEN     BT_ANCS_ATTR_DATA_MAX
#define ATTR_TIMEOUT_MS  2000     // give up on one notification's text after this
#define NQ_LEN           8        // notifications waiting for their text

// ---- WATCHDOG: armed by MCUboot (8 s); every app must feed it ----
#define WDT_IDLE_FEED_MS 2000     // screen off: wake at least this often just to feed
#define WDT_HEALTHY_MS   30000    // running this long = healthy, clear MCUboot's freeze counter
#define TEST_FREEZE      0        // 1 = freeze on purpose after boot, to test the rescue

static inline void wdt_feed(void)
{
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;   // harmless if the watchdog isn't running
}

static const struct device *disp;
static const struct device *i2c_dev;
static bool display_on = false;
static uint8_t cur_bright = 0;

// Wake-up plumbing: any interrupt (or a new notification) gives this semaphore
K_SEM_DEFINE(wake_sem, 0, 16);
static volatile bool motion_flag;
static volatile bool touch_flag;
static struct gpio_callback gpio0_cb;
static struct gpio_callback gpio1_cb;

// ---- Debug struct for nrfjprog memory dump (grep FEEDC0DE) ----
volatile struct __attribute__((packed)) {
    uint32_t marker;        // word 0
    uint32_t step;          // 1 boot, 2 display, 3 imu/touch, 4 irq, 5 loop, 6 bt, 7 bootmode, 999 crash
    uint32_t imu_who;
    int32_t  imu_err;
    int32_t  irq_err;
    int32_t  touch_err;
    int32_t  bt_err;
    int32_t  cts_err;
    uint32_t syncs;
    uint32_t raises;
    uint32_t misses;
    uint32_t ignored;
    uint32_t touches;
    uint32_t last_lat_ms;
    uint32_t fatal_reason;
    int32_t  bootmode_err;
    int32_t  settings_err;  // word 16  BONDING: result of settings_load(), 0 = keys loaded OK
    int32_t  ancs_err;      // word 17  ANCS: last error, 0 = none
    uint32_t notif_new;     // word 18  new notifications received
    uint32_t notif_old;     // word 19  pre-existing notifications sent at subscribe
    uint32_t notif_removed; // word 20
    uint32_t notif_dropped; // word 21  queue overflowed (oldest dropped)
    uint32_t ancs_timeouts; // word 22  text request never finished
    uint32_t notif_text;    // word 23  notifications whose text fully arrived
    uint32_t disc_reason;   // word 24  last Bluetooth disconnect reason
} dbg = {
    .marker = 0xFEEDC0DE, .imu_err = -99, .irq_err = -99, .touch_err = -99,
    .bt_err = -99, .fatal_reason = 0xFFFFFFFF, .bootmode_err = -99,
    .settings_err = -99,
};

// ---- Crash handler: fast-blink LED ----
// WATCHDOG: this loop doesn't feed, so after up to 8 s the watchdog reboots the watch
void k_sys_fatal_error_handler(unsigned int reason, const z_arch_esf_t *esf)
{
    ARG_UNUSED(esf);
    dbg.fatal_reason = reason;
    dbg.step = 999;
    nrf_gpio_cfg_output(DB_LED_PIN);
    while (1) {
        nrf_gpio_pin_toggle(DB_LED_PIN);
        k_busy_wait(50000);
        if (!nrf_gpio_pin_read(PB2_PIN)) {
            NRF_POWER->GPREGRET = BOOT_MODE_TYPE_BOOTLOADER;
            NVIC_SystemReset();
        }
        if (!nrf_gpio_pin_read(PB1_PIN)) {
            NVIC_SystemReset();
        }
    }
}

// =====================================================================
// Timekeeping: phone time + uptime offset (no RTC needed)
// =====================================================================

static struct k_spinlock time_lock;
static int64_t epoch_at_sync = -1;
static int64_t uptime_at_sync;

static int64_t days_from_civil(int y, unsigned m, unsigned d)
{
    y -= (m <= 2);
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

static void civil_from_days(int64_t z, int *y, unsigned *m, unsigned *d)
{
    z += 719468;
    const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = (unsigned)(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const int64_t yy = (int64_t)yoe + era * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    *d = doy - (153 * mp + 2) / 5 + 1;
    *m = mp < 10 ? mp + 3 : mp - 9;
    *y = (int)(yy + (*m <= 2));
}

static void time_received(const struct bt_cts_current_time *ct)
{
    const struct bt_cts_exact_time_256 *t = &ct->exact_time_256;
    if (t->year < 2000 || t->month < 1 || t->month > 12 || t->day < 1 || t->day > 31) {
        dbg.cts_err = -EINVAL;
        return;
    }
    int64_t ep = days_from_civil(t->year, t->month, t->day) * 86400
               + t->hours * 3600 + t->minutes * 60 + t->seconds;

    k_spinlock_key_t key = k_spin_lock(&time_lock);
    epoch_at_sync = ep;
    uptime_at_sync = k_uptime_get();
    k_spin_unlock(&time_lock, key);
    dbg.syncs++;
}

static bool time_now(int64_t *epoch, int64_t *since_sync_ms)
{
    k_spinlock_key_t key = k_spin_lock(&time_lock);
    int64_t ep = epoch_at_sync, up = uptime_at_sync;
    k_spin_unlock(&time_lock, key);
    if (ep < 0) return false;
    int64_t elapsed = k_uptime_get() - up;
    *epoch = ep + elapsed / 1000;
    *since_sync_ms = elapsed;
    return true;
}

// =====================================================================
// Bluetooth: advertise, pair, then discover CTS (time) and ANCS (notifications)
// =====================================================================

enum { BTS_OFF, BTS_ADV, BTS_CONNECTED, BTS_PAIRED, BTS_NO_CTS, BTS_PAIR_ERR, BTS_ADV_ERR };
enum { ANCS_WAIT, ANCS_SEARCH, ANCS_FOUND, ANCS_MISSING, ANCS_ERR };

// Flags 3 + CTS solicit 4 + ANCS solicit 18 = 25 of 31 bytes
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_SOLICIT16, BT_UUID_16_ENCODE(BT_UUID_CTS_VAL)),
    BT_DATA_BYTES(BT_DATA_SOLICIT128, BT_UUID_ANCS_VAL),
};
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static struct bt_cts_client cts_c;
static struct bt_ancs_client ancs_c;
static struct bt_conn *cur_conn;
static volatile bool has_cts;
static volatile bool cts_subscribed;
static volatile bool disc_started;
static volatile int bt_state = BTS_OFF;
static volatile int ancs_state = ANCS_WAIT;
static volatile int pair_err;
static int adv_fails;
static bool ancs_init_ok;

static void adv_work_fn(struct k_work *w);
static void resync_work_fn(struct k_work *w);
static void attr_work_fn(struct k_work *w);
static void done_work_fn(struct k_work *w);
static K_WORK_DELAYABLE_DEFINE(adv_work, adv_work_fn);
static K_WORK_DELAYABLE_DEFINE(resync_work, resync_work_fn);
static K_WORK_DELAYABLE_DEFINE(attr_work, attr_work_fn);
static K_WORK_DEFINE(done_work, done_work_fn);

static void adv_work_fn(struct k_work *w)
{
    int err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err == 0 || err == -EALREADY) {
        adv_fails = 0;
        bt_state = BTS_ADV;
        return;
    }
    if (++adv_fails >= 3) bt_state = BTS_ADV_ERR;
    k_work_schedule(&adv_work, K_MSEC(500));
}

// ---------------- Current Time Service ----------------

static void cts_notify_cb(struct bt_cts_client *c, struct bt_cts_current_time *ct)
{
    time_received(ct);
}

static void cts_read_cb(struct bt_cts_client *c, struct bt_cts_current_time *ct, int err)
{
    if (err) {
        dbg.cts_err = err;
        return;
    }
    time_received(ct);
}

static void cts_start(void)
{
    if (!has_cts || !cts_c.conn) return;
    if (bt_conn_get_security(cts_c.conn) < BT_SECURITY_L2) return;

    if (!cts_subscribed) {
        int err = bt_cts_subscribe_current_time(&cts_c, cts_notify_cb);
        if (err == 0 || err == -EALREADY) cts_subscribed = true;
        else dbg.cts_err = err;
    }
    int err = bt_cts_read_current_time(&cts_c, cts_read_cb);
    if (err) dbg.cts_err = err;
}

static void resync_work_fn(struct k_work *w)
{
    if (cur_conn) cts_start();
    k_work_schedule(&resync_work, K_MINUTES(RESYNC_MINUTES));
}

// ---------------- ANCS: notifications ----------------

// Buffers the ANCS client fills (one spare byte each)
static uint8_t buf_app[ATTR_APP_LEN + 1];
static uint8_t buf_title[ATTR_TITLE_LEN + 1];
static uint8_t buf_msg[ATTR_MSG_LEN + 1];

// Our copies of the notification currently being fetched
static char rx_app[ATTR_APP_LEN + 1];
static char rx_title[ATTR_TITLE_LEN + 1];
static char rx_msg[ATTR_MSG_LEN + 1];
static struct bt_ancs_evt_notif inflight;
static volatile bool req_busy;
static volatile bool rx_done;
static volatile bool ancs_ready;
static int64_t req_time;

// Queue of notifications waiting for their text (Bluetooth thread pushes, workqueue pops)
static struct bt_ancs_evt_notif nq[NQ_LEN];
static int nq_head, nq_count;
static struct k_spinlock nq_lock;

static void nq_push(const struct bt_ancs_evt_notif *n)
{
    k_spinlock_key_t key = k_spin_lock(&nq_lock);
    if (nq_count == NQ_LEN) {           // full: drop the oldest, keep the newest
        nq_head = (nq_head + 1) % NQ_LEN;
        nq_count--;
        dbg.notif_dropped++;
    }
    nq[(nq_head + nq_count) % NQ_LEN] = *n;
    nq_count++;
    k_spin_unlock(&nq_lock, key);
}

static bool nq_pop(struct bt_ancs_evt_notif *out)
{
    bool ok = false;
    k_spinlock_key_t key = k_spin_lock(&nq_lock);
    if (nq_count > 0) {
        *out = nq[nq_head];
        nq_head = (nq_head + 1) % NQ_LEN;
        nq_count--;
        ok = true;
    }
    k_spin_unlock(&nq_lock, key);
    return ok;
}

static void nq_unpop(const struct bt_ancs_evt_notif *n)
{
    k_spinlock_key_t key = k_spin_lock(&nq_lock);
    if (nq_count < NQ_LEN) {
        nq_head = (nq_head + NQ_LEN - 1) % NQ_LEN;
        nq[nq_head] = *n;
        nq_count++;
    }
    k_spin_unlock(&nq_lock, key);
}

static void nq_clear(void)
{
    k_spinlock_key_t key = k_spin_lock(&nq_lock);
    nq_head = 0;
    nq_count = 0;
    k_spin_unlock(&nq_lock, key);
}

// Finished notification handed to the main loop for display
struct shown_notif {
    uint32_t uid;
    int cat;
    char app[24];
    char title[40];
    char msg[NOTIF_MSG_MAX + 8];
};
static struct shown_notif pub;
static volatile uint32_t pub_seq;
static struct k_spinlock pub_lock;
static volatile uint32_t removed_uid;
static volatile bool removed_flag;

// UTF-8 to plain ASCII the Montserrat fonts can draw
static void to_ascii(char *dst, size_t n, const char *src)
{
    size_t o = 0;
    const unsigned char *s = (const unsigned char *)src;

    while (*s && o + 4 < n) {
        if (*s < 0x80) {
            char c = (char)*s++;
            dst[o++] = (c == '\n' || c == '\r' || c == '\t') ? ' ' : c;
            continue;
        }

        uint32_t cp;
        int len;
        if ((*s & 0xE0) == 0xC0)      { cp = *s & 0x1F; len = 2; }
        else if ((*s & 0xF0) == 0xE0) { cp = *s & 0x0F; len = 3; }
        else if ((*s & 0xF8) == 0xF0) { cp = *s & 0x07; len = 4; }
        else { s++; continue; }

        int i;
        for (i = 1; i < len; i++) {
            if ((s[i] & 0xC0) != 0x80) break;
            cp = (cp << 6) | (s[i] & 0x3F);
        }
        if (i < len) { s++; continue; }   // broken or cut-off character
        s += len;

        const char *rep = "?";
        switch (cp) {
        case 0x2018: case 0x2019: rep = "'"; break;
        case 0x201C: case 0x201D: rep = "\""; break;
        case 0x2026: rep = "..."; break;
        case 0x2013: case 0x2014: rep = "-"; break;
        case 0x00A0: rep = " "; break;
        // invisible marks iOS puts around phone numbers and in emoji
        case 0x200D: case 0x200E: case 0x200F: case 0xFE0F:
        case 0x202A: case 0x202B: case 0x202C: case 0x202D: case 0x202E:
        case 0x2066: case 0x2067: case 0x2068: case 0x2069:
            rep = ""; break;
        default:
            if (cp >= 0x1F3FB && cp <= 0x1F3FF) rep = "";   // skin tone modifiers
            break;
        }
        while (*rep && o + 1 < n) dst[o++] = *rep++;
    }
    dst[o] = '\0';
}

static void clip(char *s, size_t max)
{
    if (max > 3 && strlen(s) > max) strcpy(&s[max - 3], "...");
}

static void app_name(char *dst, size_t n, const char *bundle, int cat)
{
    if (cat == BT_ANCS_CATEGORY_ID_INCOMING_CALL) { snprintf(dst, n, "Incoming call"); return; }
    if (cat == BT_ANCS_CATEGORY_ID_MISSED_CALL)   { snprintf(dst, n, "Missed call"); return; }

    static const struct { const char *id; const char *name; } map[] = {
        { "com.apple.MobileSMS",       "Messages" },
        { "com.apple.mobilephone",     "Phone" },
        { "com.apple.mobilemail",      "Mail" },
        { "com.apple.reminders",       "Reminders" },
        { "com.apple.mobilecal",       "Calendar" },
        { "com.apple.mobiletimer",     "Clock" },
        { "com.apple.facetime",        "FaceTime" },
        { "net.whatsapp.WhatsApp",     "WhatsApp" },
        { "com.burbn.instagram",       "Instagram" },
        { "com.toyopagroup.picaboo",   "Snapchat" },
        { "com.hammerandchisel.discord", "Discord" },
        { "com.google.Gmail",          "Gmail" },
        { "com.spotify.client",        "Spotify" },
    };
    for (size_t i = 0; i < ARRAY_SIZE(map); i++) {
        if (strcmp(bundle, map[i].id) == 0) {
            snprintf(dst, n, "%s", map[i].name);
            return;
        }
    }
    const char *dot = strrchr(bundle, '.');
    snprintf(dst, n, "%s", (dot && dot[1]) ? dot + 1 : bundle);
    if (!dst[0]) snprintf(dst, n, "Notification");
}

static void copy_attr(char *dst, size_t n, const struct bt_ancs_attr *a)
{
    size_t len = MIN((size_t)a->attr_len, n - 1);
    if (len && a->attr_data) memcpy(dst, a->attr_data, len);
    dst[len] = '\0';
}

// Runs in the Bluetooth thread: one call per attribute (app, title, message)
static void ancs_ds_cb(struct bt_ancs_client *c, const struct bt_ancs_attr_response *rsp)
{
    if (rsp->command_id != BT_ANCS_COMMAND_ID_GET_NOTIF_ATTRIBUTES) return;
    if (!req_busy || rx_done) return;

    const struct bt_ancs_attr *a = &rsp->attr;
    switch (a->attr_id) {
    case BT_ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER:
        copy_attr(rx_app, sizeof(rx_app), a);
        break;
    case BT_ANCS_NOTIF_ATTR_ID_TITLE:
        copy_attr(rx_title, sizeof(rx_title), a);
        break;
    case BT_ANCS_NOTIF_ATTR_ID_MESSAGE:   // requested last, so this completes it
        copy_attr(rx_msg, sizeof(rx_msg), a);
        rx_done = true;
        k_work_submit(&done_work);
        break;
    default:
        break;
    }
}

// Runs in the Bluetooth thread: added / modified / removed events
static void ancs_ns_cb(struct bt_ancs_client *c, int err, const struct bt_ancs_evt_notif *n)
{
    if (err) {
        dbg.ancs_err = err;
        return;
    }
    switch (n->evt_id) {
    case BT_ANCS_EVENT_ID_NOTIFICATION_ADDED:
        if (n->evt_flags.pre_existing) dbg.notif_old++;
        else dbg.notif_new++;
        nq_push(n);
        k_work_reschedule(&attr_work, K_NO_WAIT);
        break;
    case BT_ANCS_EVENT_ID_NOTIFICATION_REMOVED:
        dbg.notif_removed++;
        removed_uid = n->notif_uid;
        removed_flag = true;
        break;
    default:
        break;
    }
}

static void attr_write_cb(struct bt_ancs_client *c, uint8_t err)
{
    if (err) {
        dbg.ancs_err = err;
        req_busy = false;
        k_work_reschedule(&attr_work, K_MSEC(50));
    }
}

// Workqueue: ask the iPhone for the next queued notification's text, one at a time
static void attr_work_fn(struct k_work *w)
{
    if (!ancs_ready) return;

    if (req_busy) {
        int64_t elapsed = k_uptime_get() - req_time;
        if (elapsed < ATTR_TIMEOUT_MS) {
            k_work_reschedule(&attr_work, K_MSEC(ATTR_TIMEOUT_MS - elapsed));
            return;
        }
        dbg.ancs_timeouts++;
        req_busy = false;
    }

    struct bt_ancs_evt_notif n;
    if (!nq_pop(&n)) return;

    rx_app[0] = rx_title[0] = rx_msg[0] = '\0';
    rx_done = false;
    inflight = n;
    req_time = k_uptime_get();
    req_busy = true;

    int err = bt_ancs_request_attrs(&ancs_c, &n, attr_write_cb);
    if (err) {
        req_busy = false;
        if (err == -EBUSY) {
            nq_unpop(&n);
            k_work_reschedule(&attr_work, K_MSEC(100));
            return;
        }
        dbg.ancs_err = err;
        k_work_reschedule(&attr_work, K_MSEC(50));
        return;
    }
    k_work_reschedule(&attr_work, K_MSEC(ATTR_TIMEOUT_MS));   // timeout check
}

// Workqueue: all text for one notification arrived
static void done_work_fn(struct k_work *w)
{
    static char app[ATTR_APP_LEN + 8];
    static char title[ATTR_TITLE_LEN + 8];
    static char msg[ATTR_MSG_LEN + 8];
    static struct shown_notif sn;

    dbg.notif_text++;

    if (!inflight.evt_flags.pre_existing) {
        to_ascii(app, sizeof(app), rx_app);
        to_ascii(title, sizeof(title), rx_title);
        to_ascii(msg, sizeof(msg), rx_msg);

        sn.uid = inflight.notif_uid;
        sn.cat = inflight.category_id;
        app_name(sn.app, sizeof(sn.app), app, sn.cat);
        snprintf(sn.title, sizeof(sn.title), "%s", title);
        clip(sn.title, NOTIF_TITLE_MAX);
        snprintf(sn.msg, sizeof(sn.msg), "%s", msg);
        if (strlen(rx_msg) >= ATTR_MSG_LEN) {
            // The iPhone cut the message at the SDK limit: show that it continues
            size_t l = strlen(sn.msg);
            if (l + 4 <= sizeof(sn.msg)) strcpy(&sn.msg[l], "...");
        }
        clip(sn.msg, NOTIF_MSG_MAX);

        k_spinlock_key_t key = k_spin_lock(&pub_lock);
        pub = sn;
        pub_seq++;
        k_spin_unlock(&pub_lock, key);
        k_sem_give(&wake_sem);
    }

    req_busy = false;
    k_work_reschedule(&attr_work, K_NO_WAIT);
}

static void ancs_enable(void)
{
    int err = bt_ancs_subscribe_data_source(&ancs_c, ancs_ds_cb);
    if (err && err != -EALREADY) {
        dbg.ancs_err = err;
        ancs_state = ANCS_ERR;
        return;
    }
    err = bt_ancs_subscribe_notification_source(&ancs_c, ancs_ns_cb);
    if (err && err != -EALREADY) {
        dbg.ancs_err = err;
        ancs_state = ANCS_ERR;
        return;
    }
    ancs_ready = true;
    ancs_state = ANCS_FOUND;
}

static int ancs_init(void)
{
    int err = bt_ancs_client_init(&ancs_c);
    if (err) return err;
    err = bt_ancs_register_attr(&ancs_c, BT_ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER,
                                buf_app, ATTR_APP_LEN);
    if (err) return err;
    err = bt_ancs_register_attr(&ancs_c, BT_ANCS_NOTIF_ATTR_ID_TITLE,
                                buf_title, ATTR_TITLE_LEN);
    if (err) return err;
    return bt_ancs_register_attr(&ancs_c, BT_ANCS_NOTIF_ATTR_ID_MESSAGE,
                                 buf_msg, ATTR_MSG_LEN);
}

// ---------------- Discovery: CTS first, then ANCS ----------------

#define DISC_CTS  ((void *)(uintptr_t)1)
#define DISC_ANCS ((void *)(uintptr_t)2)

static void start_discovery(struct bt_conn *conn, void *which);

static void discover_completed_cb(struct bt_gatt_dm *dm, void *ctx)
{
    struct bt_conn *conn = bt_gatt_dm_conn_get(dm);
    int err;

    if (ctx == DISC_ANCS) {
        err = bt_ancs_handles_assign(dm, &ancs_c);
        bt_gatt_dm_data_release(dm);
        if (err) {
            dbg.ancs_err = err;
            ancs_state = ANCS_ERR;
            return;
        }
        ancs_enable();
        return;
    }

    err = bt_cts_handles_assign(dm, &cts_c);
    bt_gatt_dm_data_release(dm);
    if (err) {
        dbg.cts_err = err;
        bt_state = BTS_NO_CTS;
    } else {
        has_cts = true;
        cts_start();
    }
    start_discovery(conn, DISC_ANCS);
}

static void discover_not_found_cb(struct bt_conn *conn, void *ctx)
{
    if (ctx == DISC_ANCS) {
        ancs_state = ANCS_MISSING;
        return;
    }
    bt_state = BTS_NO_CTS;
    start_discovery(conn, DISC_ANCS);
}

static void discover_error_cb(struct bt_conn *conn, int err, void *ctx)
{
    if (ctx == DISC_ANCS) {
        dbg.ancs_err = err;
        ancs_state = ANCS_ERR;
        return;
    }
    dbg.cts_err = err;
    bt_state = BTS_NO_CTS;
    start_discovery(conn, DISC_ANCS);
}

static const struct bt_gatt_dm_cb discover_cb = {
    .completed = discover_completed_cb,
    .service_not_found = discover_not_found_cb,
    .error_found = discover_error_cb,
};

static void start_discovery(struct bt_conn *conn, void *which)
{
    if (which == DISC_ANCS) {
        if (!ancs_init_ok) return;
        ancs_state = ANCS_SEARCH;
    }
    const struct bt_uuid *uuid = (which == DISC_CTS) ? BT_UUID_CTS : BT_UUID_ANCS;
    int err = bt_gatt_dm_start(conn, uuid, &discover_cb, which);
    if (err) {
        if (which == DISC_ANCS) {
            dbg.ancs_err = err;
            ancs_state = ANCS_ERR;
        } else {
            dbg.cts_err = err;
            bt_state = BTS_NO_CTS;
            start_discovery(conn, DISC_ANCS);
        }
    }
}

// ---------------- Connection callbacks ----------------

static void ancs_reset(void)
{
    ancs_ready = false;
    req_busy = false;
    nq_clear();
    k_work_cancel_delayable(&attr_work);
    ancs_state = ancs_init_ok ? ANCS_WAIT : ANCS_ERR;
}

static void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        k_work_schedule(&adv_work, K_MSEC(200));
        return;
    }
    cur_conn = bt_conn_ref(conn);
    bt_state = BTS_CONNECTED;
    has_cts = false;
    cts_subscribed = false;
    disc_started = false;
    ancs_reset();

    // Encrypt first: ANCS only appears on an encrypted link.
    // First time: iPhone pairing pop-up. Once bonded: saved keys, no pop-up.
    int e = bt_conn_set_security(conn, BT_SECURITY_L2);
    if (e) dbg.cts_err = e;
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    if (cur_conn) {
        bt_conn_unref(cur_conn);
        cur_conn = NULL;
    }
    has_cts = false;
    cts_subscribed = false;
    disc_started = false;
    ancs_reset();
    dbg.disc_reason = reason;
    if (bt_state != BTS_PAIR_ERR) bt_state = BTS_ADV;
    k_work_schedule(&adv_work, K_MSEC(200));
}

static void on_security_changed(struct bt_conn *conn, bt_security_t level,
                                enum bt_security_err err)
{
    if (err) {
        pair_err = err;
        bt_state = BTS_PAIR_ERR;
        return;
    }
    bt_state = BTS_PAIRED;
    if (!disc_started) {
        disc_started = true;
        start_discovery(conn, DISC_CTS);
    } else {
        cts_start();
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = on_connected,
    .disconnected = on_disconnected,
    .security_changed = on_security_changed,
};

// =====================================================================
// Display: brightness, sleep/wake, LVGL watch face
// =====================================================================

static void set_bright(uint8_t b)
{
    display_set_brightness(disp, b);
    cur_bright = b;
}

static void fade_to(uint8_t target, int ms)
{
    int steps = ms / 10;
    if (steps < 1) steps = 1;
    int start = cur_bright;
    for (int i = 1; i <= steps; i++) {
        set_bright((uint8_t)(start + (target - start) * i / steps));
        k_msleep(10);
    }
}

// CO5300 rule: every redraw area must start on even x/y and be even in size
static void even_rounder(lv_disp_drv_t *drv, lv_area_t *a)
{
    a->x1 &= ~1;
    a->y1 &= ~1;
    a->x2 |= 1;
    a->y2 |= 1;
}

// Text labels that only redraw when their text or color changes
struct lbl {
    lv_obj_t *obj;
    char text[40];
    uint32_t color;
};

static struct lbl L_BT, L_ANCS, L_TIME, L_SEC, L_DATE, L_SYNC, L_NDBG, L_WAKE, L_STAT;

#define C_WHITE 0xFFFFFF
#define C_GRAY  0x8A8A8A
#define C_RED   0xFF4D4D

static void make_label(struct lbl *l, const lv_font_t *font, lv_coord_t y_ofs)
{
    l->obj = lv_label_create(lv_scr_act());
    lv_obj_set_style_text_font(l->obj, font, 0);
    lv_obj_set_style_text_color(l->obj, lv_color_hex(C_WHITE), 0);
    lv_label_set_text(l->obj, "");
    lv_obj_align(l->obj, LV_ALIGN_CENTER, 0, y_ofs);
    l->text[0] = '\0';
    l->color = C_WHITE;
}

static void set_lbl(struct lbl *l, uint32_t color, const char *fmt, ...)
{
    char buf[40];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (strcmp(buf, l->text) == 0 && color == l->color) return;
    strcpy(l->text, buf);
    if (color != l->color) {
        lv_obj_set_style_text_color(l->obj, lv_color_hex(color), 0);
        l->color = color;
    }
    lv_label_set_text(l->obj, buf);
}

// ---- Notification overlay: full-screen black card above the watch face ----
static lv_obj_t *n_box, *n_app, *n_title, *n_msg;
static bool notif_visible;
static int64_t notif_shown_at;
static uint32_t notif_shown_uid;
static uint32_t notif_seen_seq;

static lv_obj_t *notif_label(const lv_font_t *font, uint32_t color, lv_coord_t y)
{
    lv_obj_t *o = lv_label_create(n_box);
    lv_obj_set_style_text_font(o, font, 0);
    lv_obj_set_style_text_color(o, lv_color_hex(color), 0);
    lv_obj_set_style_text_align(o, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(o, "");
    lv_obj_align(o, LV_ALIGN_CENTER, 0, y);
    return o;
}

static void build_notif_ui(void)
{
    n_box = lv_obj_create(lv_scr_act());
    lv_obj_set_size(n_box, LV_PCT(100), LV_PCT(100));
    lv_obj_align(n_box, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(n_box, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(n_box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(n_box, 0, 0);
    lv_obj_set_style_radius(n_box, 0, 0);
    lv_obj_set_style_pad_all(n_box, 0, 0);
    lv_obj_clear_flag(n_box, LV_OBJ_FLAG_SCROLLABLE);

    n_app   = notif_label(&lv_font_montserrat_20, C_GRAY,  -130);
    n_title = notif_label(&lv_font_montserrat_28, C_WHITE,  -90);
    n_msg   = notif_label(&lv_font_montserrat_20, C_WHITE,   20);
    lv_label_set_long_mode(n_msg, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(n_msg, NOTIF_WRAP_W);

    lv_obj_add_flag(n_box, LV_OBJ_FLAG_HIDDEN);
}

static bool notif_waiting(void)
{
    return pub_seq != notif_seen_seq;
}

static bool notif_take(struct shown_notif *out)
{
    bool fresh = false;
    k_spinlock_key_t key = k_spin_lock(&pub_lock);
    if (pub_seq != notif_seen_seq) {
        *out = pub;
        notif_seen_seq = pub_seq;
        fresh = true;
    }
    k_spin_unlock(&pub_lock, key);
    return fresh;
}

static void notif_show(const struct shown_notif *n)
{
    lv_label_set_text(n_app, n->app);
    lv_label_set_text(n_title, n->title[0] ? n->title : " ");
    lv_label_set_text(n_msg, n->msg[0] ? n->msg : " ");
    lv_obj_clear_flag(n_box, LV_OBJ_FLAG_HIDDEN);
    notif_visible = true;
    notif_shown_at = k_uptime_get();
    notif_shown_uid = n->uid;
}

static void notif_hide(void)
{
    if (!notif_visible) return;
    lv_obj_add_flag(n_box, LV_OBJ_FLAG_HIDDEN);
    notif_visible = false;
}

static void build_ui(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    make_label(&L_BT,   &lv_font_montserrat_20, -160);
    make_label(&L_ANCS, &lv_font_montserrat_14, -130);
    make_label(&L_TIME, &lv_font_montserrat_48,  -60);
    make_label(&L_SEC,  &lv_font_montserrat_20,  -15);
    make_label(&L_DATE, &lv_font_montserrat_28,   30);
    make_label(&L_SYNC, &lv_font_montserrat_20,   85);
    make_label(&L_NDBG, &lv_font_montserrat_14,  107);
    make_label(&L_WAKE, &lv_font_montserrat_14,  125);
    make_label(&L_STAT, &lv_font_montserrat_14,  150);

    build_notif_ui();   // created last so it draws on top
}

static void display_sleep(void)
{
    fade_to(0, FADE_OFF_MS);
    notif_hide();
    // Repaint the whole panel while it's dark: clears any corrupted pixels unseen
    lv_obj_invalidate(lv_scr_act());
    lv_task_handler();
    display_blanking_on(disp);
    display_on = false;
}

static void display_wake(void)
{
    set_bright(0);
    display_blanking_off(disp);
    display_on = true;
}

// =====================================================================
// IMU, touch, interrupts, buttons
// =====================================================================

static int i2c_wr(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    int ret = 0;
    for (int i = 0; i < 5; i++) {
        ret = i2c_write(i2c_dev, b, 2, addr);
        if (ret == 0) return 0;
        k_msleep(10);
    }
    return ret;
}

static int i2c_rd(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n)
{
    return i2c_write_read(i2c_dev, addr, &reg, 1, buf, n);
}

static int imu_setup(void)
{
    uint8_t who = 0;
    int e = i2c_rd(LSM_ADDR, 0x0F, &who, 1);
    dbg.imu_who = who;
    if (e) return e;
    if (who != 0x6C) return -ENODEV;

    if ((e = i2c_wr(LSM_ADDR, 0x12, 0x01))) return e;  // software reset
    k_msleep(20);

    if ((e = i2c_wr(LSM_ADDR, 0x10, 0x30))) return e;  // 52 Hz, +/-4 g
    if ((e = i2c_wr(LSM_ADDR, 0x58, 0x80))) return e;  // enable motion interrupts
    if ((e = i2c_wr(LSM_ADDR, 0x5B, 0x02))) return e;  // wake-up threshold 0.125 g
    if ((e = i2c_wr(LSM_ADDR, 0x5C, 0x00))) return e;  // 1 sample
    if ((e = i2c_wr(LSM_ADDR, 0x5F, 0x20))) return e;  // wake-up -> INT2
    return 0;
}

static int read_axes(int mg[3])
{
    uint8_t raw[6];
    int e = i2c_rd(LSM_ADDR, 0x28, raw, 6);
    if (e) return e;
    for (int i = 0; i < 3; i++) {
        int16_t v = (int16_t)((raw[i * 2 + 1] << 8) | raw[i * 2]);
        mg[i] = (int)v * 122 / 1000;
    }
    return 0;
}

static int face_mg(const int mg[3])
{
    return FACE_SIGN * mg[FACE_AXIS];
}

static int touch_setup(void)
{
    nrf_gpio_cfg_output(TP_RST_PIN);
    nrf_gpio_pin_clear(TP_RST_PIN);
    k_msleep(10);
    nrf_gpio_pin_set(TP_RST_PIN);
    k_msleep(150);

#if TOUCH_NO_AUTOSLEEP
    const struct device *tp_i2c = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    if (!device_is_ready(tp_i2c)) return -ENODEV;
    uint8_t cmd[2] = { 0xFE, 0x01 };
    return i2c_write(tp_i2c, cmd, 2, TOUCH_ADDR);
#else
    return 0;
#endif
}

static void gpio0_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (pins & BIT(TP_INT_PORT_PIN)) touch_flag = true;
    k_sem_give(&wake_sem);
}

static void gpio1_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (pins & BIT(INT2_PORT_PIN)) motion_flag = true;
    k_sem_give(&wake_sem);
}

static int irq_setup(void)
{
    const struct device *g0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    const struct device *g1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));
    if (!device_is_ready(g0) || !device_is_ready(g1)) return -ENODEV;

    int e;
    if ((e = gpio_pin_configure(g0, TP_INT_PORT_PIN, GPIO_INPUT | GPIO_PULL_UP))) return e;
    if ((e = gpio_pin_configure(g0, PB2_PORT_PIN, GPIO_INPUT | GPIO_PULL_UP))) return e;
    if ((e = gpio_pin_configure(g1, PB1_PORT_PIN, GPIO_INPUT | GPIO_PULL_UP))) return e;
    if ((e = gpio_pin_configure(g1, INT2_PORT_PIN, GPIO_INPUT))) return e;

    gpio_init_callback(&gpio0_cb, gpio0_handler, BIT(TP_INT_PORT_PIN) | BIT(PB2_PORT_PIN));
    gpio_init_callback(&gpio1_cb, gpio1_handler, BIT(PB1_PORT_PIN) | BIT(INT2_PORT_PIN));
    if ((e = gpio_add_callback(g0, &gpio0_cb))) return e;
    if ((e = gpio_add_callback(g1, &gpio1_cb))) return e;

    if ((e = gpio_pin_interrupt_configure(g0, TP_INT_PORT_PIN, GPIO_INT_EDGE_FALLING))) return e;
    if ((e = gpio_pin_interrupt_configure(g0, PB2_PORT_PIN, GPIO_INT_EDGE_FALLING))) return e;
    if ((e = gpio_pin_interrupt_configure(g1, PB1_PORT_PIN, GPIO_INT_EDGE_FALLING))) return e;
    if ((e = gpio_pin_interrupt_configure(g1, INT2_PORT_PIN, GPIO_INT_EDGE_RISING))) return e;
    return 0;
}

static void handle_buttons(void)
{
    if (!nrf_gpio_pin_read(PB1_PIN)) {
        k_msleep(50);
        if (!nrf_gpio_pin_read(PB1_PIN)) {
            if (display_on) {
                set_lbl(&L_STAT, C_WHITE, "Resetting");
                lv_task_handler();
            }
            k_msleep(200);
            sys_reboot(SYS_REBOOT_COLD);
        }
    }

    if (!nrf_gpio_pin_read(PB2_PIN)) {
        int64_t start = k_uptime_get();
        nrf_gpio_pin_set(DB_LED_PIN);
        while (!nrf_gpio_pin_read(PB2_PIN)) {
            wdt_feed();   // WATCHDOG: the 3 s countdown counts as alive
            int64_t held = k_uptime_get() - start;
            if (held >= 3000) {
                dbg.step = 7;
                int bm = bootmode_set(BOOT_MODE_TYPE_BOOTLOADER);
                dbg.bootmode_err = bm;
                if (bm == 0) {
                    if (display_on) {
                        set_lbl(&L_STAT, C_WHITE, "USB mode");
                        lv_task_handler();
                    }
                    k_msleep(200);
                    sys_reboot(SYS_REBOOT_WARM);
                }
                if (display_on) {
                    set_lbl(&L_STAT, C_RED, "Boot mode error %d", bm);
                    lv_task_handler();
                }
                k_msleep(2000);
                break;
            }
            if (display_on) {
                set_lbl(&L_STAT, C_WHITE, "USB mode in %d", (int)(3 - held / 1000));
                lv_task_handler();
            }
            k_msleep(20);
        }
        nrf_gpio_pin_clear(DB_LED_PIN);
    }
}

// =====================================================================
// Watch face content
// =====================================================================

static const char *const WDAY[] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
static const char *const MON[]  = { "Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };

static void update_ancs_line(void)
{
    switch (ancs_state) {
    case ANCS_SEARCH:
        set_lbl(&L_ANCS, C_GRAY, "Looking for notifications");
        break;
    case ANCS_FOUND:
        if (dbg.notif_new || dbg.notif_old) {
            set_lbl(&L_ANCS, C_GRAY, "Notif  %u new  %u old",
                    (unsigned)dbg.notif_new, (unsigned)dbg.notif_old);
        } else {
            set_lbl(&L_ANCS, C_GRAY, "ANCS found");
        }
        break;
    case ANCS_MISSING:
        set_lbl(&L_ANCS, C_RED, "ANCS not found");
        break;
    case ANCS_ERR:
        set_lbl(&L_ANCS, C_RED, "ANCS err %d", (int)dbg.ancs_err);
        break;
    default:
        set_lbl(&L_ANCS, C_GRAY, " ");
        break;
    }

    // Counters line: text received, removed, timeouts, dropped, last error
    if (ancs_state == ANCS_FOUND) {
        set_lbl(&L_NDBG, dbg.ancs_err ? C_RED : C_GRAY,
                "Txt %u  Rm %u  TO %u  Dp %u  E%d",
                (unsigned)dbg.notif_text, (unsigned)dbg.notif_removed,
                (unsigned)dbg.ancs_timeouts, (unsigned)dbg.notif_dropped,
                (int)dbg.ancs_err);
    } else {
        set_lbl(&L_NDBG, C_GRAY, " ");
    }
}

static void update_bt_state(void)
{
    switch (bt_state) {
    case BTS_ADV:
        // BONDING: flag it if the saved keys couldn't be loaded
        if (dbg.settings_err != 0) {
            set_lbl(&L_BT, C_RED, "Bond storage error %d", (int)dbg.settings_err);
        } else if (dbg.disc_reason != 0) {
            set_lbl(&L_BT, C_GRAY, "Advertising  last 0x%02X", (unsigned)dbg.disc_reason);
        } else {
            set_lbl(&L_BT, C_GRAY, "Advertising");
        }
        break;
    case BTS_CONNECTED: set_lbl(&L_BT, C_GRAY, "Connected"); break;
    case BTS_PAIRED:    set_lbl(&L_BT, C_GRAY, "Paired"); break;
    case BTS_NO_CTS:    set_lbl(&L_BT, C_RED,  "No time service"); break;
    case BTS_PAIR_ERR:  set_lbl(&L_BT, C_RED,  "Pairing error %d", pair_err); break;
    case BTS_ADV_ERR:   set_lbl(&L_BT, C_RED,  "Advertising error"); break;
    default:            set_lbl(&L_BT, C_RED,  "Bluetooth error %d", (int)dbg.bt_err); break;
    }
    update_ancs_line();
}

static void update_clock(void)
{
    int64_t ep, since_ms;
    if (!time_now(&ep, &since_ms)) {
        set_lbl(&L_TIME, C_WHITE, "--:--");
        set_lbl(&L_SEC, C_GRAY, " ");
        set_lbl(&L_DATE, C_GRAY, "No time yet");
        if (bt_state == BTS_PAIR_ERR) {
            set_lbl(&L_SYNC, C_RED, "Forget Watch on iPhone");
        } else {
            set_lbl(&L_SYNC, C_GRAY, "Connect your phone");
        }
        return;
    }

    int64_t days = ep / 86400;
    int sod = (int)(ep - days * 86400);
    int h = sod / 3600, m = (sod % 3600) / 60, s = sod % 60;

    int y; unsigned mon, d;
    civil_from_days(days, &y, &mon, &d);
    int wd = (int)((days + 4) % 7);   // 1970-01-01 was a Thursday

#if USE_24H
    set_lbl(&L_TIME, C_WHITE, "%02d:%02d", h, m);
    set_lbl(&L_SEC, C_GRAY, ":%02d", s);
#else
    int h12 = h % 12;
    if (h12 == 0) h12 = 12;
    set_lbl(&L_TIME, C_WHITE, "%d:%02d", h12, m);
    set_lbl(&L_SEC, C_GRAY, ":%02d %s", s, h < 12 ? "AM" : "PM");
#endif
    set_lbl(&L_DATE, C_WHITE, "%s %s %u", WDAY[wd], MON[mon - 1], d);

    unsigned mins = (unsigned)(since_ms / 60000);
    if (mins < 1)        set_lbl(&L_SYNC, C_GRAY, "Synced just now");
    else if (mins < 60)  set_lbl(&L_SYNC, C_GRAY, "Synced %u min ago", mins);
    else                 set_lbl(&L_SYNC, C_GRAY, "Synced %u h ago", mins / 60);
}

static void update_wake(const char *reason)
{
    if (strcmp(reason, "Raise") == 0) {
        set_lbl(&L_WAKE, C_GRAY, "Raise  %u ms", (unsigned)dbg.last_lat_ms);
    } else {
        set_lbl(&L_WAKE, C_GRAY, "%s", reason);
    }
}

static void update_stats(void)
{
    set_lbl(&L_STAT, C_GRAY, "R%u  M%u  I%u  T%u",
            (unsigned)dbg.raises, (unsigned)dbg.misses,
            (unsigned)dbg.ignored, (unsigned)dbg.touches);
}

// =====================================================================
// Main: raise / touch / notification to wake around the watch face
// =====================================================================

enum state { ST_OFF, ST_CHECKING, ST_ON, ST_DIM };

int main(void)
{
    dbg.step = 1;
    wdt_feed();

    uint32_t rr = NRF_POWER->RESETREAS;
    NRF_POWER->RESETREAS = rr;

    // WATCHDOG: MCUboot's freeze counter is non-zero if the last reset was a freeze
    const char *boot_reason = (NRF_POWER->GPREGRET2 > 0) ? "Boot after freeze" : "Boot";
    bool freeze_count_cleared = false;

    nrf_gpio_cfg_output(DRV_EN_PIN);
    nrf_gpio_pin_clear(DRV_EN_PIN);
    nrf_gpio_cfg_output(DB_LED_PIN);
    nrf_gpio_pin_set(DB_LED_PIN);

    // ---- Display (powered up by the co5300 driver before main) ----
    dbg.step = 2;
    disp = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(disp)) {
        while (1) { nrf_gpio_pin_toggle(DB_LED_PIN); k_msleep(100); }   // no feed: watchdog rescues
    }
    lv_disp_get_default()->driver->rounder_cb = even_rounder;
    display_on = true;
    build_ui();

    // ---- IMU, touch, interrupts ----
    dbg.step = 3;
    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    dbg.imu_err = device_is_ready(i2c_dev) ? imu_setup() : -ENODEV;
    dbg.touch_err = touch_setup();

    dbg.step = 4;
    dbg.irq_err = irq_setup();

    if (dbg.imu_err || dbg.irq_err || dbg.touch_err) {
        set_lbl(&L_BT,   C_RED, "IMU %d", (int)dbg.imu_err);
        set_lbl(&L_DATE, C_RED, "IRQ %d", (int)dbg.irq_err);
        set_lbl(&L_SYNC, C_RED, "Touch %d", (int)dbg.touch_err);
        set_lbl(&L_WAKE, C_RED, "WHO %02X", (unsigned)dbg.imu_who);
        lv_task_handler();
        set_bright(BRIGHT_FULL);
        while (1) {
            wdt_feed();   // WATCHDOG: error screen is alive, keep it up
            handle_buttons();
            lv_task_handler();
            k_msleep(50);
        }
    }

    // ---- Bluetooth ----
    dbg.step = 6;
    wdt_feed();
    int err = bt_cts_client_init(&cts_c);

    // ANCS failing to start must not take down time sync, so it's tracked separately
    dbg.ancs_err = ancs_init();
    ancs_init_ok = (dbg.ancs_err == 0);
    ancs_state = ancs_init_ok ? ANCS_WAIT : ANCS_ERR;

    if (err == 0) err = bt_enable(NULL);
    dbg.bt_err = err;
    if (err == 0) {
        // BONDING: restore saved pairing keys BEFORE advertising,
        // so a reconnecting iPhone finds the watch already knows it
        dbg.settings_err = settings_load();

        k_work_schedule(&adv_work, K_NO_WAIT);
        k_work_schedule(&resync_work, K_MINUTES(RESYNC_MINUTES));
    }
    wdt_feed();

    // First frame, then fade in
    update_bt_state();
    update_clock();
    update_wake(boot_reason);
    update_stats();
    lv_task_handler();
    fade_to(BRIGHT_FULL, FADE_IN_MS);
    nrf_gpio_pin_clear(DB_LED_PIN);

#if TEST_FREEZE
    // WATCHDOG test: hang on purpose. Expect two reboots, then USB mode (slow LED blink).
    set_lbl(&L_STAT, C_RED, "Freezing (test)");
    lv_task_handler();
    while (1) { }
#endif

    enum state st = ST_ON;
    int64_t t_motion = 0, settle_since = -1, away_since = -1, dim_start = 0;
    int64_t t_on = k_uptime_get();
    int raise_target = VIEW_ON_MG;
    const char *wake_reason = boot_reason;
    const char *dim_reason = "--";
    int64_t last_read = 0;
    int mg[3] = {0, 0, 0};
    struct shown_notif sn;

    dbg.step = 5;
    while (1) {
        int64_t now = k_uptime_get();
        bool go_on = false;

        // WATCHDOG: one feed per pass; every state comes back here within 2 s
        wdt_feed();
        if (!freeze_count_cleared && now >= WDT_HEALTHY_MS) {
            NRF_POWER->GPREGRET2 = 0;   // healthy: give future freezes a fresh count
            freeze_count_cleared = true;
        }

        switch (st) {
        case ST_OFF:
            // WATCHDOG: sleep until an interrupt, but wake every 2 s just to feed
            if (k_sem_take(&wake_sem, K_MSEC(WDT_IDLE_FEED_MS)) != 0) {
                break;
            }
            handle_buttons();

            if (notif_waiting()) {
                touch_flag = false;
                motion_flag = false;
                wake_reason = "Notification";
                go_on = true;
                break;
            }

            if (touch_flag) {
                touch_flag = false;
                motion_flag = false;
                dbg.touches++;
                wake_reason = "Touch";
                go_on = true;
                break;
            }

            if (motion_flag) {
                motion_flag = false;

                nrf_gpio_pin_set(DB_LED_PIN);
                k_msleep(15);
                nrf_gpio_pin_clear(DB_LED_PIN);

                if (read_axes(mg) == 0) {
                    int start_face = face_mg(mg);
                    if (start_face >= IGNORE_ABOVE_MG) {
                        dbg.ignored++;
                    } else {
                        raise_target = (start_face >= VIEW_ON_MG)
                                       ? start_face + RAISE_DELTA_MG
                                       : VIEW_ON_MG;
                        t_motion = k_uptime_get();
                        settle_since = -1;
                        st = ST_CHECKING;
                    }
                }
            }
            break;

        case ST_CHECKING:
            handle_buttons();
            if (notif_waiting()) {
                wake_reason = "Notification";
                go_on = true;
                break;
            }
            if (touch_flag) {
                touch_flag = false;
                dbg.touches++;
                wake_reason = "Touch";
                go_on = true;
                break;
            }
            if (read_axes(mg) == 0 && face_mg(mg) >= raise_target) {
                if (settle_since < 0) {
                    settle_since = now;
                } else if (now - settle_since >= SETTLE_MS) {
                    dbg.raises++;
                    wake_reason = "Raise";
                    go_on = true;
                    break;
                }
            } else {
                settle_since = -1;
            }
            if (now - t_motion > RAISE_WINDOW_MS) {
                dbg.misses++;
                k_sem_reset(&wake_sem);
                motion_flag = false;
                if (notif_waiting()) k_sem_give(&wake_sem);
                st = ST_OFF;
                break;
            }
            k_msleep(20);
            break;

        case ST_ON:
        case ST_DIM:
            handle_buttons();

            // New notification while awake: show it at full brightness
            if (notif_take(&sn)) {
                notif_show(&sn);
                if (st == ST_DIM) {
                    lv_task_handler();
                    fade_to(BRIGHT_FULL, FADE_IN_MS);
                    st = ST_ON;
                }
                wake_reason = "Notification";
                t_on = k_uptime_get();
                away_since = -1;
            }

            // Cleared on the phone: clear it here too
            if (removed_flag) {
                removed_flag = false;
                if (notif_visible && removed_uid == notif_shown_uid) notif_hide();
            }

            // Shown long enough: back to the watch face
            if (notif_visible && now - notif_shown_at >= NOTIF_SHOW_MS) {
                notif_hide();
            }

            if (touch_flag) {
                touch_flag = false;
                notif_hide();   // tap dismisses a notification
                if (st == ST_DIM) {
                    fade_to(BRIGHT_FULL, FADE_IN_MS);
                    st = ST_ON;
                }
                t_on = k_uptime_get();
                away_since = -1;
            }

            if (now - last_read >= 50) {
                last_read = now;

                update_bt_state();
                update_clock();
                update_wake(wake_reason);

                if (read_axes(mg) == 0) {
                    int f = face_mg(mg);
                    if (st == ST_ON) {
                        if (f < VIEW_OFF_MG) {
                            if (away_since < 0) away_since = now;
                        } else {
                            away_since = -1;
                        }
                        update_stats();
                    } else if (strcmp(dim_reason, "Away") == 0 && f >= VIEW_ON_MG) {
                        fade_to(BRIGHT_FULL, FADE_IN_MS);
                        st = ST_ON;
                        t_on = k_uptime_get();
                        away_since = -1;
                    } else {
                        int left = (int)((DIM_HOLD_MS - (now - dim_start) + 999) / 1000);
                        set_lbl(&L_STAT, C_GRAY, "Dim  %d", left);
                    }
                }
            }

            lv_task_handler();

            if (st == ST_ON) {
                const char *reason = NULL;
                // A notification on screen stays bright until it times out or is dismissed
                if (!notif_visible) {
                    if (away_since >= 0 && now - away_since >= AWAY_DIM_MS) reason = "Away";
                    if (now - t_on >= ON_TIMEOUT_MS) reason = "Timeout";
                }
                if (reason) {
                    dim_reason = reason;
                    fade_to(BRIGHT_DIM, FADE_DIM_MS);
                    dim_start = k_uptime_get();
                    st = ST_DIM;
                }
            } else if (now - dim_start >= DIM_HOLD_MS) {
                display_sleep();
                k_sem_reset(&wake_sem);
                motion_flag = false;
                if (touch_flag || notif_waiting()) k_sem_give(&wake_sem);
                st = ST_OFF;
                break;
            }
            k_msleep(20);
            break;
        }

        if (go_on) {
            int64_t t0 = (strcmp(wake_reason, "Raise") == 0) ? t_motion : k_uptime_get();
            display_wake();
            dbg.last_lat_ms = (uint32_t)(k_uptime_get() - t0);
            if (notif_take(&sn)) notif_show(&sn);
            // Draw fresh content at brightness 0, then fade in
            update_bt_state();
            update_clock();
            update_wake(wake_reason);
            update_stats();
            lv_task_handler();
            fade_to(BRIGHT_FULL, FADE_IN_MS);
            t_on = k_uptime_get();
            away_since = -1;
            touch_flag = false;
            st = ST_ON;
        }
    }
    return 0;
}