#ifndef HAL_BLE_H
#define HAL_BLE_H

/*
 * Bluetooth link to the iPhone and the three Apple services the watch uses:
 *   CTS  (Current Time Service)                 the phone's time
 *   ANCS (Apple Notification Center Service)    notifications, including calls
 *   AMS  (Apple Media Service)                  the music player
 * Watch: the Zephyr Bluetooth stack and NCS clients. Simulator: the Bluetooth
 * section of the dev tools.
 *
 * Things arrive as events. On the watch they come from the Bluetooth thread and are
 * queued; the main loop takes them one at a time with ble_poll(), so all screen
 * updates still happen on the main loop.
 *
 * All functions return 0 on success or a negative error code (-EIO when the
 * Bluetooth radio does not work).
 */

#include <stdbool.h>
#include <stdint.h>

/* Longest text the watch receives, in bytes. NCS 2.6's ANCS client accepts at most
 * BT_ANCS_ATTR_DATA_MAX (32) bytes per field; anything longer is cut off. */
#define BLE_TEXT_MAX 32

enum ble_link {
    BLE_LINK_DISCONNECTED,   /* not connected and not advertising */
    BLE_LINK_ADVERTISING,    /* waiting for the phone to connect */
    BLE_LINK_CONNECTED,      /* connected, not yet encrypted */
    BLE_LINK_BONDED,         /* encrypted with saved keys: CTS, ANCS and AMS work */
};

/* ---- Time (CTS) ---- */

/* Bits in ble_time.adjust_reason, as CTS sends them */
#define BLE_TIME_MANUAL       (1u << 0)   /* the user changed the time on the phone */
#define BLE_TIME_EXTERNAL     (1u << 1)   /* the phone synced from the network */
#define BLE_TIME_ZONE_CHANGED (1u << 2)
#define BLE_TIME_DST_CHANGED  (1u << 3)

/* The phone's local time */
struct ble_time {
    uint16_t year;
    uint8_t month;            /* 1-12 */
    uint8_t day;              /* 1-31 */
    uint8_t hour;             /* 0-23 */
    uint8_t minute;
    uint8_t second;
    int16_t utc_offset_min;   /* time zone plus daylight saving, e.g. -240 for EDT */
    uint8_t adjust_reason;    /* BLE_TIME_* bits */
};

/* ---- Notifications (ANCS) ---- */

/* Category numbers, as ANCS sends them */
enum ble_notif_category {
    BLE_NOTIF_OTHER,
    BLE_NOTIF_INCOMING_CALL,
    BLE_NOTIF_MISSED_CALL,
    BLE_NOTIF_VOICEMAIL,
    BLE_NOTIF_SOCIAL,
    BLE_NOTIF_SCHEDULE,
    BLE_NOTIF_EMAIL,
    BLE_NOTIF_NEWS,
    BLE_NOTIF_HEALTH_AND_FITNESS,
    BLE_NOTIF_BUSINESS_AND_FINANCE,
    BLE_NOTIF_LOCATION,
    BLE_NOTIF_ENTERTAINMENT,
};

/* Bits in ble_notif.flags, as ANCS sends them */
#define BLE_NOTIF_FLAG_SILENT       (1u << 0)
#define BLE_NOTIF_FLAG_IMPORTANT    (1u << 1)
#define BLE_NOTIF_FLAG_PREEXISTING  (1u << 2)   /* was already on the phone when we connected */
#define BLE_NOTIF_FLAG_POSITIVE     (1u << 3)   /* has a positive action (e.g. accept call) */
#define BLE_NOTIF_FLAG_NEGATIVE     (1u << 4)   /* has a negative action (e.g. decline call) */

struct ble_notif {
    uint32_t id;                       /* the phone's id for it, used to remove or act on it */
    uint8_t category;                  /* enum ble_notif_category */
    uint8_t flags;                     /* BLE_NOTIF_FLAG_* bits */
    char app[BLE_TEXT_MAX + 1];        /* app identifier, e.g. "com.apple.MobileSMS" */
    char title[BLE_TEXT_MAX + 1];      /* sender, or caller name for a call */
    char message[BLE_TEXT_MAX + 1];
};

enum ble_notif_action {
    BLE_ACTION_POSITIVE,   /* e.g. accept a call */
    BLE_ACTION_NEGATIVE,   /* e.g. decline a call, or dismiss */
};

/* ---- Music (AMS) ---- */

enum ble_music_state {
    BLE_MUSIC_PAUSED,
    BLE_MUSIC_PLAYING,
    BLE_MUSIC_REWINDING,
    BLE_MUSIC_FAST_FORWARDING,
};

struct ble_music {
    char title[BLE_TEXT_MAX + 1];
    char artist[BLE_TEXT_MAX + 1];
    char album[BLE_TEXT_MAX + 1];
    uint8_t state;           /* enum ble_music_state */
    uint32_t elapsed_ms;     /* track position when this update was sent */
    uint32_t duration_ms;    /* track length */
    uint32_t updated_ms;     /* time_ms() when it arrived; while playing, position = */
                             /* elapsed_ms + (time_ms() - updated_ms) */
    uint8_t volume;          /* 0-100 */
};

/* Command numbers, as AMS defines them */
enum ble_music_cmd {
    BLE_MUSIC_PLAY,
    BLE_MUSIC_PAUSE,
    BLE_MUSIC_TOGGLE,
    BLE_MUSIC_NEXT,
    BLE_MUSIC_PREVIOUS,
    BLE_MUSIC_VOLUME_UP,
    BLE_MUSIC_VOLUME_DOWN,
};

/* ---- Events ---- */

enum ble_event_type {
    BLE_EVENT_LINK,            /* link state changed: see .link */
    BLE_EVENT_TIME,            /* the phone sent its time: see .time */
    BLE_EVENT_NOTIF_ADDED,     /* see .notif (a call is a notification with category INCOMING_CALL) */
    BLE_EVENT_NOTIF_REMOVED,   /* only .notif.id is set (a call that ended is removed too) */
    BLE_EVENT_MUSIC,           /* the player changed: see .music */
};

/* Only the member that matches .type is filled in. */
struct ble_event {
    enum ble_event_type type;
    enum ble_link link;
    struct ble_time time;
    struct ble_notif notif;
    struct ble_music music;
};

int ble_init(void);

/* Takes the next event: returns 1 and fills *event, 0 if there are none, or a
 * negative error code. Call it until it returns 0 each pass of the main loop. */
int ble_poll(struct ble_event *event);

int ble_get_link(enum ble_link *link);

/* Accept or decline a call, or act on another notification (ANCS Perform Action). */
int ble_notif_action(uint32_t id, enum ble_notif_action action);

/* Send a play, pause, next... command to the phone's music player. */
int ble_music_command(enum ble_music_cmd cmd);

#endif /* HAL_BLE_H */
