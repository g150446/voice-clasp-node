#ifndef DEBUG_SESSION_H
#define DEBUG_SESSION_H
#include <stdbool.h>
#include <stdint.h>

#define DEBUG_SESSION_MS 600000
/* Wire operations: 0=query, 1=enable/renew, 2=disable. */
enum debug_status { DEBUG_OK, DEBUG_REQUIRED, DEBUG_INVALID, DEBUG_BUSY, DEBUG_QUEUE_FULL };
struct debug_session { uintptr_t owner; int64_t deadline_ms; };
static inline bool debug_session_enabled(const struct debug_session *s) { return s->owner != 0; }
static inline bool debug_session_allowed(const struct debug_session *s, uintptr_t owner) {
    return owner != 0 && s->owner == owner;
}
static inline void debug_session_clear(struct debug_session *s) { s->owner = 0; s->deadline_ms = 0; }
uint8_t debug_session_request(struct debug_session *s, uint8_t op, uintptr_t owner, int64_t now);
bool debug_session_expire(struct debug_session *s, int64_t now, bool owner_connected);
uint16_t debug_session_remaining(const struct debug_session *s, int64_t now);
#endif
