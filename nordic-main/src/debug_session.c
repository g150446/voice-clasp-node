#include "debug_session.h"
uint8_t debug_session_request(struct debug_session *s, uint8_t op, uintptr_t owner, int64_t now)
{
    if (op > 2 || !owner) return DEBUG_INVALID;
    if (!op) return DEBUG_OK;
    if (s->owner && s->owner != owner) return DEBUG_BUSY;
    if (op == 1) { s->owner = owner; s->deadline_ms = now + DEBUG_SESSION_MS; }
    else debug_session_clear(s);
    return DEBUG_OK;
}
bool debug_session_expire(struct debug_session *s, int64_t now, bool owner_connected)
{
    if (s->owner && (!owner_connected || now >= s->deadline_ms)) {
        debug_session_clear(s); return true;
    }
    return false;
}
uint16_t debug_session_remaining(const struct debug_session *s, int64_t now)
{
    return !s->owner || now >= s->deadline_ms ? 0 : (uint16_t)((s->deadline_ms - now + 999) / 1000);
}
