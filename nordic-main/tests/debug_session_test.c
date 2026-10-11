#include <assert.h>
#include <stdio.h>
#include "debug_session.h"
int main(void) {
    struct debug_session s = {0};
    assert(!debug_session_enabled(&s));
    assert(!debug_session_allowed(&s, 1));
    assert(debug_session_request(&s, 0, 1, 0) == DEBUG_OK);
    assert(!debug_session_enabled(&s));
    assert(debug_session_request(&s, 1, 1, 100) == DEBUG_OK);
    assert(debug_session_allowed(&s, 1));
    assert(!debug_session_allowed(&s, 2));
    assert(debug_session_remaining(&s, 100) == 600);
    assert(debug_session_request(&s, 1, 2, 200) == DEBUG_BUSY);
    assert(debug_session_request(&s, 2, 2, 200) == DEBUG_BUSY);
    assert(debug_session_request(&s, 3, 1, 200) == DEBUG_INVALID);
    assert(debug_session_request(&s, 1, 0, 200) == DEBUG_INVALID);
    assert(!debug_session_expire(&s, 600099, true));
    assert(debug_session_remaining(&s, 600099) == 1);
    assert(debug_session_expire(&s, 600100, true));
    assert(!debug_session_enabled(&s));
    assert(debug_session_request(&s, 1, 2, 600101) == DEBUG_OK);
    assert(debug_session_request(&s, 1, 2, 600200) == DEBUG_OK);
    assert(s.deadline_ms == 1200200); /* explicit renewal */
    assert(debug_session_expire(&s, 600201, false));
    assert(!debug_session_enabled(&s));
    assert(debug_session_request(&s, 1, 1, 600202) == DEBUG_OK);
    assert(debug_session_request(&s, 2, 1, 600203) == DEBUG_OK);
    assert(!debug_session_enabled(&s));
    puts("PASS: debug ownership, timeout, disconnect, renewal, validation");
}
