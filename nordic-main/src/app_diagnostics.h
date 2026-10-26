#ifndef APP_DIAGNOSTICS_H
#define APP_DIAGNOSTICS_H
#include <stdbool.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
bool app_debug_enabled(void);
/* Do not evaluate format arguments outside an explicitly enabled session. */
#define APP_TRACE(...) do { if (app_debug_enabled()) printk(__VA_ARGS__); } while (0)
#define APP_INFO(...) do { if (app_debug_enabled()) LOG_INF(__VA_ARGS__); } while (0)
#endif
