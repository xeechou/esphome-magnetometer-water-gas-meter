#ifdef MOCK_MODE
#    include "sensor_update_api.h"
#    include <cstdio>
#    include <cstdarg>
#    include <cstdlib>
#    include <chrono>
#    include <cstring>

#    include <string>

void
mock_publish_state(void* entity, float value)
{ printf("[MOCK] Publish state: %p = %f\n", entity, value); }

void
mock_publish_state(void* entity, const char* value)
{ printf("[MOCK] Publish state: %p = %s\n", entity, value); }

float mock_get_state(void* entity) {
    (void)entity;
    return 0.0f;
}

uint32_t mock_millis() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

void mock_log_w(const char* tag, const char* fmt, ...) {
    fprintf(stderr, "[%s] WARNING: ", tag);
    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fprintf(stderr, "\n");
}

void mock_log_i(const char* tag, const char* fmt, ...) {
    fprintf(stderr, "[%s] INFO: ", tag);
    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fprintf(stderr, "\n");
}

std::string mock_str_sprintf(const char* fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    return std::string(buf);
}

#endif
