/*
 * ACS4 Flight Computer — RAM Log (Circular Buffer) Implementation
 */

#include "logger/ram_log.h"

#include <cstring>

#include "logger/log_format.h"

extern "C" {
#include "ch.h"

#include <chprintf.h>
}

namespace acs
{

static constexpr size_t RAM_LOG_SIZE = 53248; /* 52 KiB */

static uint8_t s_ram_buf[RAM_LOG_SIZE] __attribute__((aligned(4)));

static size_t s_head = 0;
static size_t s_used = 0;

void ram_log_init()
{
    s_head = 0;
    s_used = 0;
}

void ram_log_push(const void *data, size_t len)
{
    if (len == 0 || len > RAM_LOG_SIZE)
    {
        return;
    }

    const auto *src = static_cast<const uint8_t *>(data);

    const size_t remaining = RAM_LOG_SIZE - s_head;
    if (len <= remaining)
    {
        memcpy(&s_ram_buf[s_head], src, len);
    }
    else
    {
        memcpy(&s_ram_buf[s_head], src, remaining);
        memcpy(s_ram_buf, src + remaining, len - remaining);
    }

    s_head = (s_head + len) % RAM_LOG_SIZE;

    s_used += len;
    if (s_used > RAM_LOG_SIZE)
    {
        s_used = RAM_LOG_SIZE;
    }
}

void ram_log_get(const uint8_t *&buf, size_t &size, size_t &head, size_t &used)
{
    buf  = s_ram_buf;
    size = RAM_LOG_SIZE;
    head = s_head;
    used = s_used;
}

void ram_log_print_status(BaseSequentialStream *chp)
{
    chprintf(chp, "RAM log buffer: %u / %u bytes used\r\n",
             static_cast<unsigned>(s_used),
             static_cast<unsigned>(RAM_LOG_SIZE));

    if (s_used > 0)
    {
        const uint32_t est_records = static_cast<uint32_t>(s_used) / sizeof(LogImu);
        chprintf(chp, "~%lu records (avg %uB)\r\n", est_records,
                 sizeof(LogImu));
    }
}

}  // namespace acs
