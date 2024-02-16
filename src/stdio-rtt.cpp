
#include <unistd.h>
#include <cerrno>
#include <cortex_m/debug.h>

#include "SEGGER_RTT.h"

extern "C" int _write(int file, char *ptr, int len)
{
        (void)file;
        SEGGER_RTT_Write(0, (const char *)ptr, len);
        return len;
}
