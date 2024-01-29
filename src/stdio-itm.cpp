
#include <unistd.h>
#include <cerrno>
#include <cortex_m/debug.h>

extern "C" int _write(int file, char* ptr, int len) {
        int i;

        if (file == STDOUT_FILENO || file == STDERR_FILENO) {
                for (i = 0; i < len; i++) {
                        if (ptr[i] == '\n') {
                                ITM->stim_blocking(0, '\r');
                        }
                        ITM->stim_blocking(0, ptr[i]);
                }
                return i;
        }
        errno = EIO;
        return -1;
}
