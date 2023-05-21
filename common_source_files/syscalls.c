#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>

int _close(int file)
{
    return -1;
}

off_t _lseek(int file, off_t position, int whence)
{
    return -1;
}

ssize_t _read(int file, void *buf, size_t nbyte)
{
    return -1;
}

ssize_t _write(int file, const void *buf, size_t nbyte)
{
    return -1;
}
