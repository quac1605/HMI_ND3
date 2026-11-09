// Test only: redirect the original hard-coded serial path to a PTY, without changing source.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
int open(const char *path, int flags, ...) {
  static int (*real_open)(const char *, int, ...) = 0;
  if (!real_open) real_open = dlsym(RTLD_NEXT, "open");
  mode_t mode = 0;
  if (flags & O_CREAT) { va_list ap; va_start(ap, flags); mode = va_arg(ap, int); va_end(ap); }
  if (!strncmp(path, "/dev/ttyUSB", 11)) {
    const char *pty = getenv("ZLAC_TEST_PTY");
    if (!pty || strncmp(pty, "/dev/pts/", 9)) { errno = EACCES; return -1; }
    path = pty;
  }
  return real_open(path, flags, mode);
}
