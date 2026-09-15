
#include <stddef.h>
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { return -1; }
int _getentropy(void *buffer, size_t length) { return -1; }
