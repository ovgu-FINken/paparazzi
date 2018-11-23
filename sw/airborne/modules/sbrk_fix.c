#include <std.h>

void* _sbrk(int a);
void* _sbrk(int a __attribute__((unused))) { return NULL; }
