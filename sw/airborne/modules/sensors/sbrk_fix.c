

/* workaround for newlib */
void *_sbrk(int);
void *_sbrk(int a __attribute__((unused))) {return 0;}

