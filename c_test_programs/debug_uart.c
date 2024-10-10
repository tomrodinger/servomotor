#include <stdio.h>
#include <stdint.h>

void print_int64(char *message_prefix, int64_t n)
{
    printf("%s: %lld\n", message_prefix, n);
}