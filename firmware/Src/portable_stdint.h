#ifndef __PORTABLE_STDINT_H__
#define __PORTABLE_STDINT_H__

// For 64-bit integers, adjust based on platform
#if defined(__arm__) && !defined(__aarch64__)
// Cortex-M0+ is 32-bit
#define _PRId32 "%ld"
#define _PRIu32 "%lu"
#define _PRId64 "%lld"
#define _PRIu64 "%llu"
#else
// Mac (ARM64) is 64-bit
#define _PRId32 "%d"
#define _PRIu32 "%u"
#define _PRId64 "%lld"
#define _PRIu64 "%llu"
#endif

#endif // __PORTABLE_STDINT_H__