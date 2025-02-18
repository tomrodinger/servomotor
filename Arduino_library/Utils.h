// Utils.h
#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stddef.h>

// Detect endianness at compile time
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
    // Little-endian definitions
    #define htole16(x) (x)
    #define htole32(x) (x)
    #define htole64(x) (x)
    #define le16toh(x) (x)
    #define le32toh(x) (x)
    #define le64toh(x) (x)
    #define htobe16(x) __builtin_bswap16(x)
    #define htobe32(x) __builtin_bswap32(x)
    #define htobe64(x) __builtin_bswap64(x)
    #define be16toh(x) __builtin_bswap16(x)
    #define be32toh(x) __builtin_bswap32(x)
    #define be64toh(x) __builtin_bswap64(x)
#elif defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
    // Big-endian definitions
    #define htole16(x) __builtin_bswap16(x)
    #define htole32(x) __builtin_bswap32(x)
    #define htole64(x) __builtin_bswap64(x)
    #define le16toh(x) __builtin_bswap16(x)
    #define le32toh(x) __builtin_bswap32(x)
    #define le64toh(x) __builtin_bswap64(x)
    #define htobe16(x) (x)
    #define htobe32(x) (x)
    #define htobe64(x) (x)
    #define be16toh(x) (x)
    #define be32toh(x) (x)
    #define be64toh(x) (x)
#else
    // Fallback method to detect endianness
    #include <endian.h>
    #if defined(__BYTE_ORDER) && (__BYTE_ORDER == __LITTLE_ENDIAN)
        // Little-endian definitions
        #define htole16(x) (x)
        #define htole32(x) (x)
        #define htole64(x) (x)
        #define le16toh(x) (x)
        #define le32toh(x) (x)
        #define le64toh(x) (x)
        #define htobe16(x) __builtin_bswap16(x)
        #define htobe32(x) __builtin_bswap32(x)
        #define htobe64(x) __builtin_bswap64(x)
        #define be16toh(x) __builtin_bswap16(x)
        #define be32toh(x) __builtin_bswap32(x)
        #define be64toh(x) __builtin_bswap64(x)
    #elif defined(__BYTE_ORDER) && (__BYTE_ORDER == __BIG_ENDIAN)
        // Big-endian definitions
        #define htole16(x) __builtin_bswap16(x)
        #define htole32(x) __builtin_bswap32(x)
        #define htole64(x) __builtin_bswap64(x)
        #define le16toh(x) __builtin_bswap16(x)
        #define le32toh(x) __builtin_bswap32(x)
        #define le64toh(x) __builtin_bswap64(x)
        #define htobe16(x) (x)
        #define htobe32(x) (x)
        #define htobe64(x) (x)
        #define be16toh(x) (x)
        #define be32toh(x) (x)
        #define be64toh(x) (x)
    #else
        #error "Unknown byte order"
    #endif
#endif

#endif // UTILS_H
