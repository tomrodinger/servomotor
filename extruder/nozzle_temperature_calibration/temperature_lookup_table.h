#ifndef TEMPERATURE_LOOKUP_TABLE_H
#define TEMPERATURE_LOOKUP_TABLE_H

#define TEMPERATURE_LOOKUP_TABLE_SHIFT_LEFT 7
#define TEMPERATURE_LOOKUP_TABLE_SLOPE_SHIFT_LEFT 19
#define TEMPERATURE_LOOKUP_TABLE_SIZE 30

typedef struct __attribute__((__packed__)) {
    uint16_t temperature;
    uint16_t slope;
} temperature_lookup_table_entry_t;

// Integer based lookup table in C format:
const temperature_lookup_table_entry_t temperature_lookup_table[] = {
    { 4296, 10888 },
    { 5657, 8509 },
    { 6720, 7084 },
    { 7606, 6260 },
    { 8388, 5784 },
    { 9111, 5492 },
    { 9798, 5281 },
    { 10458, 5099 },
    { 11096, 4928 },
    { 11712, 4772 },
    { 12308, 4645 },
    { 12889, 4567 },
    { 13460, 4552 },
    { 14029, 4611 },
    { 14605, 4742 },
    { 15198, 4937 },
    { 15815, 5180 },
    { 16462, 5452 },
    { 17144, 5738 },
    { 17861, 6035 },
    { 18615, 6364 },
    { 19411, 6777 },
    { 20258, 7380 },
    { 21181, 8343 },
    { 22224, 9918 },
    { 23463, 12468 },
    { 25022, 16481 },
    { 27082, 22598 },
    { 29907, 31642 },
    { 33862, 44645 },
};

#endif
