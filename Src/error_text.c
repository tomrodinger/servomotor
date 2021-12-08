#include <stdint.h>

char error_text[] =
    "\0"                             // error 0
    "time went backwards\0"          // error 1
    "flash unlock fail\0"            // error 2
    "flash write fail\0"             // error 3
    "too many bytes\0"               // error 4
    "command overflow\0"             // error 5
    "command too long\0"             // error 6
    "not in open loop\0"             // error 7
    "queue not empty\0"              // error 8
    "hall sensor error\0"            // error 9
    "calibration overflow\0"         // error 10
    "not enough minima or maxima\0"  // error 11
    "vibration four step\0"          // error 12
    "not in closed loop\0"           // error 13
    "overvoltage\0"                  // error 14
    "accel too high\0"               // error 15
    "vel too high\0"                 // error 16
    "queue is full\0"                // error 17
    "run out of queue items\0"       // error 18
    "motor busy\0"                   // error 19
    "position out of range\0"        // error 20
    "hall position out of range\0"   // error 21
    "current sensor failed\0"        // error 22
    "max motor current too high\0"   // error 23
    "multi-move more than 32 moves\0"// error 24
    "\0";                            // this marks the end of the error messages

char *get_error_text(uint16_t error_code)
{
    char *ptr = error_text;
    uint16_t i;

    for(i = 0; i < error_code; i++) {
        while(*ptr != 0) {
            ptr += 1;
        }
        ptr += 1;
        if(*ptr == 0) {
            return "unknown error";
        }
    }
    return ptr;
}
