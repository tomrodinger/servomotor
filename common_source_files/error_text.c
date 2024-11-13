#include <stdint.h>

char error_text[] =
    "\0"                                      // error 0
    "time went backwards\0"                   // error 1
    "flash unlock fail\0"                     // error 2
    "flash write fail\0"                      // error 3
    "too many bytes\0"                        // error 4
    "command overflow\0"                      // error 5
    "command too long\0"                      // error 6
    "not in open loop\0"                      // error 7
    "queue not empty\0"                       // error 8
    "hall sensor error\0"                     // error 9
    "calibration overflow\0"                  // error 10
    "not enough minima or maxima\0"           // error 11
    "vibration four step\0"                   // error 12
    "not in closed loop\0"                    // error 13
    "overvoltage\0"                           // error 14
    "accel too high\0"                        // error 15
    "vel too high\0"                          // error 16
    "queue is full\0"                         // error 17
    "run out of queue items\0"                // error 18
    "motor busy\0"                            // error 19
    "position out of range\0"                 // error 20   TO BE DEPRECATED
    "hall position out of range\0"            // error 21   TO BE DEPRECATED
    "current sensor failed\0"                 // error 22
    "max pwm voltage too high\0"              // error 23
    "multi-move more than 32 moves\0"         // error 24
    "safety limit exceeded\0"                 // error 25
    "turn point out of safety zone\0"         // error 26
    "predicted position out of safety zone\0" // error 27
    "predicted velocity too high\0"           // error 28
    "debug1\0"                                // error 29
    "control loop took too long\0"            // error 30
    "index out of range\0"                    // error 31
    "can't pulse when intervals are active\0" // error 32
    "invalid run mode\0"                      // error 33
    "parameter out of range\0"                // error 34
    "disable MOSFETs first\0"                 // error 35
    "framing error\0"                         // error 36
    "overrun error\0"                         // error 37
    "noise error\0"                           // error 38
    "go to closed loop failed\0"              // error 39
    "overheat\0"                              // error 40
    "test mode active\0"                      // error 41
    "position discrepancy\0"                  // error 42
    "overcurrent\0"                           // error 43
    "PWM too high\0"                          // error 44
    "position deviation too large\0"          // error 45
    "move too far\0"                          // error 46
    "hall position delta too large\0"         // error 47
    "\0";                                     // this marks the end of the error messages

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
