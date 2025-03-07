#include <errno.h>
#if defined(__APPLE__) || defined(__FreeBSD__) || defined(__NetBSD__)
#include <util.h>      // openpty() on macOS, FreeBSD, NetBSD
#else
#include <pty.h>       // openpty() on Linux and most other UNIX systems
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include <signal.h>

extern void simulate_ADC_hall_sensor_values(void);

// Undefine system termios.h macros that conflict with our register names
#undef CR1
#undef CR2
#undef CR3

// Then our headers
#include "motor_hal.h"
#include "RS485.h"
#include "mosfets.h"
#include "motor_control.h"
#include "simulator_reset.h"

// External declarations from RS485.c
extern volatile uint8_t commandReceived;
extern volatile char selectedAxis;
extern volatile uint8_t command;
extern volatile uint8_t valueBuffer[MAX_VALUE_BUFFER_LENGTH];
extern int main_simulation(void);

// SDL headers
#include <SDL.h>
#include <SDL_ttf.h>

// -----------------------------------------------------------------------------
// Additional commands from the firmware (main.c). For brevity, not all commands
// are shown, but you can add or remove as needed.
// -----------------------------------------------------------------------------
#define DISABLE_MOSFETS_COMMAND                    0
#define ENABLE_MOSFETS_COMMAND                     1
#define TRAPEZOID_MOVE_COMMAND                     2
#define SET_MAX_VELOCITY_COMMAND                   3
#define GO_TO_POSITION_COMMAND                     4
#define SET_MAX_ACCELERATION_COMMAND               5
#define START_CALIBRATION_COMMAND                  6
#define CAPTURE_HALL_SENSOR_DATA_COMMAND           7
#define RESET_TIME_COMMAND                         8
#define GET_CURRENT_TIME_COMMAND                   9
#define TIME_SYNC_COMMAND                          10
#define GET_N_ITEMS_IN_QUEUE_COMMAND               11
#define EMERGENCY_STOP_COMMAND                     12
#define ZERO_POSITION_COMMAND                      13
#define HOMING_COMMAND                             14
#define GET_POSITION_COMMAND                       15
#define GET_HALL_SENSOR_POSITION_COMMAND           16
#define GET_COMPREHENSIVE_POSITION_COMMAND         17
#define GET_STATUS_COMMAND                         18
#define GO_TO_CLOSED_LOOP_COMMAND                  19
#define GET_UPDATE_FREQUENCY_COMMAND               20
#define MOVE_WITH_ACCELERATION_COMMAND             21
#define DETECT_DEVICES_COMMAND                     22
#define SET_DEVICE_ALIAS_COMMAND                   23
#define GET_PRODUCT_INFO_COMMAND                   24
#define GET_PRODUCT_DESCRIPTION_COMMAND            25
#define GET_FIRMWARE_VERSION_COMMAND               26
#define MOVE_WITH_VELOCITY_COMMAND                 27
#define SYSTEM_RESET_COMMAND                       28
#define SET_MAXIMUM_MOTOR_CURRENT                  29
#define MULTI_MOVE_COMMAND                         30
#define SET_SAFETY_LIMITS_COMMAND                  31
#define ADD_TO_QUEUE_TEST_COMMAND                  32
#define PING_COMMAND                               33
#define CONTROL_HALL_SENSOR_STATISTICS_COMMAND     34
#define GET_HALL_SENSOR_STATISTICS_COMMAND         35
#define READ_MULTIPURPOSE_BUFFER_COMMAND           36
#define GET_SUPPLY_VOLTAGE_COMMAND                 37
#define GET_MAX_PID_ERROR_COMMAND                  38
#define TEST_MODE_COMMAND                          39
#define VIBRATE_COMMAND                            40
#define IDENTIFY_COMMAND                           41
#define GET_TEMPERATURE_COMMAND                    42
#define SET_PID_CONSTANTS_COMMAND                  43
#define SET_MAX_ALLOWABLE_POSITION_DEVIATION       44
#define GET_DEBUG_VALUES_COMMAND                   45

// -----------------------------------------------------------------------------
// Original Protocol & Visualization defines
// -----------------------------------------------------------------------------
static const char *font_path = "/System/Library/Fonts/Supplemental/Arial.ttf";
#define RESPONSE_CHAR        0xFE
#define RESPONSE_NO_PAYLOAD  0
#define RESPONSE_HAS_PAYLOAD 1
#define AXIS_MOTOR           0
#define AXIS_ALL             255

#define ERROR_PARAMETER_OUT_OF_RANGE    34

// Colors
static const SDL_Color COLOR_DARK_GREY = {80, 80, 80, 255};
static const SDL_Color COLOR_SILVER    = {192, 192, 192, 255};
static const SDL_Color COLOR_RED       = {200, 30, 30, 255};

// Globals
static int      gMasterFD = -1;
static int      gQuit     = 0;
static pthread_t gReadThread;
static pthread_t gWriteThread;
static pthread_t gMotorThread;
static pthread_t gSysTickThread;
static bool call_USART1_IRQHandler = false;

// External declaration of SysTick_Handler from main.c
extern void SysTick_Handler(void);

// SysTick simulation thread running at 100Hz
static void *systick_thread_func(void *arg)
{
    (void)arg;
    extern volatile int motor_control_thread_state; // Add extern declaration
    struct timespec lastTime = {0, 0};
    struct timespec lastStatsTime = {0, 0};
    uint32_t tickCount = 0;
    
    while (!gQuit) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        
        // First iteration - initialize start time
        if (lastTime.tv_sec == 0) {
            lastTime = now;
            lastStatsTime = now;
            continue;
        }
        
        // Calculate next target time (10ms = 10000000ns per tick)
        struct timespec target = lastTime;
        target.tv_nsec += 10000000;
        if (target.tv_nsec >= 1000000000) {
            target.tv_sec++;
            target.tv_nsec -= 1000000000;
        }

        // Sleep until just before target time
        struct timespec sleep_target = target;
        if (sleep_target.tv_nsec >= 100000) {  // Leave 100μs for fine-tuning
            sleep_target.tv_nsec -= 100000;
        } else if (sleep_target.tv_sec > 0) {
            sleep_target.tv_sec--;
            sleep_target.tv_nsec = 999900000;  // 100μs before next second
        }
        
        // Calculate time to sleep
        clock_gettime(CLOCK_MONOTONIC, &now);
        struct timespec sleep_time;
        sleep_time.tv_sec = sleep_target.tv_sec - now.tv_sec;
        sleep_time.tv_nsec = sleep_target.tv_nsec - now.tv_nsec;
        if (sleep_time.tv_nsec < 0) {
            sleep_time.tv_sec--;
            sleep_time.tv_nsec += 1000000000;
        }
        
        // Only sleep if we have time
        if (sleep_time.tv_sec > 0 || sleep_time.tv_nsec > 0) {
            nanosleep(&sleep_time, NULL);
        }

        // Brief spinlock for final precision
        do {
            clock_gettime(CLOCK_MONOTONIC, &now);
        } while ((now.tv_sec < target.tv_sec) ||
                (now.tv_sec == target.tv_sec && now.tv_nsec < target.tv_nsec));
        
        // Update target time for next iteration
        lastTime = target;
        
        // Call SysTick handler and count ticks
        SysTick_Handler();
        tickCount++;
        
        // Print stats every 60 seconds (1 minute)
        double elapsed = (now.tv_sec - lastStatsTime.tv_sec) +
                        (now.tv_nsec - lastStatsTime.tv_nsec) / 1e9;
        if (elapsed >= 60.0) {
            double frequency = tickCount / elapsed;
            printf("SysTick frequency: %.2f Hz (target: 100 Hz)\n", frequency);
            printf("Motor control thread state: %d\n", motor_control_thread_state);
            tickCount = 0;
            lastStatsTime = now;
        }
    }
    return NULL;
}
static uint8_t  gMosfetsEnabled = 0;  // Cache MOSFET state
volatile int gResetProgress = 0; // Reset state machine: 0=normal, 1=waiting for exits, 2=ready for reset
volatile int motor_control_thread_state = 0; // Track what the motor control thread is doing

// Simulate transmitting bytes through RS485
static void *write_thread_func(void *arg)
{
    (void)arg;
    printf("Write thread started\n");
    
    while (!gQuit) {
        // Handle transmit
        if ((USART1->ISR & USART_ISR_TXE_TXFNF_Msk) == 0) {
            uint8_t txByte = USART1->TDR;
            ssize_t written = write(gMasterFD, &txByte, 1);
            if (written < 0) {
                printf("Write error: %s (errno: %d)\n", strerror(errno), errno);
                exit(1);
            }
            else if (written == 0) {
                printf("Write failed: no bytes written\n");
                exit(1);
            }            
            // Set TXE to indicate transmit complete
            USART1->ISR |= USART_ISR_TXE_TXFNF_Msk;
        }
        if (   g_interrupts_enabled                                                                   &&
                (((USART1->CR1 & USART_CR1_TXFEIE)         && (USART1->ISR & USART_ISR_TXE_TXFNF_Msk)) ||
                 ((USART1->CR1 & USART_CR1_RXNEIE_RXFNEIE) && (USART1->ISR & USART_ISR_RXNE_RXFNE   )))    ) {
            USART1_IRQHandler();
        }
        usleep(40);    // ~230400 baud timing (each byte takes ~43.4μs)
    }
    return NULL;
}

// Called when MOSFETs are enabled/disabled
void mosfets_state_changed(uint8_t enabled) {
    gMosfetsEnabled = enabled;
}

// -----------------------------------------------------------------------------
// SDL Window/Renderer
// -----------------------------------------------------------------------------
static SDL_Window   *gWindow   = NULL;
static SDL_Renderer *gRenderer = NULL;
static TTF_Font     *gFont     = NULL;
// -----------------------------------------------------------------------------
// SDL Helpers
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Read Thread
// -----------------------------------------------------------------------------
#include "RS485.h"

// Simulate receiving bytes through RS485
static void *read_thread_func(void *arg)
{
    (void)arg;
    printf("Read thread started with gMasterFD: %d\n", gMasterFD);
    
    while (!gQuit) {
        usleep(100);  // Prevent busy waiting
        if ((USART1->ISR & USART_ISR_RXNE_RXFNE) == 0) {
            uint8_t byte;
            int n = read(gMasterFD, &byte, 1);
            if (n < 0) {
                if (errno != EAGAIN) {
                    printf("Read error: %s (errno: %d)\n", strerror(errno), errno);
                }
                continue;
            }
            if (n == 0) {
                printf("FD closed\n");
                break;
            }
            printf("Received: %hhu\n", byte);
            // Store byte and set RXNE flag
            USART1->RDR = byte;
            USART1->ISR |= USART_ISR_RXNE_RXFNE;
            
            // Process through interrupt handler if interrupts are enabled
            if (g_interrupts_enabled) {
                call_USART1_IRQHandler = true;
            }

            if (commandReceived) {
                printf("Command received: Axis=0x%02X, Command=0x%02X\n", selectedAxis, command);
            }            
        }
    }
    return NULL;
}

// -----------------------------------------------------------------------------
// SDL Helpers
// -----------------------------------------------------------------------------
static void draw_filled_circle(SDL_Renderer *rend, int cx, int cy, int radius, SDL_Color color)
{
    SDL_SetRenderDrawColor(rend, color.r, color.g, color.b, color.a);
    for (int dy = -radius; dy <= radius; dy++) {
        int dxmax = (int)floor(sqrt(radius*radius - dy*dy));
        for (int dx = -dxmax; dx <= dxmax; dx++) {
            SDL_RenderDrawPoint(rend, cx + dx, cy + dy);
        }
    }
}

static void draw_text(SDL_Renderer *rend, TTF_Font *font,
                      const char *msg, int x, int y, SDL_Color col)
{
    SDL_Surface *surf = TTF_RenderText_Blended(font, msg, col);
    if (!surf) return;
    SDL_Texture *tex = SDL_CreateTextureFromSurface(rend, surf);
    if (!tex) {
        SDL_FreeSurface(surf);
        return;
    }
    SDL_Rect rect = { x, y, surf->w, surf->h };
    SDL_RenderCopy(rend, tex, NULL, &rect);
    SDL_DestroyTexture(tex);
    SDL_FreeSurface(surf);
}

static void sdl_set_color(SDL_Renderer *rend, SDL_Color c)
{
    SDL_SetRenderDrawColor(rend, c.r, c.g, c.b, c.a);
}

#define WHEEL_SEGMENTS 6
static SDL_Color gWheelColors[WHEEL_SEGMENTS] = {
    {255,   0,   0, 255},
    {255, 165,   0, 255},
    {255, 255,   0, 255},
    {  0, 255,   0, 255},
    {  0,   0, 255, 255},
    {128,   0, 128, 255}
};

static void draw_color_wheel(SDL_Renderer *rend, int cx, int cy, int radius, double angleDeg)
{
    double radOffset = -angleDeg * M_PI / 180.0;
    double step = 2.0 * M_PI / (double)WHEEL_SEGMENTS;
    for (int i = 0; i < WHEEL_SEGMENTS; i++) {
        double a0 = i * step + radOffset;
        double a1 = (i + 1) * step + radOffset;
        sdl_set_color(rend, gWheelColors[i]);
        const int subSteps = 16;
        for (int s = 0; s < subSteps; s++) {
            double t = (double)s / (double)subSteps;
            double a = a0 + (a1 - a0)*t;
            for (int r = 0; r < radius; r++) {
                int px = (int)(cx + cos(a) * r);
                int py = (int)(cy + sin(a) * r);
                SDL_RenderDrawPoint(rend, px, py);
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Render the Motor
// -----------------------------------------------------------------------------
static void render_motor(void)
{
    // Get window size
    int windowW, windowH;
    SDL_GetWindowSize(gWindow, &windowW, &windowH);
    
    // Calculate motor size to fit window
    int motorW = 180, motorH = 180;
    int centerX = windowW / 2;
    int centerY = windowH / 2;
    int bodyX   = centerX - (motorW / 2);
    int bodyY   = centerY - (motorH / 2);

    SDL_Color bodyColor;
    uint16_t error = MotorHAL_GetError();
    if (error != 0) {
        bodyColor = COLOR_RED;
    } else if (gMosfetsEnabled) {
        bodyColor = COLOR_SILVER;
    } else {
        bodyColor = COLOR_DARK_GREY;
    }

    // motor body
    sdl_set_color(gRenderer, bodyColor);
    SDL_Rect r = { bodyX, bodyY, motorW, motorH };
    SDL_RenderFillRect(gRenderer, &r);

    // bolt holes
    sdl_set_color(gRenderer, (SDL_Color){60, 60, 60, 255});
    int holeR = 7;
    draw_filled_circle(gRenderer, bodyX + holeR,         bodyY + holeR,         holeR, (SDL_Color){60,60,60,255});
    draw_filled_circle(gRenderer, bodyX + motorW-holeR,  bodyY + holeR,         holeR, (SDL_Color){60,60,60,255});
    draw_filled_circle(gRenderer, bodyX + holeR,         bodyY + motorH-holeR,  holeR, (SDL_Color){60,60,60,255});
    draw_filled_circle(gRenderer, bodyX + motorW-holeR,  bodyY + motorH-holeR,  holeR, (SDL_Color){60,60,60,255});

    // status text - centered above motor
    char textBuf[128];
    if (error != 0) {
        snprintf(textBuf, sizeof(textBuf), "ERROR = %u", error);
    } else if (gMosfetsEnabled) {
        snprintf(textBuf, sizeof(textBuf), "ENABLED");
    } else {
        snprintf(textBuf, sizeof(textBuf), "DISABLED");
    }
    
    // Get text dimensions to center it
    int textWidth, textHeight;
    TTF_SizeText(gFont, textBuf, &textWidth, &textHeight);
    int textX = centerX - (textWidth / 2);
    draw_text(gRenderer, gFont, textBuf, textX, bodyY - 25, (SDL_Color){255,255,255,255});

    // Motor current - centered below motor with smaller font
    if (gMosfetsEnabled) {
        // Create a smaller version of the font for current display
        TTF_Font *smallFont = TTF_OpenFont(font_path, 12); // 50% of size 24
        if (smallFont) {
            char currentBuf[128];
            snprintf(currentBuf, sizeof(currentBuf), "CURRENT = %u", MotorHAL_GetCurrent());
            
            // Get text dimensions to center it
            int currentWidth, currentHeight;
            TTF_SizeText(smallFont, currentBuf, &currentWidth, &currentHeight);
            int currentX = centerX - (currentWidth / 2);
            
            draw_text(gRenderer, smallFont, currentBuf, currentX, bodyY + motorH + 5, (SDL_Color){255,255,255,255});
            TTF_CloseFont(smallFont);
        }
    }

    // color wheel (motor shaft)
    int shaftRadius = 35;
    draw_color_wheel(gRenderer, centerX, centerY, shaftRadius, MotorHAL_GetPosition());
}

// -----------------------------------------------------------------------------
// Signal Handler (Ctrl-C)
// -----------------------------------------------------------------------------
static void cleanup_and_exit(void)
{
    // Cleanup SDL
    if (gRenderer) SDL_DestroyRenderer(gRenderer);
    if (gWindow)   SDL_DestroyWindow(gWindow);
    if (gFont)     TTF_CloseFont(gFont);

    TTF_Quit();
    SDL_Quit();
    exit(0);
}

static void handle_sigint(int signum)
{
    (void)signum;
    gQuit = 1;
    cleanup_and_exit();
}

// -----------------------------------------------------------------------------
// main()
// -----------------------------------------------------------------------------
// Initialize SDL on the main thread
static void init_sdl(void)
{
    printf("Initializing SDL...\n");
    fflush(stdout);
    
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        exit(1);
    }
    printf("SDL initialized successfully\n");
    fflush(stdout);
    
    printf("Initializing TTF...\n");
    fflush(stdout);
    if (TTF_Init() != 0) {
        fprintf(stderr, "TTF_Init failed: %s\n", TTF_GetError());
        SDL_Quit();
        exit(1);
    }

    printf("Creating window...\n");
    fflush(stdout);
    
    // Create a more compact window for the motor
    int windowWidth = 220;  // Smaller width to fit the motor
    int windowHeight = 260; // Smaller height to fit the motor
    
    // Get the current desktop resolution
    SDL_DisplayMode dm;
    if (SDL_GetCurrentDisplayMode(0, &dm) != 0) {
        fprintf(stderr, "SDL_GetCurrentDisplayMode failed: %s\n", SDL_GetError());
        dm.w = 1920; // Default if we can't get actual value
        dm.h = 1080;
    }
    
    // Position in the top right corner of the current (active) display
    gWindow = SDL_CreateWindow(
        "Motor Firmware Simulator",
        SDL_WINDOWPOS_CENTERED_DISPLAY(0), 0, // Centered horizontally, at top vertically on primary display
        windowWidth, windowHeight,
        SDL_WINDOW_SHOWN
    );
    
    if (!gWindow) {
        fprintf(stderr, "CreateWindow failed: %s\n", SDL_GetError());
        TTF_Quit();
        SDL_Quit();
        exit(1);
    }
    
    // Ensure window appears in the top right
    int x = dm.w - windowWidth;
    int y = 0;
    SDL_SetWindowPosition(gWindow, x, y);
    if (!gWindow) {
        fprintf(stderr, "CreateWindow failed: %s\n", SDL_GetError());
        TTF_Quit();
        SDL_Quit();
        exit(1);
    }

    printf("Creating renderer...\n");
    fflush(stdout);
    gRenderer = SDL_CreateRenderer(gWindow, -1, SDL_RENDERER_ACCELERATED);
    if (!gRenderer) {
        fprintf(stderr, "CreateRenderer failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(gWindow);
        TTF_Quit();
        SDL_Quit();
        exit(1);
    }

    gFont = TTF_OpenFont(font_path, 24);
    if (!gFont) {
        fprintf(stderr, "Warning: Could not open font '%s': %s\n", font_path, TTF_GetError());
    }
}

// Update visualization on main thread
static void update_visualization(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    SDL_Event e;
    if (SDL_PollEvent(&e)) {  // Use if instead of while to make it non-blocking
        if (e.type == SDL_QUIT) {
            gQuit = 1;  
        } else if (e.type == SDL_KEYDOWN) {
            if (e.key.keysym.sym == SDLK_q) {
                gQuit = 1;
            }
        }
    }

    if (gRenderer) {
        SDL_SetRenderDrawColor(gRenderer, 30, 30, 30, 255);
        SDL_RenderClear(gRenderer);
        render_motor();
        SDL_RenderPresent(gRenderer);
    }
}

// These functions are no-ops when building the actual motor firmware.
// They are only implemented for the simulator environment to provide
// visualization and testing capabilities.

// Thread-safe shared memory for motor state
typedef struct {
    pthread_mutex_t mutex;
    int64_t motor_position;
    uint8_t mosfets_enabled;
    double angle_deg;
} MotorSharedState;

static MotorSharedState gSharedState = {
    .mutex = PTHREAD_MUTEX_INITIALIZER
};

// High-priority motor control thread running at 32kHz
static void *motor_control_thread_func(void *arg)
{
    (void)arg;
    motor_control_thread_state = 1; // Thread started
    struct timespec lastTime = {0, 0};
    struct timespec lastStatsTime = {0, 0};
    uint32_t callCount = 0;
    int64_t max_behind_ns = 0;  // Track maximum timing deviation
    
    // Set high priority (macOS doesn't support SCHED_FIFO)
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_RR);
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &param) != 0) {
        printf("Warning: Could not set thread priority\n");
    }
    
    while (!gQuit) {
        motor_control_thread_state = 2; // Getting current time
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        
        // Maintain precise 31.25kHz timing (32μs period)
        if (lastTime.tv_sec == 0) {
            motor_control_thread_state = 3; // Initializing start time
            // First iteration - initialize start time
            clock_gettime(CLOCK_MONOTONIC, &lastTime);
        } else {
            motor_control_thread_state = 4; // Calculating next target time
            // Calculate next target time (32μs = 32000ns per tick)
            struct timespec target = lastTime;
            target.tv_nsec += 32000;
            if (target.tv_nsec >= 1000000000) {
                target.tv_sec++;
                target.tv_nsec -= 1000000000;
            }

            motor_control_thread_state = 5; // Calculating sleep target
            // Sleep until just before target time
            struct timespec sleep_target = target;
            if (sleep_target.tv_nsec >= 2000) {  // Leave 2μs for fine-tuning
                sleep_target.tv_nsec -= 2000;
            } else if (sleep_target.tv_sec > 0) {
                sleep_target.tv_sec--;
                sleep_target.tv_nsec = 999998000;  // 2μs before next second
            }
            
            motor_control_thread_state = 6; // Calculating sleep time
            // Calculate time to sleep
            clock_gettime(CLOCK_MONOTONIC, &now);
            struct timespec sleep_time;
            sleep_time.tv_sec = sleep_target.tv_sec - now.tv_sec;
            sleep_time.tv_nsec = sleep_target.tv_nsec - now.tv_nsec;
            if (sleep_time.tv_nsec < 0) {
                sleep_time.tv_sec--;
                sleep_time.tv_nsec += 1000000000;
            }
            
            motor_control_thread_state = 7; // Sleeping
            // Only sleep if we have time
            if (sleep_time.tv_sec > 0 || sleep_time.tv_nsec > 0) {
                nanosleep(&sleep_time, NULL);
            }

            motor_control_thread_state = 8; // Spinlock for precision
            // Brief spinlock for final precision
            do {
                clock_gettime(CLOCK_MONOTONIC, &now);
            } while ((now.tv_sec < target.tv_sec) ||
                    (now.tv_sec == target.tv_sec && now.tv_nsec < target.tv_nsec));

            motor_control_thread_state = 9; // Updating target time
            // Update target time for next iteration
            lastTime = target;

            motor_control_thread_state = 10; // Tracking timing statistics
            // Track timing statistics (after the critical timing section)
            int64_t behind_ns = (now.tv_sec - target.tv_sec) * 1000000000LL +
                              (now.tv_nsec - target.tv_nsec);
            if (behind_ns > max_behind_ns) {
                max_behind_ns = behind_ns;
            }
        }
        
        motor_control_thread_state = 11; // Updating GPIO IDR
        // Update GPIO IDR based on BSRR (simulating hardware behavior)
        // For each port: clear reset bits, then set set bits
        GPIOA->IDR = (GPIOA->IDR & ~(GPIOA->BSRR >> 16)) | (GPIOA->BSRR & 0xFFFF);
        GPIOB->IDR = (GPIOB->IDR & ~(GPIOB->BSRR >> 16)) | (GPIOB->BSRR & 0xFFFF);
        GPIOC->IDR = (GPIOC->IDR & ~(GPIOC->BSRR >> 16)) | (GPIOC->BSRR & 0xFFFF);
        
        motor_control_thread_state = 12; // Clearing BSRR
        // Clear BSRR after processing (like real hardware)
        GPIOA->BSRR = 0;
        GPIOB->BSRR = 0;
        GPIOC->BSRR = 0;
        
        motor_control_thread_state = 13; // Simulating ADC hall sensor values
        simulate_ADC_hall_sensor_values();
        
        motor_control_thread_state = 14; // Checking interrupts enabled
        // Run motor control if interrupts are enabled and no reset in progress
        static int debug_counter = 0;
        if (debug_counter++ % 100000 == 0) {
            printf("DEBUG: g_interrupts_enabled = %d, gResetProgress = %d, (USART1->ISR & USART_ISR_RXNE_RXFNE) = %u\n",
                   g_interrupts_enabled, gResetProgress, USART1->ISR & USART_ISR_RXNE_RXFNE); // DEBUG - temporarily added to aid in debugging
            void print_nReceivedBytes(void);
            print_nReceivedBytes();
        }
        
        motor_control_thread_state = 15; // Running motor control
        if (g_interrupts_enabled && gResetProgress == 0) {
            motor_control_thread_state = 151; // Before TIM16_IRQHandler
            TIM16_IRQHandler();
            motor_control_thread_state = 152; // After TIM16_IRQHandler
        }
        else if (gResetProgress == 1) {
            // If we're waiting for TIM16_IRQHandler to exit, signal that we've exited
            // by incrementing gResetProgress to 2
            gResetProgress = 2;
            printf("Motor control thread: TIM16_IRQHandler exited, gResetProgress = %d\n", gResetProgress);
        }
        
        motor_control_thread_state = 16; // Updating shared state
        // Update shared state (after motor control)
        pthread_mutex_lock(&gSharedState.mutex);
        gSharedState.motor_position = get_motor_position();
        gSharedState.mosfets_enabled = is_mosfets_enabled();
        gSharedState.angle_deg = (double)gSharedState.motor_position * (360.0 / COUNTS_PER_ROTATION);
        pthread_mutex_unlock(&gSharedState.mutex);
        
        motor_control_thread_state = 17; // Incrementing call count
        callCount++;
        motor_control_thread_state = 18; // Checking if time to print stats
        // Print stats every 60 seconds (1 minute)
        if (lastStatsTime.tv_sec == 0 ||
            now.tv_sec - lastStatsTime.tv_sec >= 60) {
            motor_control_thread_state = 19; // Printing stats
            if (lastStatsTime.tv_sec != 0) {
                double elapsed = (now.tv_sec - lastStatsTime.tv_sec) +
                               (now.tv_nsec - lastStatsTime.tv_nsec) / 1e9;
                double avgFreq = callCount / elapsed;
                printf("Motor timing: %.2f Hz (target: 31250 Hz), max deviation: %.2f μs\n",
                       avgFreq, max_behind_ns / 1000.0);
                max_behind_ns = 0; // Reset for next period
            }
            callCount = 0;
            lastStatsTime = now;
        }
        
        motor_control_thread_state = 20; // End of loop
    }
    
    motor_control_thread_state = 21; // Thread exiting
    return NULL;
}

// Main thread visualization update at 60fps
void motor_simulator_visualization(void)
{
    static struct timespec lastTime = {0, 0};
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    
    // Update at ~60fps (every 16.67ms)
    if (lastTime.tv_sec == 0 || 
        (now.tv_sec - lastTime.tv_sec) * 1000000000ULL + 
        (now.tv_nsec - lastTime.tv_nsec) >= 16670000) {
        
        // Get latest motor state
        pthread_mutex_lock(&gSharedState.mutex);
        gMosfetsEnabled = gSharedState.mosfets_enabled;
        MotorHAL_SetPosition(gSharedState.angle_deg);
        pthread_mutex_unlock(&gSharedState.mutex);
        
        // Update visualization
        update_visualization();
        lastTime = now;
    }
}

// Initialize simulator environment
void motor_simulator_init(void)
{
    signal(SIGINT, handle_sigint);

    // Initialize HAL
    MotorHAL_StatusTypeDef status = MotorHAL_Init();
    if (status != MOTOR_HAL_OK) {
        fprintf(stderr, "Failed to initialize motor HAL\n");
        exit(1);
    }

    // Initialize USART1 for RS485
    USART1->BRR = 278; // 230400 baud @ 64MHz
    USART1->CR1 = USART_CR1_FIFOEN | USART_CR1_RXNEIE_RXFNEIE | 
                  USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    USART1->CR2 = USART_CR2_RTOEN;
    USART1->RTOR = ((230400 / 10) << USART_RTOR_RTO_Pos); // 0.1s timeout
    USART1->CR3 = USART_CR3_DEM | USART_CR3_EIE;

    // Call rs485_init() to set up any additional state
    rs485_init();

    // Initialize SDL on main thread
    init_sdl();

    // Create PTY
    int slaveFD = -1;
    if (openpty(&gMasterFD, &slaveFD, NULL, NULL, NULL) == -1) {
        perror("openpty");
        exit(1);
    }
    char *slaveName = ttyname(slaveFD);
    if (!slaveName) {
        perror("ttyname");
        close(gMasterFD);
        close(slaveFD);
        exit(1);
    }
    printf("\nMotor Simulator started.\nUse this serial port in your program:\n  %s\n\n", slaveName);

    // Configure terminal settings
    printf("Configuring terminal settings...\n");
    
    // Make both FDs non-blocking
    int flags = fcntl(gMasterFD, F_GETFL, 0);
    fcntl(gMasterFD, F_SETFL, flags | O_NONBLOCK);
    flags = fcntl(slaveFD, F_GETFL, 0);
    fcntl(slaveFD, F_SETFL, flags | O_NONBLOCK);

    // Configure both FDs
    struct termios tios;
    memset(&tios, 0, sizeof(tios));
    
    // Raw mode, no echo, no signals
    cfmakeraw(&tios);
    
    // 230400 baud, 8N1
    cfsetspeed(&tios, B230400);
    tios.c_cflag |= CS8;
    
    // No flow control
    tios.c_cflag &= ~(CRTSCTS);
    tios.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    // Configure master side
    if (tcsetattr(gMasterFD, TCSANOW, &tios) == -1) {
        perror("tcsetattr master failed");
        exit(1);
    }
    
    // Configure slave side
    if (tcsetattr(slaveFD, TCSANOW, &tios) == -1) {
        perror("tcsetattr slave failed");
        exit(1);
    }
    
    printf("Terminal settings configured (baud: 230400, 8N1, raw mode)\n");

    // Start read, write, motor control, and SysTick threads
    pthread_create(&gReadThread, NULL, read_thread_func, NULL);
    pthread_detach(gReadThread);
    pthread_create(&gWriteThread, NULL, write_thread_func, NULL);
    pthread_detach(gWriteThread);
    pthread_create(&gMotorThread, NULL, motor_control_thread_func, NULL);
    pthread_detach(gMotorThread);
    pthread_create(&gSysTickThread, NULL, systick_thread_func, NULL);
    pthread_detach(gSysTickThread);
}

int main() {
    motor_simulator_init();
    while(1) {        
        main_simulation();
        // If a reset was requested, wait for gResetProgress to reach 2
        // This ensures that both fatal_error and TIM16_IRQHandler have exited
        printf("Reset requested, waiting for gResetProgress to reach 2 (current: %d)\n", gResetProgress);        
        while (gResetProgress < 2) { // Wait for gResetProgress to reach 2
            usleep(1000); // Sleep for 1ms to avoid busy waiting
        }
        printf("gResetProgress reached %d, proceeding with reset\n", gResetProgress);
        g_interrupts_enabled = 0; // Disable interrupts at each system reset        
        // Reset all modules that need to be reinitialized on system reset
        reset_all_modules();
        gResetProgress = 0; // Since we are done the reset now, we can start getting back to a non-recent state
        printf("The system was reset. About to call main_simulation() again\n");
    }
    return 0;
}
