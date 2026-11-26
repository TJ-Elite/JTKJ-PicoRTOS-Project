// Names: Jimmy Last, Tuukka Junnikkala and Topias Saarela

#include <ctype.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include <tusb.h>
#include "usbSerialDebug/helper.h"

#include "tkjhat/sdk.h"

// Default stack size for the tasks. It can be reduced to 1024 if task is not using lot of memory.
#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX 1

// Time related definitions.
#define NO_TIME 0U
#define DEBOUNCE_MS 50U
#define INTEGRATION_TIME_MS 100U
#define DEBUG_PRINT_INTERVAL_MS 1000U // How often the sensor debug line is repeated while the symbol stays the same.
#define HOLD_TO_SEND_MS 2000U      // Duration button 2 must be pressed to insert a newline and send.

// Accelerometer definitions.
#define NO_ACCEL_VALUE (-100.0f)
// Normalized accelerometer threshold value. Should be in the range 0.5 < ACCEL_THRESHOLD < 1.0.
#define ACCEL_THRESHOLD 0.8f

// Light sensor definitions.
// Values below DASH_LUX are dots, values between DASH_LUX and SPACE_LUX are dashes.
// Values above SPACE_LUX but not NO_LIGHT_VALUE are spaces.
#define DASH_LUX 100U
#define SPACE_LUX 1000U
#define NO_LIGHT_VALUE UINT32_MAX
#define MAX_LIGHT_VALUE (UINT32_MAX - 1U)

// Buzzer definitions.
#define BUZZER_FREQ_LOW_HZ 4000U
#define BUZZER_FREQ_HIGH_HZ ((BUZZER_FREQ_LOW_HZ) + 1000U)
#define BUZZER_CONFIRM_DURATION_MS 300U
#define BUZZER_FAIL_DURATION_MS ((BUZZER_CONFIRM_DURATION_MS) / 2U)

// LED definitions.
#define LED_OFF (false)
#define LED_ON (!(LED_OFF))
// Note that all durations should have a unique length, because we take advantage of that.
#define DOT_DURATION_MS 150U
#define DASH_DURATION_MS (3U * DOT_DURATION_MS)
#define MORSE_SPACE_MS (DOT_DURATION_MS + 1U)
#define CHAR_SPACE_MS (3U * DOT_DURATION_MS + 1U)
#define WORD_SPACE_MS (7U * DOT_DURATION_MS)

// Display definitions. Dimensions are in pixels at font scaling 1.
// https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
#define DISPLAY_WIDTH 128U
#define DISPLAY_HEIGHT 64U

#define CHAR_WIDTH 5U
#define CHAR_SPACING 1U
#define CHARS_PER_LINE ((DISPLAY_WIDTH) / ((CHAR_WIDTH) + (CHAR_SPACING)))

#define CHAR_HEIGHT 8U
#define LINE_SPACING 1U
#define LINE_HEIGHT ((CHAR_HEIGHT) + (LINE_SPACING))
#define MAX_LINES ((DISPLAY_HEIGHT) / (LINE_HEIGHT))

#define MAX_RECEIVED_CHARS ((CHARS_PER_LINE) * (MAX_LINES))
// Also reserves space for '\0'.
#define RECEIVED_TEXT_SIZE ((MAX_RECEIVED_CHARS) + 1)
// Reserves enough space for the maximum number of characters the screen can fit at once expressed in Morse code,
// including the spaces between characters. Also reserves space for the final '\0' even in the worst case.
#define INPUT_BUFFER_SIZE ((MAX_RECEIVED_CHARS) * ((MAX_MORSE_LENGTH) + 1))

#define LINE_START_X ((int16_t)0)

// Morse code definitions.
#define DOT '.'
#define DASH '-'
#define SPACE ' '
#define INVALID_MORSE '?'

#define MORSE_TABLE_SIZE (sizeof(morseTable) / sizeof(morseTable[0]))
#define MAX_MORSE_LENGTH 5
#define SYMBOL_BUFFER_SIZE ((MAX_MORSE_LENGTH) + 1)
#define MORSE_BUFFER_SIZE 8
#define OUTPUT_BUFFER_SIZE 128
#define OUTPUT_BUFFER_MAX_INDEX ((OUTPUT_BUFFER_SIZE) - 1)

#if CFG_TUSB_OS != OPT_OS_FREERTOS
#error "This should be using FREERTOS but the CFG_TUSB_OS is not OPT_OS_FREERTOS"
#endif

typedef enum accelerometerStatus
{
    ICM42670_NOT_INITIALIZED = 1,
    ICM42670_INITIALIZED,
    ACCEL_INITIALIZED
} AccelerometerStatus;

typedef enum morseState
{
    IDLE = 1,
    READING_CHARACTER,
    CHARACTER_DONE,
    WORD_DONE
} MorseState;

typedef enum inputMode
{
    ACCEL_INPUT = 0,
    LIGHT_INPUT
} InputMode;

// Struct for morse-events.
typedef struct
{
    char symbol;        // Decoded symbol (dot, dash or space)
    uint32_t timestamp; // The time, when the symbol was processed.
} morse_event_t;

typedef struct
{
    const char character;
    const char *morseCode;
} MorsePair;

// Function prototypes.
static void gpio_callback(uint gpio, uint32_t events);

static void comms_task(void *arg);
static void input_task(void *arg);
static void logic_task(void *arg);
static void ui_task(void *arg);
static void usbTask(void *arg);

static inline void toggleInputMode();
static inline void interpretInput();
static int initAccelerometer();
static bool readAccelerometer(float *x, float *y, float *z);
static char classifyAccel(float x, float y, float z);
static char classifyLight(uint32_t lux);
static const char *symbolName(char symbol);
static void printSensorDebug(InputMode mode, float x, float y, float z, uint32_t lux);
static void printMorseDebug(void);
static void interpretAccelerometer();
static inline void clearAccelReadings();
static void interpretLight();
static inline void resetMorseStateMachine();
static void printToScreen(const char *text);
static int sendViaUsb();
static const char *charToMorse(char c);
static char morseToChar(const char *morse);
static void decodeMessage(const char *message);
static void handleMorsePlayback();
static bool handlePlaybackTiming();
static bool loadNextPlaybackSymbol();
static inline TickType_t getMorseDuration(const char c, bool *const ledState);
static inline void handleMorsePlaybackEnd();
static inline void resetMorsePlayback();

// Constants
static const char space_string[] = " ";

static const MorsePair morseTable[] = {
    {'A', ".-"},
    {'B', "-..."},
    {'C', "-.-."},
    {'D', "-.."},
    {'E', "."},
    {'F', "..-."},
    {'G', "--."},
    {'H', "...."},
    {'I', ".."},
    {'J', ".---"},
    {'K', "-.-"},
    {'L', ".-.."},
    {'M', "--"},
    {'N', "-."},
    {'O', "---"},
    {'P', ".--."},
    {'Q', "--.-"},
    {'R', ".-."},
    {'S', "..."},
    {'T', "-"},
    {'U', "..-"},
    {'V', "...-"},
    {'W', ".--"},
    {'X', "-..-"},
    {'Y', "-.--"},
    {'Z', "--.."},
    {'0', "-----"},
    {'1', ".----"},
    {'2', "..---"},
    {'3', "...--"},
    {'4', "....-"},
    {'5', "....."},
    {'6', "-...."},
    {'7', "--..."},
    {'8', "---.."},
    {'9', "----."}};

// Global variables
AccelerometerStatus accelStatus = ICM42670_NOT_INITIALIZED;
MorseState morseState = IDLE;
InputMode inputMode = ACCEL_INPUT;

static float accel_x = NO_ACCEL_VALUE;
static float accel_y = NO_ACCEL_VALUE;
static float accel_z = NO_ACCEL_VALUE;
static uint32_t ambientLight = NO_LIGHT_VALUE;
static bool newTextToPrint = false;
static bool playbackInProgress = false;
static uint16_t playbackIndex = 0U;       // Index of the received character we are playing back.
static const char *playbackSymbol = NULL; // Morse code for the symbol we are playing back.
static uint8_t playbackSymbolIndex = 0U;  // Index within the Morse symbol.
static TickType_t ledPhaseStartTime = NO_TIME;
static TickType_t ledPhaseDuration = NO_TIME;
static char receivedText[RECEIVED_TEXT_SIZE] = "";
static char outputBuffer[OUTPUT_BUFFER_SIZE] = "";
static char current_morse[MORSE_BUFFER_SIZE] = ""; // Temporary buffer for one Morse symbol.

// Volatile
static volatile bool b1_pressed = false;
static volatile bool b2_pressed = false;
static volatile bool both_pressed = false; // Both buttons are pressed at the same time.
static volatile TickType_t b1_press_time = NO_TIME;
static volatile TickType_t b2_press_time = NO_TIME;
static volatile bool b1_down = false; // True if button 1 is being pressed down.
static volatile bool b2_down = false; // True if button 2 is being pressed down.
static volatile bool sendMessage = false;
static volatile morse_event_t latest_event = {'\0', 0};
static volatile uint8_t bufferIndex = 0;
static volatile uint8_t morse_index = 0; // Keeps track of the current position in the temporary buffer.

static void gpio_callback(uint gpio, uint32_t events)
{
    TickType_t time_now = xTaskGetTickCountFromISR();

    if (BUTTON1 == gpio)
    {
        if (events & GPIO_IRQ_EDGE_FALL)
        {
            // Only accept new button presses after the debounce time has elapsed.
            if (time_now - b1_press_time > pdMS_TO_TICKS(DEBOUNCE_MS))
            {
                b1_press_time = time_now;
                b1_down = true;
            }
        }
        else if (events & GPIO_IRQ_EDGE_RISE)
        {
            b1_down = false;
            TickType_t press_duration = time_now - b1_press_time;

            if (press_duration > pdMS_TO_TICKS(DEBOUNCE_MS))
            {
                b1_pressed = true;
            }
        }
    }
    else if (BUTTON2 == gpio)
    {
        if (events & GPIO_IRQ_EDGE_FALL)
        {
            // Only accept new button presses after the debounce time has elapsed.
            if (time_now - b2_press_time > pdMS_TO_TICKS(DEBOUNCE_MS))
            {
                b2_press_time = time_now;
                b2_down = true;
            }
        }
        else if (events & GPIO_IRQ_EDGE_RISE)
        {
            b2_down = false;
            TickType_t press_duration = time_now - b2_press_time;

            if (press_duration > pdMS_TO_TICKS(DEBOUNCE_MS))
            {
                // Checks if button 2 was pressed for a long enough time to do a different function.
                if (press_duration >= pdMS_TO_TICKS(HOLD_TO_SEND_MS))
                {
                    b2_long_press = true; // Long press: insert newline and send if ready.
                }
                else
                {
                    b2_pressed = true; // Short press: toggle input mode.
                }
            }
        }
    }
}

static void comms_task(void *arg)
{
    (void)arg;

    char line[INPUT_BUFFER_SIZE];
    size_t lineLen = 0;

    while (1)
    {
        // Check if data has been received from the computer via USB.
        if (tud_cdc_n_connected(CDC_ITF_TX) && tud_cdc_n_available(CDC_ITF_TX))
        {
            char c;
            while (tud_cdc_n_available(CDC_ITF_TX) && tud_cdc_n_read(CDC_ITF_TX, &c, 1) == 1)
            {
                if ('\n' == c)
                {
                    line[lineLen] = '\0';
                    decodeMessage(line);
                    lineLen = 0;
                }
                else if ('\r' != c && lineLen < sizeof(line) - 1)
                {
                    line[lineLen++] = c;
                }
            }
        }

        // If we have an unsent message, try sending it again via USB serial.
        if (sendMessage)
        {
            // Result is greater than zero when data was sent successfully.
            if (sendViaUsb() > 0)
            {
                resetMorseStateMachine();
            }
        }
        // Do not remove this.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void input_task(void *arg)
{
    (void)arg;

    init_veml6030();           // Initialize the light sensor.
    (void)initAccelerometer(); // Initialize the accelerometer.

    while (1)
    {
        if (ACCEL_INPUT == inputMode)
        {
            // Try to get a reading from the accelerometer and show it on CDC 0 with the symbol it means.
            float x, y, z;
            if (readAccelerometer(&x, &y, &z))
            {
                printSensorDebug(ACCEL_INPUT, x, y, z, 0);
            }
        }
        else
        {
            // Get a reading from the light sensor and show it on CDC 0 with the symbol it means.
            ambientLight = veml6030_read_light();
            printSensorDebug(LIGHT_INPUT, 0, 0, 0, ambientLight);
        }

        // Do not remove this.
        vTaskDelay(pdMS_TO_TICKS(INTEGRATION_TIME_MS));
    }
}

static void logic_task(void *arg)
{
    (void)arg;

    while (1)
    {
        // Button 2 long press: Inserts a newline if preceded by two spaces and tries to send the message.
        if (b2_long_press)
        {
            // Clear button press flags.
            b2_long_press = false;

            // '\n' can only be inserted and the message sent when preceded by two spaces,
            // i.e. morseState is WORD_DONE, and there is no pending message waiting to be sent.
            if (WORD_DONE == morseState && !sendMessage)
            {
                // Make sure there's room for the '\n' and '\0'.
                if (bufferIndex >= OUTPUT_BUFFER_MAX_INDEX)
                {
                    bufferIndex = OUTPUT_BUFFER_MAX_INDEX - 1;
                }
                outputBuffer[bufferIndex++] = '\n';
                outputBuffer[bufferIndex] = '\0';

                sendMessage = true; // Flag message ready for sending via USB serial.
            }
        }

        // Button 1 records the Morse input from the user.
        if (b1_pressed)
        {
            b1_pressed = false;

            // Clear the previous symbol and interpret the latest sensor measurement.
            latest_event.symbol = '\0';
            interpretInput();

            char symbol = latest_event.symbol;

            // The state changes of the Morse statemachine.
            switch (morseState)
            {
            case IDLE:
                // If dot or dash, put in the temporary string and morseState changes to READING_CHARACTER.
                if (symbol == DOT || symbol == DASH)
                {
                    if (morse_index < sizeof(current_morse) - 1)
                    {
                        current_morse[morse_index++] = symbol;
                        current_morse[morse_index] = '\0';
                        printMorseDebug();
                    }
                    morseState = READING_CHARACTER;
                }
                break;

            case READING_CHARACTER:
                // If dot or dash, put in the temporary string.
                if (symbol == DOT || symbol == DASH)
                {
                    if (morse_index < sizeof(current_morse) - 1)
                    {
                        current_morse[morse_index++] = symbol;
                        current_morse[morse_index] = '\0';
                        printMorseDebug();
                    }
                }
                // Space: Store Morse symbol and space in outputBuffer, if there's enough room and the symbol is valid.
                // morseState changes to CHARACTER_DONE.
                else if (symbol == SPACE)
                {
                    // The Morse symbol is only stored if it's actually a valid Morse symbol,
                    // there is no message currently being sent, and there's still enough
                    // room for it as well as two spaces, '\n' and '\0' after it.
                    const char c = morseToChar(current_morse);
                    const size_t morseLength = strlen(current_morse);

                    if (!sendMessage && '\0' != c && bufferIndex <= OUTPUT_BUFFER_SIZE - (morseLength + 4))
                    {
                        for (size_t i = 0; i < morseLength; ++i)
                        {
                            outputBuffer[bufferIndex++] = current_morse[i];
                        }
                        outputBuffer[bufferIndex++] = SPACE;
                        outputBuffer[bufferIndex] = '\0';
                    }
                    // Sound the buzzer at a higher pitch and for a shorter time than send success sound
                    // to signal that the Morse symbol was discarded.
                    else
                    {
                        // Note: This implementation is blocking.
                        buzzer_play_tone(BUZZER_FREQ_HIGH_HZ, BUZZER_FAIL_DURATION_MS);
                    }

                    current_morse[0] = '\0';
                    morse_index = 0;
                    morseState = CHARACTER_DONE;
                    printMorseDebug();
                }
                break;

            case CHARACTER_DONE:
                // If dot or dash put in the temporary string and morseState changes to READING_CHARACTER.
                if (symbol == DOT || symbol == DASH)
                {
                    if (morse_index < sizeof(current_morse) - 1)
                    {
                        current_morse[morse_index++] = symbol;
                        current_morse[morse_index] = '\0';
                        printMorseDebug();
                    }
                    morseState = READING_CHARACTER;
                }
                // If space, morseState changes to WORD_DONE.
                else if (symbol == SPACE)
                {
                    // Put the second space in the buffer as long as a message isn't currently being send and there's also room for '\n' and '\0' after.
                    if (!sendMessage && bufferIndex < OUTPUT_BUFFER_MAX_INDEX - 1)
                    {
                        outputBuffer[bufferIndex++] = SPACE;
                        outputBuffer[bufferIndex] = '\0';
                    }

                    morseState = WORD_DONE;
                    if (!sendMessage && bufferIndex > 0)
                    {
                        outputBuffer[bufferIndex++] = '\n';
                        outputBuffer[bufferIndex] = '\0';
                        sendMessage = true;
                    }
                    printMorseDebug();
                }
                break;

            case WORD_DONE:
                // If dot or dash, start a new word.
                // Do nothing, if we receive a space.
                if (symbol == DOT || symbol == DASH)
                {
                    if (!sendMessage && bufferIndex > 0 && outputBuffer[bufferIndex - 1] == SPACE)
                    {
                        // Delete the second space we put in the output buffer.
                        outputBuffer[--bufferIndex] = '\0';
                    }

                    if (morse_index < sizeof(current_morse) - 1)
                    {
                        current_morse[morse_index++] = symbol;
                        current_morse[morse_index] = '\0';
                        printMorseDebug();
                    }

                    morseState = READING_CHARACTER;
                    printMorseDebug();
                }
                break;

            default:
                break;
            }
        }

        // Button 2: switches between the two input modes.
        if (b2_pressed)
        {
            b2_pressed = false;
            toggleInputMode();
        }

        // Do not remove this.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * Handles printing received messages to the screen as well as playing them back
 * in Morse form using the red LED.
 */
static void ui_task(void *arg)
{
    (void)arg;

    init_display();

    while (1)
    {
        if (newTextToPrint)
        {
            newTextToPrint = !newTextToPrint;
            printToScreen(receivedText);

            // "Erase" the written string if Morse playback has already concluded.
            if (!playbackInProgress)
            {
                receivedText[0] = '\0';
            }
        }

        handleMorsePlayback(); // Better to be after printToScreen(), because that one can take a while.

        // Do not remove this.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void usbTask(void *arg)
{
    (void)arg;
    while (1)
    {
        tud_task(); // With FreeRTOS wait for events
                    // Do not add vTaskDelay.
    }
}

static inline void toggleInputMode()
{
    if (ACCEL_INPUT == inputMode)
    {
        ambientLight = NO_LIGHT_VALUE; // Clear old value.
        inputMode = LIGHT_INPUT;
        rgb_led_write(0, 0, 255); // Set the RGB LED to blue to indicate light sensor input mode.
    }
    else
    {
        clearAccelReadings(); // Clear old values.
        inputMode = ACCEL_INPUT;
        rgb_led_write(0, 255, 0); // Set the RGB LED to green to indicate accelerometer input mode.
    }
}

static inline void interpretInput()
{
    switch (inputMode)
    {
    case ACCEL_INPUT:
        interpretAccelerometer();
        break;
    case LIGHT_INPUT:
        interpretLight();
        break;
    default:
        break; // Do nothing.
    }
}

/**
 * This function checks whether the accelerometer has been initialized and is running.
 * If not, it tries to initialize it.
 * Returns 0 if the accelerometer is running.
 */
static int initAccelerometer()
{
    if (ICM42670_NOT_INITIALIZED == accelStatus)
    {
        // Try to initialize the ICM42670. Returns 0 on success.
        if (init_ICM42670())
        {
            return 1; // ICM42670 initialization failed.
        }
        accelStatus = ICM42670_INITIALIZED;
    }

    if (ICM42670_INITIALIZED == accelStatus)
    {
        // Try to start the accelerometer. Returns 0 on success.
        // The default values for the accelerometer are output data rate 100 Hz, and full-scale range: ±4 g.
        if (ICM42670_start_with_default_values())
        {
            return 2; // Failed to start the accelerometer.
        }
        accelStatus = ACCEL_INITIALIZED;
    }

    return 0; // Accelerometer is running.
}

/**
 * Reads the accelerometer and stores the values for interpretAccelerometer().
 * The values are also given to the caller through x, y and z (in g).
 * Returns true if the reading succeeded.
 */
static bool readAccelerometer(float *x, float *y, float *z)
{
    // First we check that the acceleremeter is running. If not, we try to initialize it.
    if (initAccelerometer())
    {
        return false; // Return if the accelerometer isn't running.
    }

    float ax = 0.0, ay = 0.0, az = 0.0, gx = 0.0, gy = 0.0, gz = 0.0, t = 0.0;

    // Read the sensor values. Returns 0 on success.
    if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t))
    {
        return false;
    }

    // Store the accelerometer values.
    accel_x = ax;
    accel_y = ay;
    accel_z = az;

    *x = ax;
    *y = ay;
    *z = az;
    return true;
}

/**
 * Returns the Morse symbol that matches the orientation of the device,
 * or '\0' if the orientation does not match any valid position.
 */
static char classifyAccel(float x, float y, float z)
{
    // Calculate the vector magnitude and scale the normalized threshold value with it.
    // This helps compensate for situations where the accelerometer readings are a bit off and
    // don't produce 1.0 g when laying perfectly flat as expected.
    float vector_magnitude = sqrtf(x * x + y * y + z * z);
    float threshold = ACCEL_THRESHOLD * vector_magnitude;

    // If one axis has a value above the threshold, it is the corresponding Morse input.
    if (fabsf(z) > threshold)
    {
        return DASH;
    }
    if (fabsf(x) > threshold)
    {
        return DOT;
    }
    if (fabsf(y) > threshold)
    {
        return SPACE;
    }
    return '\0';
}

/**
 * Returns the Morse symbol that matches the light level (in lux).
 */
static char classifyLight(uint32_t lux)
{
    if (lux < DASH_LUX)
    {
        return DOT;
    }
    if (lux < SPACE_LUX)
    {
        return DASH;
    }
    return SPACE;
}

/**
 * Name of a symbol for the debug output. '\0' means "no valid symbol".
 */
static const char *symbolName(char symbol)
{
    switch (symbol)
    {
    case DOT:
        return "DOT (.)";
    case DASH:
        return "DASH (-)";
    case SPACE:
        return "SPACE";
    default:
        return "NONE";
    }
}

/**
 * Prints the latest sensor value and the symbol it means to CDC 0 (usb_serial_print):
 *   ACCEL x=0.02 y=-0.01 z=0.99 -> DASH (-)
 *   LIGHT Lux:420 -> DASH (-)
 * A line is printed when the symbol or the mode changes, and otherwise at most
 * every DEBUG_PRINT_INTERVAL_MS. Nothing is printed if the serial port is not open.
 */
static void printSensorDebug(InputMode mode, float x, float y, float z, uint32_t lux)
{
    static char lastSymbol = 0; // 0 = nothing printed yet.
    static InputMode lastMode = ACCEL_INPUT;
    static TickType_t lastPrint = 0;

    char symbol = (LIGHT_INPUT == mode) ? classifyLight(lux) : classifyAccel(x, y, z);
    char shown = ('\0' == symbol) ? '?' : symbol; // '?' keeps "nothing printed yet" (0) distinguishable.
    TickType_t now = xTaskGetTickCount();

    if (shown == lastSymbol && mode == lastMode && now - lastPrint < pdMS_TO_TICKS(DEBUG_PRINT_INTERVAL_MS))
    {
        return; // Same symbol as last time and printed recently.
    }

    lastSymbol = shown;
    lastMode = mode;
    lastPrint = now;

    char line[80];
    if (LIGHT_INPUT == mode)
    {
        snprintf(line, sizeof(line), "LIGHT Lux:%lu -> %s\n", (unsigned long)lux, symbolName(symbol));
    }
    else
    {
        snprintf(line, sizeof(line), "ACCEL x=%.2f y=%.2f z=%.2f -> %s\n",
                 (double)x, (double)y, (double)z, symbolName(symbol));
    }
    usb_serial_print(line);
}

static void interpretAccelerometer()
{
    // Return if there are no accelerometer readings.
    if (NO_ACCEL_VALUE == accel_x || NO_ACCEL_VALUE == accel_y || NO_ACCEL_VALUE == accel_z)
    {
        return;
    }

    char symbol = classifyAccel(accel_x, accel_y, accel_z);

    // Clear the accelerometer readings to prevent processing the same values multiple times.
    clearAccelReadings();

    if ('\0' == symbol)
    {
        return; // The device orientation deviates too much from the valid positions.
    }

    latest_event.symbol = symbol;
    latest_event.timestamp = xTaskGetTickCount();
}

/**
 * Prints the current Morse message status to CDC 0.
 */
static void printMorseDebug(void)
{
    char line[256];

    snprintf(line, sizeof(line),
             "MORSE | Buffer: \"%s\" | Current: \"%s\" | State: %d\n",
             outputBuffer,
             current_morse,
             morseState);

    usb_serial_print(line);
}

static inline void clearAccelReadings()
{
    accel_x = NO_ACCEL_VALUE;
    accel_y = NO_ACCEL_VALUE;
    accel_z = NO_ACCEL_VALUE;
}

static void interpretLight()
{
    // Return if there is no light reading.
    if (NO_LIGHT_VALUE == ambientLight)
    {
        return;
    }

    // Interpret the light level as either a dot, dash or space.
    latest_event.symbol = classifyLight(ambientLight);

    latest_event.timestamp = xTaskGetTickCount();
    ambientLight = NO_LIGHT_VALUE; // Clear the light value to prevent processing the same value multiple times.
}

/**
 * Resets the Morse state machine. Note that sendMessage flag is also set to false.
 */
static inline void resetMorseStateMachine()
{
    sendMessage = false;
    morseState = IDLE;

    // Clear the buffers and their indexes.
    bufferIndex = 0;
    outputBuffer[0] = '\0';
    current_morse[0] = '\0';
    morse_index = 0;
}

/**
 * Prints text to the display. Handles wrapping long string to multiple lines. Clears prior content.
 */
static void printToScreen(const char *text)
{
    // Return if given a NULL pointer or the string is empty.
    if (!text || !(*text))
    {
        return;
    }

    // Clear the old content.
    clear_display();

    int16_t y = 0;
    while (*text && (uint32_t)y <= (uint32_t)((DISPLAY_HEIGHT) - (LINE_HEIGHT)))
    {
        char line[CHARS_PER_LINE + 1U];      // Takes into account the space needed for the termination character.
        strncpy(line, text, CHARS_PER_LINE); // If the copied string contains '\0', the code still works.
        line[CHARS_PER_LINE] = '\0';

        write_text_xy(LINE_START_X, y, line);

        text += strlen(line); // strlen() excludes the null terminator.
        y += LINE_HEIGHT;
    }
}

/**
 * Try to send the data in outputBuffer via USB.
 * Plays a confirmation tone if successful. Code execution is blocked while the buzzer is sounding.
 */
static int sendViaUsb()
{
    uint32_t bytesWritten = 0;

    usb_serial_print("Sending Morse message via CDC 1:\n");
    usb_serial_print(outputBuffer);
    usb_serial_print("\n");

    if (tud_cdc_n_connected(CDC_ITF_TX))
    {
        // Send message to the ACM1.
        bytesWritten = tud_cdc_n_write(CDC_ITF_TX, outputBuffer, strlen(outputBuffer));
        bytesWritten += tud_cdc_n_write_flush(CDC_ITF_TX);
    }

    // Result is greater than zero when data was sent successfully.
    if (bytesWritten > 0)
    {
        // Note: This implementation is blocking.
        buzzer_play_tone(BUZZER_FREQ_LOW_HZ, BUZZER_CONFIRM_DURATION_MS);
    }

    return bytesWritten;
}

static const char *charToMorse(char c)
{
    if (SPACE == c)
    {
        return space_string;
    }

    if (INVALID_MORSE == c)
    {
        return NULL;
    }

    c = toupper((unsigned char)c);

    for (size_t i = 0; i < MORSE_TABLE_SIZE; ++i)
    {
        if (morseTable[i].character == c)
        {
            return morseTable[i].morseCode;
        }
    }

    // Return NULL if character not found in the Morse table.
    return NULL;
}

static char morseToChar(const char *morse)
{
    for (size_t i = 0; i < MORSE_TABLE_SIZE; ++i)
    {
        if (strcmp(morseTable[i].morseCode, morse) == 0)
        {
            return morseTable[i].character;
        }
    }

    // Return '\0' if Morse code not found in the Morse table.
    return '\0';
}

/**
 * Tries to decode the given Morse message, storing decoded characters in receivedText[].
 * Any space-separated substrings that can't be decoded are stored as '?'.
 * newTextToPrint and playbackInProgress are set to true if at least one character was
 * decoded successfully.
 */
static void decodeMessage(const char *message)
{
    // Return if given a NULL pointer or the string is empty.
    if (!message || !(*message))
    {
        return;
    }

    char symbol[SYMBOL_BUFFER_SIZE]; // Reserves space for '\0' as well.
    int8_t symbolIndex = 0;
    int16_t textIndex = 0;

    while (*message)
    {
        if (SPACE == *message)
        {
            // End of a Morse character.
            if (symbolIndex > 0)
            {
                symbol[symbolIndex] = '\0';
                symbolIndex = 0; // This should be set to 0 BEFORE potentially breaking from the while loop.

                if (textIndex >= RECEIVED_TEXT_SIZE - 1)
                {
                    break; // receivedText[] is full.
                }

                char c = morseToChar(symbol);
                receivedText[textIndex++] = ('\0' != c) ? morseToChar(symbol) : INVALID_MORSE;
            }

            // Two spaces separate words.
            if (SPACE == *(message + 1))
            {
                if (textIndex >= RECEIVED_TEXT_SIZE - 1)
                {
                    break; // receivedText[] is full.
                }

                receivedText[textIndex++] = SPACE;

                // Skip all additional spaces.
                while (SPACE == *message)
                    ++message;
                continue; // This skips the additional ++message at the end of the loop.
            }
        }
        // Read the next char in message.
        else if (symbolIndex < SYMBOL_BUFFER_SIZE - 1)
        {
            // Ignore '\n' characters.
            if ('\n' != *message)
            {
                symbol[symbolIndex++] = *message;
            }
        }
        // The symbol couldn't be decoded because it's too long.
        else
        {
            symbolIndex = 0; // Prevents the processing step after the while loop.
            if (textIndex >= RECEIVED_TEXT_SIZE - 1)
            {
                break; // receivedText[] is full.
            }
            receivedText[textIndex++] = INVALID_MORSE;
        }

        ++message;
    }

    // Handle the last Morse character.
    if (symbolIndex > 0)
    {
        symbol[symbolIndex] = '\0';
        char c = morseToChar(symbol);
        if (textIndex < RECEIVED_TEXT_SIZE - 1)
        {
            receivedText[textIndex++] = ('\0' != c) ? morseToChar(symbol) : INVALID_MORSE;
        }
    }

    receivedText[textIndex] = '\0';
    if (textIndex > 0)
    {
        newTextToPrint = true;
        playbackInProgress = true;
        resetMorsePlayback();
    }
}

static void handleMorsePlayback()
{
    // Return if playback isn't in progress.
    if (!playbackInProgress)
    {
        return;
    }

    if (!handlePlaybackTiming())
    {
        // Return if we're waiting for the LED to have been on or off for a certain amount of
        // time, and not enough time has elapsed, or if another wait period is initiated by
        // handlePlaybackTiming().
        return;
    }

    // Locate the next valid playbackSymbol, if playback of a symbol isn't already in progress.
    // Playback ends if we've reached the end of the received message.
    if (!loadNextPlaybackSymbol())
    {
        return;
    }

    // Move on to the next character if we've reached the end of the current one's Morse code.
    if ('\0' == playbackSymbol[playbackSymbolIndex])
    {
        playbackSymbol = NULL;
        playbackSymbolIndex = 0U;
        ++playbackIndex;
        if (!loadNextPlaybackSymbol())
        {
            return;
        }
    }

    char c = playbackSymbol[playbackSymbolIndex++];

    // Determine whether the LED should be on or off and for how long.
    bool ledState;
    TickType_t duration = getMorseDuration(c, &ledState);

    // Return if the character wasn't a valid one.
    if (NO_TIME == duration)
    {
        return;
    }

    // Set the LED state and make note of when we started and for how long we should keep going.
    set_red_led_status(ledState);
    ledPhaseStartTime = xTaskGetTickCount();
    ledPhaseDuration = duration;
}

/**
 * Checks to see if the set wait time has elapsed. If it has and we just finished signaling a dot or a dash,
 * unless the this was the final dot or dash in this Morse symbol and the next symbol is a space,
 * turn the LED off and wait for the space between characters within the same Morse symbol.
 *
 * The function returns "true", if it's okay to continue to the next Morse playback phase,
 * otherwise "false".
 */
static bool handlePlaybackTiming()
{
    if (NO_TIME != ledPhaseDuration)
    {
        // Return false if current LED phase is still in progress.
        if (xTaskGetTickCount() - ledPhaseStartTime < ledPhaseDuration)
        {
            return false;
        }
        // If we just finished signaling a dot or a dash...
        else if (DOT_DURATION_MS == ledPhaseDuration || DASH_DURATION_MS == ledPhaseDuration)
        {
            // If it wasn't the last dot or dash within the Morse symbol, turn the LED off
            // for one dot duration.
            if ('\0' != playbackSymbol[playbackSymbolIndex])
            {
                set_red_led_status(LED_OFF);
                ledPhaseStartTime = xTaskGetTickCount();
                ledPhaseDuration = MORSE_SPACE_MS;

                return false;
            }
            // If we finished the Morse symbol and the next character isn't a space,
            // turn the LED off for three times the dot duration.
            else if (SPACE != receivedText[playbackIndex + 1])
            {
                set_red_led_status(LED_OFF);
                ledPhaseStartTime = xTaskGetTickCount();
                ledPhaseDuration = CHAR_SPACE_MS;

                return false;
            }
            // If the next character is a space, let the logic that follows handle that.
            else
            {
                ledPhaseStartTime = NO_TIME;
                ledPhaseDuration = NO_TIME;
            }
        }
        // Move on to the next phase.
        else
        {
            ledPhaseStartTime = NO_TIME;
            ledPhaseDuration = NO_TIME;
        }
    }

    return true;
}

/**
 * Tries to find the next character in the received message that has a valid Morse code.
 * Handles resetting Morse playback variables if the end of the message is reached and
 * returns false to signal to the caller that the message has ended.
 */
static bool loadNextPlaybackSymbol()
{
    // End playback if we've reached the end of the received text.
    if ('\0' == receivedText[playbackIndex])
    {
        handleMorsePlaybackEnd();
        return false;
    }

    // Acquire the next valid Morse symbol, if playback of a symbol isn't already in progress.
    while (NULL == playbackSymbol)
    {
        playbackSymbol = charToMorse(receivedText[playbackIndex]);
        playbackSymbolIndex = 0U;

        // If the read symbol isn't valid, advance playbackIndex by one.
        if (NULL == playbackSymbol)
        {
            ++playbackIndex;

            // End playback if we've reached the end of the received text.
            if ('\0' == receivedText[playbackIndex])
            {
                handleMorsePlaybackEnd();
                return false;
            }
        }
    }

    return true;
}

static inline TickType_t getMorseDuration(const char c, bool *const ledState)
{
    TickType_t duration;

    switch (c)
    {
    case DOT:
        duration = DOT_DURATION_MS;
        *ledState = LED_ON;
        break;
    case DASH:
        duration = DASH_DURATION_MS;
        *ledState = LED_ON;
        break;
    case SPACE:
        duration = WORD_SPACE_MS;
        *ledState = LED_OFF;
        break;
    default:
        duration = NO_TIME;
        *ledState = LED_OFF;
        break;
    }

    return pdMS_TO_TICKS(duration);
}

static inline void handleMorsePlaybackEnd()
{
    playbackInProgress = false;
    resetMorsePlayback();

    if (!newTextToPrint)
    {
        // "Erase" receivedText[] if it's already been printed to the screen.
        receivedText[0] = '\0';
    }
}

static inline void resetMorsePlayback()
{
    playbackIndex = 0;
    playbackSymbol = NULL;
    playbackSymbolIndex = 0;
    ledPhaseStartTime = NO_TIME;
    ledPhaseDuration = NO_TIME;
    set_red_led_status(LED_OFF);
}

int main()
{
    // Comment when using TinyUSB.
    // stdio_init_all();

    // Uncomment this lines if you want to wait till the serial monitor is connected
    /*while (!stdio_usb_connected()){
        sleep_ms(10);
    }*/

    init_hat_sdk();
    sleep_ms(300); // Wait some time so initialization of USB and hat is done.

    // Initialize the buttons, the buzzer and the LEDs.
    init_sw1();
    init_sw2();
    init_red_led();
    init_rgb_led();
    init_buzzer();
    rgb_led_write(0, 255, 0); // Set the RGB LED initially to green to indicate accelerometer input mode.

    // Register the generic callback.
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, gpio_callback);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);

    TaskHandle_t hCommsTask = NULL;
    TaskHandle_t hInputTask = NULL;
    TaskHandle_t hLogicTask = NULL;
    TaskHandle_t hUiTask = NULL;
    TaskHandle_t hUSB = NULL;

    xTaskCreate(usbTask, "usb", 2048, NULL, 3, &hUSB);
#if (configNUMBER_OF_CORES > 1)
    vTaskCoreAffinitySet(hUSB, 1u << 0);
#endif

    // Create the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(comms_task,         // (en) Task function
                                    "comms",            // (en) Name of the task
                                    DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                                    NULL,               // (en) Arguments of the task
                                    2,                  // (en) Priority of this task
                                    &hCommsTask);       // (en) A handle to control the execution of this task

    if (result != pdPASS)
    {
        printf("Comms task creation failed.\n");
        return 0;
    }

    result = xTaskCreate(input_task,         // (en) Task function
                         "input",            // (en) Name of the task
                         DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                         NULL,               // (en) Arguments of the task
                         2,                  // (en) Priority of this task
                         &hInputTask);       // (en) A handle to control the execution of this task

    if (result != pdPASS)
    {
        printf("Input task creation failed.\n");
        return 0;
    }

    result = xTaskCreate(logic_task,         // (en) Task function
                         "logic",            // (en) Name of the task
                         DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                         NULL,               // (en) Arguments of the task
                         2,                  // (en) Priority of this task
                         &hLogicTask);       // (en) A handle to control the execution of this task

    if (result != pdPASS)
    {
        printf("Logic task creation failed.\n");
        return 0;
    }

    result = xTaskCreate(ui_task,            // (en) Task function
                         "ui",               // (en) Name of the task
                         DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                         NULL,               // (en) Arguments of the task
                         2,                  // (en) Priority of this task
                         &hUiTask);          // (en) A handle to control the execution of this task

    if (result != pdPASS)
    {
        printf("UI task creation failed.\n");
        return 0;
    }

    // This should go just before vTaskStartSheduler without any delays.
    // Otherwise, the TinyUSB stack won't recognize the device.
    tusb_init();
    usb_serial_init();

    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}
