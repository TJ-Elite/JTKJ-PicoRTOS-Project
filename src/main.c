#include <ctype.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"

// Exercise 4. Include the libraries necessaries to use the USB-serial-debug, and TinyUSB.

#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX 1

// Time related definitions.
#define NO_TIME 0U
#define DEBOUNCE_MS 50U
#define INTEGRATION_TIME_MS 100U
#define LONG_PRESS_MS 800U // Long press for button 2.
#define DOT_MAX_MS 250U    // Presses on button 1, that are shorter than this, are dots.
#define CHAR_PAUSE_MS 600U // If the pause between presses for button 1 are longer than this morseState changes to CHARACTER_DONE.
#define WORD_PAUSE_MS 1200 // If the pause between presses for button 1 are longer than this morseState changes to WORD_DONE.

// Light sensor definitions.
// Values below DASH_LUX are dots, values between DASH_LUX and SPACE_LUX are dashes.
// Values above SPACE_LUX but not NO_LIGHT_VALUE are spaces.
#define DASH_LUX 100U
#define SPACE_LUX 1000U
#define NO_LIGHT_VALUE UINT32_MAX
#define MAX_LIGHT_VALUE (UINT32_MAX - 1U)

// Other definitions.
#define MORSE_TABLE_SIZE (sizeof(morseTable) / sizeof(morseTable[0]))

// Function prototypes.
static void gpio_callback(uint gpio, uint32_t events);

static void input_task(void *arg);
static void logic_task(void *arg);
static void print_task(void *arg);

static void interpretAccelerometer();
static void interpretLight();
static const char *charToMorse(char c);
static char morseToChar(const char *morse);

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
    char symbol;        // Decoded symbol ('.' or '-')
    uint32_t timestamp; // The time, when the symbol was processed.
} morse_event_t;

typedef struct
{
    const char character;
    const char *morseCode;
} MorsePair;

// Constants
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

// Global variables.
MorseState morseState = IDLE;
InputMode inputMode = ACCEL_INPUT;

static uint32_t ambientLight = NO_LIGHT_VALUE;
char outputBuffer[128];
uint8_t bufferIndex = 0;

static volatile bool b1_pressed = false;
static volatile bool b2_s_pressed = false; // Short press for button 2
static volatile bool b2_l_pressed = false; // Long press for button 2
static volatile TickType_t b1_press_time = NO_TIME;
static volatile TickType_t b2_press_time = NO_TIME;
static volatile morse_event_t latest_event = {'\0', 0};

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
            }
        }
        else if (events & GPIO_IRQ_EDGE_RISE)
        {
            TickType_t press_duration = time_now - b1_press_time;

            if (press_duration > pdMS_TO_TICKS(DEBOUNCE_MS))
            {
                // defines if the symbol is a '.' or a '-' depending on the press time
                if (press_duration < pdMS_TO_TICKS(DOT_MAX_MS))
                {
                    latest_event.symbol = '.';
                }
                else
                {
                    latest_event.symbol = '-';
                }
                b1_pressed = true;
                latest_event.timestamp = time_now;
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
            }
        }
        else if (events & GPIO_IRQ_EDGE_RISE)
        {
            TickType_t press_duration = time_now - b2_press_time;

            if (press_duration > pdMS_TO_TICKS(DEBOUNCE_MS))
            {
                // Determines if button 2 was pressed for a short or a long time.
                if (press_duration < pdMS_TO_TICKS(LONG_PRESS_MS))
                {
                    b2_s_pressed = true;
                }
                else
                {
                    b2_l_pressed = true;
                }
            }
        }
    }
}

static void input_task(void *arg)
{
    (void)arg;

    init_veml6030(); // Initialize the light sensor.
    // TODO: Initialize the accelerometer.

    while (1)
    {
        if (ACCEL_INPUT == inputMode)
        {
            // TODO: Read the accelerometer values and store them.
            // We may want to average the values over a few samples to reduce noise.
        }
        else
        {
            // Get reading from the light sensor.
            ambientLight = veml6030_read_light();
        }

        // Do not remove this.
        vTaskDelay(pdMS_TO_TICKS(INTEGRATION_TIME_MS));
    }
}

static void logic_task(void *arg)
{
    (void)arg;
    // The tick the last action was done in.
    TickType_t last_action_tick = xTaskGetTickCount();

    while (1)
    {
        TickType_t current_tick = xTaskGetTickCount();

        // Button 1 records the Morse input from the user.
        if (b1_pressed)
        {
            b1_pressed != b1_pressed;

            // Try to interpret the latest sensor measurement as a dot, dash or space.
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

            char symbol = latest_event.symbol;
            last_action_tick = current_tick;

            switch (morseState)
            {
            case IDLE:
                // If dot or dash put in outputBuffer.
                if (symbol == '.' || symbol == '-')
                {
                    // Checks that the index doesn't go over the size of the array.
                    if (bufferIndex < 127)
                    {
                        outputBuffer[bufferIndex++] = symbol;
                    }
                    morseState = READING_CHARACTER;
                }
                break;

            case READING_CHARACTER:
                // Checks that the index doesn't go over the size of the array.
                if (bufferIndex < 127)
                {
                    outputBuffer[bufferIndex++] = symbol;
                }
                morseState = READING_CHARACTER;
                break;

            case CHARACTER_DONE:
                // Checks that the index doesn't go over the size of the array.
                if (bufferIndex < 127)
                {
                    outputBuffer[bufferIndex++] = symbol;
                }
                morseState = READING_CHARACTER;
                break;

            case WORD_DONE:
                // Checks that the index doesn't go over the size of the array.
                if (bufferIndex < 126)
                {
                    outputBuffer[bufferIndex++] = ' ';
                    outputBuffer[bufferIndex++] = symbol;
                }
                morseState = READING_CHARACTER;
                break;
            default:
                break;
            }
        }
        // If there's a small pause between the symbols, morseState changes to CHARACTER_DONE.
        else if (morseState == READING_CHARACTER && (current_tick - last_action_tick > pdMS_TO_TICKS(CHAR_PAUSE_MS)))
        {
            morseState = CHARACTER_DONE;
        }
        // If the pause goes longer adds a space to the buffer and morseState changes to WORD_DONE.
        else if (morseState == CHARACTER_DONE && (current_tick - last_action_tick > pdMS_TO_TICKS(WORD_PAUSE_MS)))
        {
            if (bufferIndex < 127 && bufferIndex > 0 && outputBuffer[bufferIndex - 1] != ' ')
            {
                outputBuffer[bufferIndex++] = ' ';
            }
            morseState = WORD_DONE;
        }

        // Button 2 short press: switches between the two input modes.
        if (b2_s_pressed)
        {
            b2_s_pressed = false;

            if (ACCEL_INPUT == inputMode)
            {
                inputMode = LIGHT_INPUT;
                rgb_led_write(0, 0, 255); // Set the RGB LED to blue to indicate light sensor input mode.
            }
            else
            {
                inputMode = ACCEL_INPUT;
                rgb_led_write(0, 255, 0); // Set the RGB LED to green to indicate accelerometer input mode.
            }
        }

        // Button 2 long press: inserts a newline and ends the message if there are two spaces before it.
        if (b2_l_pressed)
        {
            b2_l_pressed = false;

            // Checks if the buffer has at least two symbols and if they are spaces.
            if (bufferIndex >= 2 && outputBuffer[bufferIndex - 1] == ' ' && outputBuffer[bufferIndex - 2] == ' ')
            {
                if (bufferIndex < 127)
                {
                    outputBuffer[bufferIndex++] = '\n';
                }

                // Message ends.
                bufferIndex = 0;
                morseState = IDLE;
            }
        }

        // Do not remove this.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void print_task(void *arg)
{
    (void)arg;

    while (1)
    {
        // Exercise 4. Use usb_serial_print() instead of printf() for debug output
        //             produced after the USB serial library has been initialized.
        //             Do not replace error messages that can execute before usb_serial_init().
        //             Use the TinyUSB library to send data through the other serial port (CDC 1).
        //             You can use the functions at https://github.com/hathach/tinyusb/blob/master/src/class/cdc/cdc_device.h
        //             You can find an example at hello_dual_cdc
        //             The data written using this should be provided using CSV format:
        //             timestamp, illuminance

        // Do not remove this.
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Exercise 4: Uncomment the following line to activate the TinyUSB library.

/*
static void usbTask(void *arg) {
    (void)arg;
    while (1) {
        tud_task();              // With FreeRTOS wait for events
                                 // Do not add vTaskDelay.
    }
}*/

static void interpretAccelerometer()
{
    // TODO
}

static void interpretLight()
{
    if (ambientLight != NO_LIGHT_VALUE)
    {
        return;
    }

    if (ambientLight < DASH_LUX)
    {
        latest_event.symbol = '.';
    }
    else if (ambientLight < SPACE_LUX)
    {
        latest_event.symbol = '-';
    }
    else
    {
        latest_event.symbol = ' ';
    }

    latest_event.timestamp = xTaskGetTickCount();
}

static const char *charToMorse(char c)
{
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

int main()
{

    // Exercise 4: Comment the statement stdio_init_all();
    //             Instead, add AT THE END OF MAIN (before vTaskStartScheduler();) adequate statements to enable the TinyUSB library and the usb-serial-debug.
    //             You can see hello_dual_cdc for help
    //             In CMakeLists.txt add usb_serial_debug to target_link_libraries.
    //             In CMakeLists.txt deactivate pico_enable_stdio_usb

    stdio_init_all();

    // Uncomment this lines if you want to wait till the serial monitor is connected
    /*while (!stdio_usb_connected()){
        sleep_ms(10);
    }*/

    init_hat_sdk();
    sleep_ms(300); // Wait some time so initialization of USB and hat is done.

    // Initialize the buttons and LEDs.
    init_sw1();
    init_sw2();
    init_red_led();
    init_rgb_led();
    rgb_led_write(0, 255, 0); // Set the RGB LED initially to green to indicate accelerometer input mode.

    // Register the generic callback.
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, gpio_callback);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);

    TaskHandle_t hButtonTask, hInputTask, hLogicTask, hPrintTask, hUSB = NULL;

    // Exercise 4: Uncomment this xTaskCreate to create the task that enables dual USB communication.

    /*
    xTaskCreate(usbTask, "usb", 2048, NULL, 3, &hUSB);
    #if (configNUMBER_OF_CORES > 1)
        vTaskCoreAffinitySet(hUSB, 1u << 0);
    #endif
    */

    // Create the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(input_task,         // (en) Task function
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

    result = xTaskCreate(print_task,         // (en) Task function
                         "print",            // (en) Name of the task
                         DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                         NULL,               // (en) Arguments of the task
                         2,                  // (en) Priority of this task
                         &hPrintTask);       // (en) A handle to control the execution of this task

    if (result != pdPASS)
    {
        printf("Print task creation failed.\n");
        return 0;
    }

    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}
