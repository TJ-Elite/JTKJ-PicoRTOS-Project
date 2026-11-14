
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

// Light sensor definitions.
#define NO_LIGHT_VALUE UINT32_MAX
#define MAX_LIGHT_VALUE (UINT32_MAX - 1U)
#define DASH_LUX 100U
#define SPACE_LUX 1000U

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

// Global variables.
MorseState morseState = IDLE;
InputMode inputMode = ACCEL_INPUT;

static uint32_t ambientLight = NO_LIGHT_VALUE;
char outputBuffer[128];
uint8_t bufferIndex = 0;

volatile bool b1_pressed = false;
volatile bool b2_pressed = false;
volatile TickType_t b1_press_time = NO_TIME;
volatile TickType_t b2_press_time = NO_TIME;

static void gpio_callback(uint gpio, uint32_t events)
{
    // Only process falling edge events.
    if (!(events & GPIO_IRQ_EDGE_FALL))
    {
        return;
    }

    TickType_t time_now = xTaskGetTickCountFromISR();

    if (BUTTON1 == gpio)
    {
        // Only accept new button presses after the debounce time has elapsed.
        if (time_now - b1_press_time > pdMS_TO_TICKS(DEBOUNCE_MS))
        {
            b1_pressed = true;
            b1_press_time = time_now;
        }
    }
    else if (BUTTON2 == gpio)
    {
        // Only accept new button presses after the debounce time has elapsed.
        if (time_now - b2_press_time > pdMS_TO_TICKS(DEBOUNCE_MS))
        {
            b2_pressed = true;
            b2_press_time = time_now;
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

    while (1)
    {
        // Button 1 records the Morse input from the user.
        if (b1_pressed)
        {
            b1_pressed = false;
            // TODO: Record the Morse input. Different cases for accelerometer and light sensor input modes.
            // Processing the Morse symbol

            char symbol; // Initializing the detected symbol here, when there is a way to detected it.
            
            switch (morseState) {
                case IDLE:
                    // If dot or dash put in outputBuffer.
                    if (symbol == '.' || symbol == '-') {
                        // Checks that the index doesn't go over the size of the array.
                        if (bufferIndex < 127) {
                            outputBuffer[bufferIndex++] = symbol;
                        }
                        morseState = READING_CHARACTER;
                    }
                    break;
            
                case READING_CHARACTER:
                    // If dot or dash put in outputBuffer.
                    if (symbol == '.' || symbol == '-') {
                        // Checks that the index doesn't go over the size of the array.
                        if (bufferIndex < 127) {
                            outputBuffer[bufferIndex++] = symbol;
                        }
                    }
                    // If space decode, store and clear buffer.
                    else if (symbol == ' ') {
                        morseState = CHARACTER_DONE;
                    }
                    break;

                case CHARACTER_DONE:
                    // If dot or dash put in outputBuffer.
                    if (symbol == '.' || symbol == '-') {
                        // Checks that the index doesn't go over the size of the array.
                        if (bufferIndex < 127) {
                            outputBuffer[bufferIndex++] = symbol;
                        }
                        morseState = READING_CHARACTER;
                    }
                    // If space word is done.
                    else if (symbol == ' ') {
                        morseState = WORD_DONE;
                    }
                    break;

                case WORD_DONE:
                    // If dot or dash put in outputBuffer and store space.
                    if (symbol == '.' || symbol == '-') {
                        // Checks that the index doesn't go over the size of the array.
                        if (bufferIndex < 126) {
                            outputBuffer[bufferIndex++] = ' ';
                            outputBuffer[bufferIndex++] = symbol;
                        }
                        morseState = READING_CHARACTER;
                    }
                    //If space form and send message.
                    else if (symbol == ' ') {
                    bufferIndex = 0;
                    morseState = IDLE;
                    }
                    break;
            }
        }

        // Button 2 switches between the two input modes.
        if (b2_pressed)
        {
            b2_pressed = false;

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

int main()
{

    // Exercise 4: Comment the statement stdio_init_all();
    //             Instead, add AT THE END OF MAIN (before vTaskStartScheduler();) adequate statements to enable the TinyUSB library and the usb-serial-debug.
    //             You can see hello_dual_cdc for help
    //             In CMakeLists.txt add usb_serial_debug to target_link_libraries.
    //             In CMakeLists.txt deactivate pico_enable_stdio_usb
    // Tehtävä 4:  Kommentoi lause stdio_init_all();
    //             Sen sijaan lisää MAIN LOPPUUN (ennen vTaskStartScheduler();) tarvittavat komennot aktivoidaksesi TinyUSB-kirjaston ja usb-serial-debugin.
    //             Voit katsoa apua esimerkistä hello_dual_cdc.
    //             Lisää CMakeLists.txt-tiedoston target_link_libraries-kohtaan usb_serial_debug.
    //             Poista CMakeLists.txt-tiedostosta käytöstä pico_enable_stdio_usb

    stdio_init_all();

    // Uncomment this lines if you want to wait till the serial monitor is connected
    /*while (!stdio_usb_connected()){
        sleep_ms(10);
    }*/

    init_hat_sdk();
    sleep_ms(300); // Wait some time so initialization of USB and hat is done.

    // Initialize the buttons and LEDs.
    gpio_init(BUTTON1);
    gpio_set_dir(BUTTON1, GPIO_IN);

    gpio_init(BUTTON2);
    gpio_set_dir(BUTTON2, GPIO_IN);

    gpio_init(RED_LED_PIN);
    gpio_set_dir(RED_LED_PIN, GPIO_OUT);

    init_rgb_led();
    rgb_led_write(0, 255, 0); // Set the RGB LED initially to green to indicate accelerometer input mode.

    // Register the generic callback.
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_FALL, true, gpio_callback);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_FALL, true);

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
