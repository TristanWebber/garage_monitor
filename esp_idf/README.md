# ESP-IDF Implementation

- [Introduction](#introduction)
- [Quickstart](#quickstart)
- [The Strategy](#the-strategy)
- [Getting Started](#getting-started)
- [The `main` File](#the-main-file)
- [The `peripheral_manager` Files](#the-peripheral_manager-files)
- [The `connection_manager` Files](#the-connection_manager-files)
- [Interrupts and debounce timers](#interrupts-and-debounce-timers)
- [Building and Flashing](#building-and-flashing)
- [Observations and Next Steps](#observations-and-next-steps)

## Introduction

This second example builds on the first. The project is fundamentally seeking to achieve the same outcomes, but this time using Espressif's ESP-IDF rather than Arduino. By using the ESP-IDF, some enhancements to function will be achievable with a marginal increase to the amount of effort and knowledge required.

A reminder on some of the focus areas identified at the conclusion of the simple arduino implementation:

- All of the items in the `loop()` function operated on a single thread, so a long running function risks blocking other functions from starting and executing
- The read and send functions were happening on a fixed interval, and were not configured to react to changes in value as they occurred. In particular, the door state.

As it happens, these are able to be controlled at a granular level when developing in the ESP-IDF. The ESP-IDF is Espressif's own hardware abstraction layer and it is based on FreeRTOS. FreeRTOS is a Realtime Operating System (RTOS) allowing for control of what, how and when different tasks run.

This walkthrough will first replicate the functionality we achieved in our previous project using FreeRTOS tasks, and then this will be improved upon by using task notifications. The concept of `task notifications` will be explored in the context of only asking the processor to perform a certain task if some pre-configured event occurs. For example, the configuration could be set up so that if the microcontroller detects that the reed switch changes state, immediately publish that state to the MQTT broker.

Of course, choosing to implement a project with ESP-IDF has different challenges to a project implemented using the Arduino framework, and some of those aspects will be explored. In particular, the use of the `idf.py` tool, `menuconfig` for achieving very fine control over how the microcontroller is configured, and `CMake` as the build system.

## Quickstart

This series of projects is presented as an educational piece, however if you simply want to take some or all of the repository and use it for your own projects without suffering through the narrative you can:

0. [Acquire and assemble](https://github.com/TristanWebber/garage_monitor/blob/main/README.md#hardware) the appropriate hardware
1. Setup a [dashboard](https://github.com/TristanWebber/garage_monitor/blob/main/README.md#dashboard)
1. Ensure you have the [ESP-IDF and toolchain](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html) installed
2. Clone the repository. If you are only interested in this subproject, use `sparse-checkout`
```bash
git clone --no-checkout https://github.com/TristanWebber/garage_monitor.git && \
cd garage_monitor && \
git sparse-checkout esp_idf && \
git checkout
```
3. [Update](#other-files) and rename the `_config.h` file
4. [Source](#mqtt) the `datacake_ca.pem` file and update the relevant CMakeLists.txt fils if necessary
5. [Build and flash](#building-and-flashing) the project to your microcontroller

## The strategy

A reminder on the objective. The project needs to:
- Take readings from a reed switch on a door
- Take readings from a temperature and humidity sensor
- Publish data to a cloud MQTT broker over WiFi
- Repeat periodically

This iteration, compared to the previous Arduino version, will assert more control over when and how the various tasks occur. Similar to the previous project, an open source driver for the DHT22 temperature and humidity sensor will be used so for now the interface to that device will remain a straightforward affair.

To achieve a like-for-like comparison between this project and the last, this project will start by replicating functionality, and then the new features will be added.

## Getting started

If you are cloning this repository, these steps will be unecessary. However, for completeness, the steps taken to create the project from scratch are shared here.

### Create project

Basic interactions with the ESP-IDF toolchain occurs using the `idf.py` command line tool. This set of commands shown below will:
- Activate the Python virtual environment required for the idf.py tool to work
- Create the skeleton project
- Configure the project for the esp32c3 target
- Compile the IDF core libraries. This final step will allow for your LSP to understand the context of your project when you start.

```bash
. $HOME/esp/esp-idf/export.sh
idf.py create-project esp_idf
cd esp_idf
idf.py set-target esp32c3
idf.py build
```

### Configure the settings for development

The ESP-IDF allows for adjustment of thousands of settings for the microcontroller by using the `menuconfig` UI. Every setting for the microcontroller is stored in the `sdkconfig` file in the project directory and whilst it is possible to manually edit this file, the recommended approach is to instead use the UI. You can enter menuconfig by entering the command `idf.py menuconfig`

For this project, most of the default settings are just fine, however two minor details will be changed. Below is a set of navigations described in the context of the main menu:

- Bootloader config -> Bootloader log verbosity -> Info
- Serial flasher config -> Flash size -> 4MB

By changing these two settings, we are telling the toolchain that we want the serial monitor to log messages of verbosity level `Info` or higher, and that our chip is connected to 4MB of flash memory.

## The main file

Using the same philosophy as the previous project, the main file will be designed as a simple entry point for the program. It will only call the relevant functions, which we will develop in separate files, and it will manage the timing of tasks.

The entry point for our code in an IDF project is the `void app_main(void)` function. We can conceive this as analogous to the `void setup()` function we were using in the Arduino implementation. By default in a new ESP-IDF project, this will be contained in the main folder in a file called `project_name.c`, where `project_name` is whatever you passed as an argument to the `idf.py create-project ...` command. My typical preference is for the entry point to be contained in a file called `main.c` so it is easy for anyone to infer where the code will start executing. Renaming the file becomes our first intoduction to the build system `CMake`.

If you open the `CMakeLists.txt` file in the main directory you will see something like:

```cmake
idf_component_register(SRCS "project_name.c"
                    INCLUDE_DIRS ".")
```

The build system uses this information to compile the `project_name.c` file in this directory, so it follows if we rename `project_name.c` to `main.c`, the CMakeLists.txt file will also need to be updated accordingly. This is something to keep in mind for every file and directory created in the project.

Now that we can be confident the compiler will find our main file, let's get in to writing how our program will flow in pseudocode:

```C
void app_main(void) {
    // Do this once
    sensor_init();
    wifi_init();
    mqtt_client_init();

    // Do this repeatedly
    sensor_read();
    mqtt_publish();
}
```

Herein lies the first major difference to the Arduino Framework. There is no `loop()` function, so if we are to repeatedly run a block of code, that is something for us to create. To do this, FreeRTOS will be used. This is the tool that the ESP-IDF uses to schedule tasks, and if you scrape away the surface layer of the Arduino framework for ESP32, it turns out that the `loop()` function is actually a FreeRTOS task anyway. To replicate that functionality in our ESP-IDF implementation, a task could be created as follows:

```C
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void read_and_send_task(void *pvParameters);

void app_main(void) {

    ---snip---

    xTaskCreate(&read_and_send_task, "read_and_send_task", 5120, NULL, 1, NULL);
}

void read_and_send_task(void *pvParameters) {
    while (true) {
        sensor_read();
        mqttPublish();
        vTaskDelay(PUBLISH_INTERVAL);
    }
    vTaskDelete(NULL);
}
```

The `xTaskCreate` function allocates 5,120 bytes to a thread to run the function `read_and_send_task`, gives the thread the same name and assigns the thread a priority of 1. FreeRTOS will then manage scheduling of this along with any other tasks in the program, according to their priorities, and whether they have yielded to the scheduler.

Within the `read_and_send_task` function, the infinite loop contained within the `while` block attempts to read and publish sensor values, exactly like in the previous implementation and the `vTaskDelay` allows the scheduler to schedule other threads until `PUBLISH_INTERVAL` elapses. This therefore replicates our baseline functionality and allows us to directly compare the Arduino framework and ESP-IDF projects for the `main` file. One final point to clarify is that a FreeRTOS task should never return. To prevent this undefined behaviour the `vTaskDelete(NULL)` function is placed outside the infinite loop, and this destroys the task. Even though the infinite loop should never break unless we make a mistake, it is considered best practice to include this boilerplate just in case.

Before moving on to implementing the functions this code calls, there is a simple optimisation to be made. Recall from our Arduino implementation, our scheduling was managed by the `delay(PUBLISH_INTERVAL)` function. That creates an inconvenience that it will always wait for `PUBLISH_INTERVAL` regardless of how long any of the reading or connection handling functions happened to take, so over time the actual gap between publish events will average out to something larger than publish interval. Here is one of the major benefits of splitting our tasks out:

1. It is possible to decouple all of the WiFi and MQTT connection management code into different tasks, and;
2. There are FreeRTOS scheduling tools allowing the task delay to be manipulated.

Point 1 will be explored in the implementation of those features, however point 2 can be optimised by using the `xTaskDelayUntil` function in place of `vTaskDelay`:

```C

---snip---

void read_and_send_task(void *pvParameters) {

    TickType_t task_start = xTaskGetTickCount();

    while (true) {
        sensor_read();
        mqttPublish();

        xTaskDelayUntil(&task_start, PUBLISH_INTERVAL / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
```

This way, the task will always ask the scheduler to start at `PUBLISH_INTERVAL` increments after it first started. It's a subtle difference, but it means that the occassional long running task won't muck with the next scheduled read and send event the same way a simple delay would.

## The peripheral_manager files

The pseudocode `sensor_init()` and `sensor_read()` functions called by `app_main` are the same as the interface as envisaged in the Arduino framework project. This provides a perfect opportunity to see how the implementations differ.

### Initiating and reading from the reed switch

Using ESP-IDF, the interface to initiate and read from a digital input are remarkably straightforward. The ESP-IDF provides a Hardware Abstraction Layer (HAL) that still permits us to totally ignore the underlying implementation details. In contrast to the Arduino implementation from the previous project, the only difference is one extra line of code because the direction and pull resistor choices are set by different functions:

```C
void sensor_init(void) {
    gpio_set_direction(DOOR_SW_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DOOR_SW_PIN, GPIO_PULLDOWN_ONLY);
}

void sensor_read(SensorData *sensor_data) {
    sensor_data->door_status = gpio_get_level(DOOR_SW_PIN);
}
```

Easy and painless.

### Initiating and reading from the DHT sensor

Our job is made similarly easy here because there is an existing library, or in the ESP-IDF nomenclature, `component`, to interface to the DHT22 sensor. This can be added to the project by entering the following command:

```bash
idf.py add-dependency "suda-morris/am2302_rmt^1.0.0"
idf.py build
```

The build step is not necessary, however it will compile the new library and therefore make it available to the LSP. This functionality to add dependencies is incredibly convenient because there is no need to worry about modifying CMake files - idf.py does it all for us.

Once the component has been installed, it looks like our particular application can simply use the code provided in the example folder contained in the component. Great! It appears there is one major difference that needs to be accounted for - The function to read from the sensor needs to be passed a handle in order to work properly:

```C
esp_err_t am2302_read_temp_humi(am2302_handle_t sensor, float *temp, float *humi);
```

A minor change to our approach makes this still trivial to deal with. The `sensor_read` function will need to take the sensor handle as an argument. All very doable.

### Making a component

Another useful feature of the ESP-IDF projects is the ability to place our own libraries in the `components` directory, keeping the main directory uncluttered, and giving us as developers the ability to copy/paste modules created for previous projects into new projects and the IDF immediately knowing how to access those abstractions. To use the peripheral manager as a component, it will be necessary to explore the directory structure and create a `CMakeLists.txt` file.

The directory structure should look like this:

```
esp_idf/
--CMakeLists.txt
--main/
----CMakeLists.txt
----main.c
--components/
----peripheral_manager/
------CMakeLists.txt
------peripheral_manager.h
------peripheral_manager.c
```

And the `CMakeLists.txt` for the `peripheral_manager` component will look like this:

```cmake
idf_component_register(SRCS "peripheral_manager.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver log
                    PRIV_REQUIRES main)
```

This tells cmake:

- Source code file(s), `SRCS`, for this component, is `peripheral_manager.c`
- The header file(s) for the component is located in `INCLUDE_DIRS`. In this case, the header is located in the root of the component
- The component depends on other components in `REQUIRES`. In this case the `driver` and `log` components of the IDF are dependencies
- The component depends privately on other components in `PRIV_REQUIRES`. In this case, `main` is privately included becuse the peripheral manager component accesses the `config.h` file from the `main` component, but does not intend to make any part of that file visible in its interfaces.

Then all that needs to be done is to ensure the appropriate variables are able to be passed between the relevant scopes. This is mainly a trivial exercise, however care needs to be taken passing arguments to a task because the single argument needs to be a `void *` type. Carefully managing the types can be achieved as follows:

```C
void app_main(void) {
    ---snip---
    // Casting the am2302_handle_t sensor to a void *
    xTaskCreate(&read_and_send_task, "read_and_send_task", 5 * 1024, (void *)sensor, 1, NULL);
}

void read_and_send_task(void *pvParameters) {
    ---snip---
    while (true) {
        // Casting back to am2302_handle_t
        am2302_handle_t sensor = (am2302_handle_t)pvParameters;
        SensorData sensor_data;
        sensor_read(&sensor, &sensor_data);
        ---snip---
    }
    vTaskDelete(NULL);
}
```

With a functioning peripheral manager implementation, it's time to figure out how to connect to the wifi network and send that data.

## The connection_manager files

The functions for the connection management are easily derived from examples provided by the libraries themselves. There are minor exceptions to this, mainly in the application-specific approach to connection failures.

The only reason to touch on these in any detail is to notice how different the implementation looks compared to the Arduino version. Aside from that, it's an exercise in copy and paste.

### WiFi

A look at the example WiFi connection code provided by Espressif shows the intrinsic link between FreeRTOS and the underlying operation of the ESP-IDF. The individual functions to initiate and start the WiFi connection are broadly familiar to someone with some background in Arduino, but the event handler provides significantly more implementation flexibility to the programmer.

The drawing attention to the `wifi_event_handler`, it can be seen that:

The `wifi_init` function:
- Creates an event loop
- Registers some `WIFI_EVENT` and `IP_EVENT` enum values to the `wifi_event_handler`

The event loop monitors the network interface and notifies on change of state, i.e. events. This means that it is up to the programmer to decide how to handle various events, and this is done in the `wifi_event_handler`. In this implementation, the event handler listens for certain events and takes action to connect to wifi and to attempt to reconnect whenever a disconnect event happens. This provides clear benefits where the user wishes to execute some custom code when events occur. In this example, the case managing `WIFI_EVENT_STA_DISCONNECTED` automatically attempts to reconnect to the network.

Contrast this to the Arduino implementation where the `connection_handler` we wrote had to execute in `loop()`, on the same thread as everything else.

### MQTT

The code used for the MQTT functions is similar to the Espressif examples so there's no need to run through. Like the WiFi, the client uses an event loop and an event handler function so the connection will manage events as they occur.

We again have some exposure to our build system. For the CA certificate we use connect to the broker, the ESP-IDF allows for files to be encoded to binary. So if we were to save a certificate to a `.pem` file by executing this command in the directory containing the MQTT component:

```bash
openssl s_client -connect your-broker.com:8883 -showcerts < /dev/null 2> /dev/null | openssl x509 -outform PEM > ca_cert.pem
```

The `ca_cert.pem` file will be written to flash as a binary if we place a line in the `CMakeLists.txt` file in the project root directory:

```cmake
---snip---
target_add_binary_data(esp_idf.elf "components/mqtt_manager/ca_cert.pem" TEXT)
```

And this data is accessible in our program by creating pointers to the data:

```C
extern const uint8_t datacake_ca_pem_start[]   asm("_binary_ca_cert_pem_start");
extern const uint8_t datacake_ca_pem_end[]     asm("_binary_ca_cert_pem_end");
```

## Interrupts and debounce timers

That now gives us a baseline implementation that replicates the functionality from the Arduino version of the project, with some minor improvements along the way. Arguably those improvements are a poor justification for the extra learning curve, so let's target one of the major shortfalls from the previous implementation - Publishing the door state change as it occurs.

Right now, the `gpio_get_level()` IDF function is used to read the door state on demand. It's possible to attach an interrupt handler to a pin, triggering the execution of a task whenever certain changes are detected on a pin. For our application, it would be ideal to have the state of the reed switch published as the changes occur, so as a `positive edge` OR `negative edge` happen, some code should be triggered to publish the state of the digital input. One approach is to add another init function and handler:

```C
void interrupt_init() {
    gpio_set_intr_type(DOOR_SW_PIN, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(DOOR_SW_PIN, door_state_change_callback, (void *)args);
}

static void door_state_change_callback(void *pvParameters) {
    //do stuff
}
```

This does part of what we want - Every time the `isr_service` detects `ANYEDGE` on `DOOR_SW_PIN`, the `door_state_change_callback` will be called. In a perfect world, all of the logic to read the sensor and publish the read could go in the `door_state_change_callback` function, when the switch is closed or open, the switch actually 'bounces' between those states many times in fractions of a second before the actual state settles. It's possible to debounce in software by simply adding a delay to wait out whatever happens in this stabilisation period.

This introduces another concept in FreeRTOS - Software timers. Using a FreeRTOS timer, we can delay any action from the interrupt until a stabilisation period has elapsed. After that period, a `callback` function will be triggered to take a read and send it:

```C
---snip---

static TimerHandle_t debounce_timer_handle;

void app_main(void) {
    ---snip---
    interrupt_init(&debounce_timer_handle);
    debounce_timer_handle = xTimerCreate("interrupt_send_callback", pdMS_TO_TICKS(DEBOUNCE_DURATION), pdFALSE, (void *) 0, interrupt_send_callback);
}

static void door_state_change_callback(void *pvParameters) {
    TimerHandle_t *debounce_timer_handle = (TimerHandle_t *)pvParameters;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xTimerStopFromISR(*debounce_timer_handle, &xHigherPriorityTaskWoken);
    xTimerStartFromISR(*debounce_timer_handle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void interrupt_send_callback(TimerHandle_t xTimer) {
    // Read and send
}
```

This is configured by:

 - Linking the `door_state_change_callback` to the relevant pin
 - Creating a software timer `debounce_timer_handle`. The timer runs for 50ms when triggered and then calls the `interrupt_send_callback`.

 When an interrupt is triggered, `door_state_change_callback` is called. This takes the `debounce_handle` and starts the timer. When the timer expires, the `interrupt_send_callback` finally takes a read of the sensor and publishes the state.

## Building and flashing

Commands to build, flash and monitor the project are as follows:

```bash
idf.py build
idf.py -p /dev/ttyACM0 flash
idf.py -p /dev/ttyACM0 monitor
```

The flash command performs a build step prior to attempting to flash. The commands can be chained, if preferred. i.e, `idf.py -p /dev/ttyACM0 flash monitor` achieves the same thing as doing all three one by one.

If you intend to deploy the device and don't want the logs output to serial, it's easy to change the verbosity of the logs with `menuconfig`:

- Bootloader config -> Bootloader log verbosity -> None

## Observations and Next Steps

The move from the Arduino framework to ESP-IDF resulted in an incrementally higher level of challenge but an exponentially greater level of control over implementation details. Everything the project set out to do has been achieved in this step, and it should be clear that by learning the details of FreeRTOS, there is a huge amount of freedom that can be gained in approaches to problems.

The next subproject will explore two ideas existing at opposite ends of a spectrum:

- Writing a basic driver for DHT, and explore how the devices actually communicate with each other
- Using C++ to see how some concepts of a higher level language can be used in embedded development

Check it out [here](https://github.com/TristanWebber/garage_monitor/tree/main/esp_cpp)
