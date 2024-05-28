#include <chrono>
#include <condition_variable>
#include <thread>

#include "esp_err.h"
#include "nvs_flash.h"

#include "main.h"
#include "config.h"
#include "peripheral_manager.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"

static Main cpp_main;
Sensors sensors(Config::DOOR_SW_PIN, Config::DHT_PIN);
Wifi wifi;
Mqtt mqtt_client;

std::mutex interrupt_mux;
std::condition_variable interrupt_cv;
bool interrupt_ready = false;
bool door_state;

void Main::start(void) {

    sensors.init();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(wifi.init());

    while (wifi.get_state() != Wifi::state_e::CONNECTED) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    ESP_ERROR_CHECK(mqtt_client.client_init());

    create_tasks();
}

void Main::create_tasks(void) {

    std::thread read_and_send_thread(&Main::read_and_send);
    std::thread interrupt_listen_thread(&Main::interrupt_listen);
    std::thread interrupt_send_thread(&Main::interrupt_send);

    read_and_send_thread.join();
    interrupt_listen_thread.join();
    interrupt_send_thread.join();
}

[[noreturn]] void Main::read_and_send(void) {

    using namespace std::chrono;
    auto task_start = steady_clock::now();

    while (true) {
        Sensors::SensorData sensor_data;
        sensors.read(&sensor_data);

        while (mqtt_client.get_state() != Mqtt::state_e::CONNECTED) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        char pub_buff[6];
        sprintf(pub_buff, "%d", sensor_data.door_status);
        mqtt_client.publish(Config::DOOR_TOPIC, pub_buff, false);
        sprintf(pub_buff, "%.2f", sensor_data.temperature);
        mqtt_client.publish(Config::TEMP_TOPIC, pub_buff, false);
        sprintf(pub_buff, "%.2f", sensor_data.humidity);
        mqtt_client.publish(Config::HUMI_TOPIC, pub_buff, false);

        auto task_timing_offset = (steady_clock::now() - task_start) % seconds {Config::PUBLISH_INTERVAL_SECONDS};
        auto next_event = steady_clock::now() + seconds {Config::PUBLISH_INTERVAL_SECONDS} - task_timing_offset;
        std::this_thread::sleep_until(next_event);
    }
}

[[noreturn]] void Main::interrupt_send(void) {
    while (true) {
        std::unique_lock<std::mutex> lock(interrupt_mux);
        while (!interrupt_ready) {
            interrupt_cv.wait(lock);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        bool door_status = sensors.get_door_state();

        while (mqtt_client.get_state() != Mqtt::state_e::CONNECTED) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        char pub_buff[2];
        sprintf(pub_buff, "%d", door_status);
        mqtt_client.publish(Config::DOOR_TOPIC, pub_buff, false);

        interrupt_ready = false;
    }
}

[[noreturn]] void Main::interrupt_listen(void) {
    door_state = sensors.get_door_state();
    while (true) {
        bool read_state = sensors.get_door_state();
        if (read_state != door_state) {
            {
                std::lock_guard<std::mutex> lock(interrupt_mux);
                interrupt_ready = true;
                door_state = read_state;
            }
            interrupt_cv.notify_one();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

extern "C" void app_main(void) {
    cpp_main.start();
}
