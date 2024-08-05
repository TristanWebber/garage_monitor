pub const WIFI_SSID: &str = "your_ssid";//OR env!("WIFI_SSID");
pub const WIFI_PASS: &str = "your_pw";//OR env!("WIFI_PASS");

pub const BROKER_URI: &str = "mqtts://your.mqtt-server.com:8883";
pub const BROKER_USER: &str = "your_broker_username";
pub const BROKER_PASS: &str = "your_broker_password";
pub const DOOR_TOPIC: &str = "full_path_to_topic/DOOR";
pub const TEMP_TOPIC: &str = "full_path_to_topic/TEMP";
pub const HUMI_TOPIC: &str = "full_path_to_topic/HUMI";

pub const PUBLISH_INTERVAL: u32 = 300;
pub const DEBOUNCE_MS: u32 = 50;