idf_component_register(SRCS "wifi_manager.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES esp_netif esp_wifi log
                    PRIV_REQUIRES main)
