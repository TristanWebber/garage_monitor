idf_component_register(SRCS "mqtt_manager.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES mqtt log
                    PRIV_REQUIRES main)
