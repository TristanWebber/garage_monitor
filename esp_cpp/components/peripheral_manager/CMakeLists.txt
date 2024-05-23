idf_component_register(SRCS "peripheral_manager.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES driver log dht22
                    PRIV_REQUIRES main)
