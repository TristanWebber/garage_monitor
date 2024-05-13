idf_component_register(SRCS "peripheral_manager.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver log
                    PRIV_REQUIRES main)
