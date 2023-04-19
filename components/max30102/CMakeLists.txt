set(component_srcs "max30102.c" "MAX30102_ESP32C3.c")

idf_component_register(SRCS "${component_srcs}"
                       PRIV_REQUIRES driver
                       INCLUDE_DIRS ".")
