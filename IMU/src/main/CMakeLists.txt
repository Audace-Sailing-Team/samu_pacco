idf_component_register(
    SRCS "main.c" "bno055.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer freertos
)