idf_component_register(
        SRCS "main.c" "sd_card_file_system.c" "fseq.c" "interface.c" "my_lcd/my_lcd.c" "pin/pin.c"
        "."
        PRIV_REQUIRES fatfs driver esp_timer
        )
# PRIV_REQUIRES "..."
# target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
