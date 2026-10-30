

idf_component_register(
    SRCS "wireless.c" "main.c" "uart_input.c" "wheelchair_controller.c" "nanocobs/cobs.c" "pal.c" "wireless.c"
    INCLUDE_DIRS "." "nanocobs")

# Following flags can't be turned on, because code in the ESP-IDF does not conform
# -Wc++-compat -Wconversion

# target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror -Wc++-compat -Wconversion)
