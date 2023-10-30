list(APPEND examples
    timer
    servo
    gps
)

# message(STATUS "Examples ----------------------")
foreach(app ${examples})
    add_executable(${app} ${app}.cpp)
    target_link_libraries(
        ${app}
            PRIVATE
                pico_stdlib
                # pico_multicore
                hardware_exception
                hardware_adc
                hardware_i2c
                hardware_uart
                hardware_pio
                hardware_pwm
                # hardware_spi
                ${libs})
    target_include_directories(
        ${app}
        PRIVATE
        ../src
    )

    # create uf2 output --------------------------------------------------
    pico_add_extra_outputs(${app})

    # stdio -> USB --------------------------------------------------------
    pico_enable_stdio_usb(${app} 1)
    pico_enable_stdio_uart(${app} 0)

    # fix cleanup command ------------------------------------------------
    set(clean_files
        ${app}.bin
        ${app}.hex
        ${app}.elf
        ${app}.dis
        ${app}.elf.map
        ${app}.uf2)

    set_target_properties(${app}
        PROPERTIES
            ADDITIONAL_CLEAN_FILES
                "${clean_files}")

    # Show info after project bulid ---------------------------------------
    add_custom_command(
        TARGET ${app}
        POST_BUILD
        COMMAND echo -------------------------------------
                # && ls -al ${PROJECT_NAME}.uf2
                # && stat -f "%N %z / 2097152B of Flash" ${PROJECT_NAME}.uf2
                # && echo -------------------------------------
                && picotool info ${app}.uf2 -a
                && echo -------------------------------------
    )

endforeach()