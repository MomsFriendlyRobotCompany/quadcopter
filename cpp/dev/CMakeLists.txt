# set(WIFI_SSID "test" CACHE INTERNAL "secret")
# set(WIFI_PASSWORD "test" CACHE INTERNAL "secret")

if (PICO_CYW43_SUPPORTED) # set by PICO_BOARD=pico_w
    if (NOT TARGET pico_cyw43_arch)
        message("Skipping Pico W examples as support is not available")
    else()
        message("Pico W good")
    endif()
endif()

include(secrets.cmake)

list(APPEND examples
    timer
    # timer2
    # ultrasound
    # ultrasoundpio # doesn't work
    # usb
    # servopio
    # servo
    # gps
    # ext_int
    # multi_core
    # press_alt
    # mags
    # accels
    # iir_filter
    udp
)

# message(STATUS "Examples ----------------------")
foreach(app ${examples})
    add_executable(${app} ${app}.cpp)
    target_link_libraries(
        ${app}
            PRIVATE
                pico_stdlib
                pico_multicore
                hardware_exception
                hardware_adc
                hardware_i2c
                hardware_uart
                hardware_pio
                hardware_pwm
                hardware_spi
                hardware_rtc
                pico_cyw43_arch_lwip_threadsafe_background
                ${libs})

    target_include_directories(
        ${app}
        PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ../src
    )

    if (${app} STREQUAL "udp")
        target_link_libraries(
            ${app}
                PRIVATE
                    pico_cyw43_arch_lwip_threadsafe_background)

        target_compile_definitions(${app} PRIVATE
            WIFI_SSID=\"${WIFI_SSID}\"
            WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
            )
    endif()

    if (${app} STREQUAL "servopio")
    pico_generate_pio_header(${app} ${CMAKE_CURRENT_LIST_DIR}/pico_servo.pio)
    endif()

    # if (${app} STREQUAL "ultrasoundpio")
    # pico_generate_pio_header(${app} ${CMAKE_CURRENT_LIST_DIR}/pico_sr04.pio) # broke
    # endif()

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