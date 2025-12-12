set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

idf_component_register(
    SRCS 
        "main.c"
        "../core/compute_cmac.c"
        "../core/neighbour_table.cpp"
        "../core/physics_integration.c"
        "../core/flocking_control.c"
        "../core/telemetry.c"
        "../core/monitor_task.c"

    INCLUDE_DIRS 
        "."
        "../core"

    REQUIRES 
        jgromes__radiolib 
        esp_driver_gpio 
        esp_timer
        mbedtls
        wpa_supplicant 
        esp_wifi 
        nvs_flash 
        esp_netif 
        mqtt
)