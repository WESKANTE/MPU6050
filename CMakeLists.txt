cmake_minimum_required(VERSION 3.19) #Version minima requerida para compilar
#Importando herramientas de sdk
include(pico_sdk_import.cmake)
#Definir proyecto
project(mpu)
#Inicializando las herramientas de sdk
pico_sdk_init()



#Indicar donde se hace este pedo
add_executable(mpu
        main.c
        )
pico_enable_stdio_usb(mpu 1)
pico_enable_stdio_uart(mpu 1)

# create map/bin/hex file etc. Crea archivos extra para debug
pico_add_extra_outputs(mpu)

# pull in common dependencies. Añadir las librerias al proyecto
target_link_libraries(mpu pico_stdlib hardware_i2c)


