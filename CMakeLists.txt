# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(littlefs_filesystem)

set(PDL_DRIVER_DIR ${ZEPHYR_HAL_INFINEON_MODULE_DIR}/mtb-pdl-cat1/drivers/source)
set(HAL_SOURCE_DIR ${ZEPHYR_HAL_INFINEON_MODULE_DIR}/mtb-hal-cat1/COMPONENT_PSOC6HAL/source)
set(LITTLEFS_LIB_DIR ${ZEPHYR_HAL_INFINEON_MODULE_DIR}/../../fs/littlefs)
set(LITTLEFS_BD_LIB_DIR ${ZEPHYR_HAL_INFINEON_MODULE_DIR}/../../fs/littlefs/bd)
set(MTB_LITTLEFS_LIB_DIR ${ZEPHYR_HAL_INFINEON_MODULE_DIR}/../../lib/mtb-littlefs)
set(SERIAL_FLASH_LIB_DIR ${ZEPHYR_HAL_INFINEON_MODULE_DIR}/../../lib/serial-flash)
set(ABSTRACTION_RTOS_ZEPHYR_DIR ${ZEPHYR_HAL_INFINEON_MODULE_DIR}/../../../zephyr/modules/hal_infineon/abstraction-rtos)

zephyr_library_sources(${PDL_DRIVER_DIR}/cy_smif.c ${PDL_DRIVER_DIR}/cy_smif_memslot.c ${PDL_DRIVER_DIR}/cy_dma.c ${PDL_DRIVER_DIR}/cy_sd_host.c)
zephyr_library_sources(${HAL_SOURCE_DIR}/cyhal_qspi.c ${HAL_SOURCE_DIR}/cyhal_sdhc.c)
zephyr_include_directories(${LITTLEFS_LIB_DIR})
zephyr_include_directories(${LITTLEFS_BD_LIB_DIR})
zephyr_include_directories(${MTB_LITTLEFS_LIB_DIR}/include)
zephyr_include_directories(${SERIAL_FLASH_LIB_DIR})
zephyr_include_directories(${ABSTRACTION_RTOS_ZEPHYR_DIR}/include/COMPONENT_ZEPHYR)
FILE(GLOB littlefs_sources ${LITTLEFS_LIB_DIR}/*.c ${LITTLEFS_BD_LIB_DIR}/*.c)
FILE(GLOB serial_flash_sources ${SERIAL_FLASH_LIB_DIR}/*.c)
FILE(GLOB mtb_littlefs_sources ${MTB_LITTLEFS_LIB_DIR}/source/*.c)
FILE(GLOB abstraction_rtos_sources ${ABSTRACTION_RTOS_ZEPHYR_DIR}/source/COMPONENT_ZEPHYR/*.c)
zephyr_library_sources(${littlefs_sources} ${serial_flash_sources} ${mtb_littlefs_sources} ${abstraction_rtos_sources})

add_definitions( -DCOMPONENT_CUSTOM_DESIGN_MODUS )
add_definitions( -DCY_RETARGET_IO_CONVERT_LF_TO_CRLF )
add_definitions( -DLFS_THREADSAFE )

zephyr_include_directories(GeneratedSource src)
FILE(GLOB app_sources GeneratedSource/*c src/*.c)
target_sources(app PRIVATE ${app_sources})
