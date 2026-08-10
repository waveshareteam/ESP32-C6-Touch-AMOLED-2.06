set(SRCS "")
list(APPEND SRCS
    "src/bsp_board_extra.c"
)

set(INCLUDE_DIRS "")
list(APPEND INCLUDE_DIRS "include")

if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.5")
    set(BSP_EXTRA_REQUIRES esp_driver_gpio esp_driver_i2s)
    set(BSP_EXTRA_PRIV_REQUIRES esp_driver_i2c esp_driver_ledc)
else()
    set(BSP_EXTRA_REQUIRES driver)
    set(BSP_EXTRA_PRIV_REQUIRES "")
endif()

foreach(SRC_DIR IN LISTS SRC_DIRS)
    file(GLOB_RECURSE SRC ${SRC_DIR}/*.c)
    list(APPEND SRCS ${SRC})
    list(APPEND INCLUDE_DIRS ${SRC_DIR}/include)
endforeach()

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES ${BSP_EXTRA_REQUIRES}
    PRIV_REQUIRES ${BSP_EXTRA_PRIV_REQUIRES} esp_timer fatfs
)

target_compile_options(
    ${COMPONENT_LIB} PRIVATE
        -Wno-format
        -Wno-int-conversion
        -Wno-incompatible-pointer-types
        -Wunused-function
        -Wno-unused-variable
        -Wno-unused-function
        -Wno-overflow
        -Wno-unused-but-set-variable
        -Wno-discarded-qualifiers)
