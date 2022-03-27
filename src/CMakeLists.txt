add_executable(${CMAKE_PROJECT_NAME})


# link the executable using the IR transmit and receive libraries
#
target_link_libraries(${CMAKE_PROJECT_NAME} LINK_PUBLIC
pico_stdlib
hardware_spi
)


target_sources(${CMAKE_PROJECT_NAME} PRIVATE main.c)

pico_add_extra_outputs(${CMAKE_PROJECT_NAME})
