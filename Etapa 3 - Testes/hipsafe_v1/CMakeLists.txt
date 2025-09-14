cmake_minimum_required(VERSION 3.10)
project(host_tests CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(host_tests
    main.cpp
    src/evento.cpp         # implementação
    src/analise_postural.cpp
    src/bogus.cpp
)

target_include_directories(host_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc   # para encontrar analise_postural.h e evento.h
)

# Warnings úteis
target_compile_options(host_tests PRIVATE -Wall -Wextra -Wpedantic)

# pthread pode ser necessário no Linux para std::chrono
if(UNIX AND NOT APPLE)
    target_link_libraries(host_tests PRIVATE pthread)
endif()
