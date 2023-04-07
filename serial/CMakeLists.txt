cmake_minimum_required(VERSION 3.5)
project(serial)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find catkin
find_package(ament_cmake REQUIRED)


    # If Linux, add rt and pthread
set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)
include_directories(${PROJECT_NAME} PRIVATE include) 

set(serial_SRCS
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)


list(APPEND serial_SRCS src/impl/unix.cc)
list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)


## Add serial library
add_library(${PROJECT_NAME} SHARED ${serial_SRCS})

target_link_libraries(${PROJECT_NAME} rt pthread)




## Uncomment for example
#add_executable(serial_example examples/serial_example.cc)
#add_dependencies(serial_example ${PROJECT_NAME})
#target_link_libraries(serial_example ${PROJECT_NAME})

## Include headers
include_directories(include)

## Install executable
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

## Install headers
install(DIRECTORY include/
DESTINATION include/)

ament_export_include_directories(include) 
ament_export_libraries(${PROJECT_NAME})


ament_package()
