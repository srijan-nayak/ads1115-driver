add_executable(basic_read basic_read.cpp)
target_link_libraries(basic_read PRIVATE ads1115)

add_executable(continuous_read continuous_read.cpp)
target_link_libraries(continuous_read PRIVATE ads1115)
