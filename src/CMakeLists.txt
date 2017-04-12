SET(pwd ${CMAKE_CURRENT_LIST_DIR})

AUX_SOURCE_DIRECTORY(${pwd} src)
add_executable(cloth ${src})
message(STATUS "cloth added")

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

target_link_libraries(cloth ${stdgl_libraries})
target_link_libraries(cloth Eigen3::Eigen)
