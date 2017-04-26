SET(pwd ${CMAKE_CURRENT_LIST_DIR})

AUX_SOURCE_DIRECTORY(${pwd} src)
add_executable(cloth ${src})
message(STATUS "cloth added")

#target_link_libraries(cloth ${stdgl_libraries} ${LIBIGL_EXTRA_LIBRARIES} Eigen3::Eigen)
target_link_libraries(cloth ${stdgl_libraries})
