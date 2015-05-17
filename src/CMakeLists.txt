
if (WIN32)
  set_source_files_properties(pinect.c PROPERTIES LANGUAGE CXX)
endif(WIN32)


include_directories (${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_BINARY_DIR}/UVC_PROJ-prefix/src/UVC_PROJ-build/include ${CMAKE_BINARY_DIR}/FREENECT_PROJ-prefix/src/FREENECT_PROJ-build/include ${CMAKE_SOURCE_DIR}/include)

if(PROJECT_OS_LINUX)

LIST(APPEND SRC pinect.c pinect-fnt.c pinect-v4l.c pinect-uvc.c)

else(PROJECT_OS_LINUX)

LIST(APPEND SRC pinect.c pinect-fnt.c pinect-uvc.c)

endif(PROJECT_OS_LINUX)

add_library (pinect SHARED ${SRC})
set_target_properties(pinect PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(pinect  ${CMAKE_BINARY_DIR}/FREENECT_PROJ-prefix/src/FREENECT_PROJ-build/lib/libfreenect.a ${CMAKE_BINARY_DIR}/UVC_PROJ-prefix/src/UVC_PROJ-build/libuvc.a ${LIBUSB_1_LIBRARIES})
install (TARGETS pinect DESTINATION lib)
