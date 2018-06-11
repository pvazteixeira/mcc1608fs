set(LIBRARY_NAME mcc1608fs)

file(GLOB implementation_files *.c *.h)
file(GLOB header_files *.h *.hpp)

add_library(${LIBRARY_NAME} SHARED ${implementation_files})

#INCLUDE_DIRECTORIES(".")

target_link_libraries(${LIBRARY_NAME} m pthread -lc -lm -lhidapi-libusb -lusb-1.0)
set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION "${PROJECT_SOVERSION}")

pods_install_libraries(${LIBRARY_NAME})
pods_install_headers(${header_files} DESTINATION ${LIBRARY_NAME})
pods_install_pkg_config_file(${LIBRARY_NAME}
  LIBS -l${LIBRARY_NAME}
  VERSION 0.0.1)
