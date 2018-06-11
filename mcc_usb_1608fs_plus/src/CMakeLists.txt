file(GLOB SRC *.c *.h)

add_library(mcc_usb_1608fs_plus
  ${BUILD_STATIC_OR_SHARED_LIBRARIES}
  SHARED
  ${SRC}
  )

INCLUDE_DIRECTORIES(".")

target_link_libraries(mcc_usb_1608fs_plus m pthread -lc -lm -lhidapi-libusb -lusb-1.0)
set_target_properties(mcc_usb_1608fs_plus PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION "${PROJECT_SOVERSION}")

