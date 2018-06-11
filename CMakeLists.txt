cmake_minimum_required(VERSION 2.8.0)


# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME mcc1608fs)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

add_subdirectory(src)


########################################################3

# add_subdirectory(mcc_usb_1608fs_plus)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUNIX -D_LINUX=1 -DCMDLINE=1")

# list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# find_package(LCM REQUIRED)
# find_package(LibBot REQUIRED)
# include_directories(${LCM_INCLUDE_DIRS})
# include_directories(${LIBBOT_INCLUDE_DIRS})

# SET(SRC
#   MCC1608FS.cpp
# )

# ADD_EXECUTABLE(MCC1608FS ${SRC})

# TARGET_LINK_LIBRARIES(MCC1608FS -lhidapi-libusb -lusb-1.0
#    mcc_usb_1608fs_plus
#    ${LCM_LIBRARIES}
#    ${LIBBOT_LIBRARIES}
#    pthread)

#target_link_libraries(attitude_control ${LCM_LIBRARIES} ${LIBBOT_LIBRARIES} ${CMAKE_SOURCE_DIR}/../../build/lib/liblcmtypes_NJORD.a )


