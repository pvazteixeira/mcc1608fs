file(GLOB_RECURSE implem_files *.cpp)
file(GLOB_RECURSE header_files *.hpp)

set(EXE_NAME ${POD_NAME}-publisher)

add_executable(${EXE_NAME} ${implem_files} ${header_files})

# 3. pkg-config packages
set(REQUIRED_PACKAGES bot2-core bot2-param-client mcc1608fs)
pods_use_pkg_config_packages(${EXE_NAME} ${REQUIRED_PACKAGES})

TARGET_LINK_LIBRARIES(${EXE_NAME} -lhidapi-libusb -lusb-1.0 pthread)
pods_install_executables(${EXE_NAME})
