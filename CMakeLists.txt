# Do not change the order of these commands
cmake_minimum_required(VERSION 3.16)
#set(EXTRA_COMPONENT_DIRS "../components")
set(COMPONENTS "main" "main/my_lcd" "main/pin")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(light_controller)
# idf_build_set_property(COMPILE_OPTIONS "-Wno-error" APPEND)
