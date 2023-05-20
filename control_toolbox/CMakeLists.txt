add_library(control_toolbox.pid ${EMIL_EXCLUDE_FROM_ALL} STATIC)

target_include_directories(control_toolbox.pid PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(control_toolbox.pid PUBLIC
    infra.util
)

target_sources(control_toolbox.pid PRIVATE
    include/pid.hpp
    src/pid.cpp
)

add_subdirectory(test)
