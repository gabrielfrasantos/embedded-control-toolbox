cmake_minimum_required(VERSION 3.21)

project(embedded-pid VERSION 1.0.0)

add_subdirectory(infra-lib)
add_subdirectory(control_toolbox)

if (NOT EMIL_BUILD_TESTS)
    add_subdirectory(hal-st)

    add_executable(embedded-pid main.cpp)

    target_link_libraries(embedded-pid PUBLIC
        infra.event
        hal_st.stm32fxxx
        hal_st.instantiations
        control_toolbox.pid
    )

    halst_target_default_linker_scripts(embedded-pid)
    halst_target_default_init(embedded-pid)
endif()
