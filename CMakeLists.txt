cmake_minimum_required(VERSION 3.21)

project(embedded-pid VERSION 1.0.0)

add_executable(embedded-pid Main.cpp)

target_link_libraries(embedded-pid PUBLIC
    infra.event
    hal_st.stm32fxxx
)

halst_target_default_linker_scripts(embedded-pid)
halst_target_default_init(embedded-pid)
