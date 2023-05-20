add_executable(control_toolbox.pid_test)
emil_build_for(control_toolbox.pid_test BOOL EMIL_BUILD_TESTS)
emil_add_test(control_toolbox.pid_test)

target_link_libraries(control_toolbox.pid_test PUBLIC
    gmock_main
    control_toolbox.pid
)

target_sources(control_toolbox.pid_test PRIVATE
    test_pid.cpp
)
