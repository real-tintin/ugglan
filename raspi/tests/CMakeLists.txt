cmake_minimum_required(VERSION 3.1)

set(CMakeDemo_TEST_SRC
    test_wall_time.cpp
    catch_with_main.cpp
)
set(CMakeDemo_TEST_HEADER
    catch.h
)

PREPEND(CMakeDemo_TEST_SRC)

# Make an executable target that depends on the test source code we specified
# above.
add_executable(TestCMakeDemo ${CMakeDemo_TEST_SRC} ${CMakeDemo_TEST_HEADER})

# Enable testing via CTest
enable_testing()
# Add our test as runnable via CTest
add_test(NAME TestCMakeDemo  COMMAND TestCMakeDemo)

# Link our unit tests against the library we compiled
target_link_libraries(TestCMakeDemo CMakeDemo)
