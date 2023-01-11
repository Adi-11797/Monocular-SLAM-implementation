add_executable(unit_test MACOSX_BUNDLE main.cpp)

target_link_libraries(
    unit_test PRIVATE mono-slam utils CLI11::CLI11
)
add_test(NAME unitTest COMMAND unit_test)