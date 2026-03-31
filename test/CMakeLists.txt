add_executable(openkarto_tests
  test_placeholder.cpp
  test_types.cpp
  test_meta_removal.cpp
  test_string_conversion.cpp
  test_geometry.cpp
)

target_link_libraries(openkarto_tests PRIVATE
  OpenKarto
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(openkarto_tests)
