add_executable(openkarto_tests
  test_placeholder.cpp
)

target_link_libraries(openkarto_tests PRIVATE
  OpenKarto
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(openkarto_tests)
