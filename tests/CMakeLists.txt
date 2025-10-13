set(ALL_TESTS
  picorv32
)

foreach(test_name ${ALL_TESTS})
  set(test_dir ${CMAKE_CURRENT_SOURCE_DIR}/${test_name})
  add_test(
    NAME ${test_name}
    WORKING_DIRECTORY ${test_dir}
    COMMAND
      ./test
      $<TARGET_FILE:silisizer-bin>
  )
endforeach()
