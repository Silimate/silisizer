set(ALL_TESTS
  tclreadline
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

add_test(
  NAME silisize_flags
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMAND
    $<TARGET_FILE:silisizer-bin>
    -exit
    ${CMAKE_CURRENT_SOURCE_DIR}/test_silisize_flags.tcl
)

add_test(
  NAME wns_policy
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/wns_policy
  COMMAND
    $<TARGET_FILE:silisizer-bin>
    -exit
    ${CMAKE_CURRENT_SOURCE_DIR}/wns_policy/test_wns_policy.tcl
)
