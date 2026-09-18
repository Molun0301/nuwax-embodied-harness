# 原生定位消息使用当前通信库配套生成器，避免混用不同版本的C++头文件。
set(NUWAX_NATIVE_ACK_SCHEMA "${CMAKE_CURRENT_LIST_DIR}/../../../../lcmtypes/nuwax_msgs/LioInputAck.lcm"
    CACHE FILEPATH "原生扫描处理回执的权威消息定义")
function(nuwax_generate_native_lcm_types schema_root)
  find_program(NUWAX_LCM_GEN_EXECUTABLE NAMES lcm-gen)
  if(NOT NUWAX_LCM_GEN_EXECUTABLE)
    message(FATAL_ERROR "Native LIO requires lcm-gen matching the selected LCM library")
  endif()
  set(message_names
    std_msgs_Time std_msgs_Header
    geometry_msgs_Point geometry_msgs_Quaternion geometry_msgs_Vector3
    geometry_msgs_Pose geometry_msgs_PoseWithCovariance
    geometry_msgs_Twist geometry_msgs_TwistWithCovariance
    nav_msgs_Odometry sensor_msgs_Imu sensor_msgs_PointField sensor_msgs_PointCloud2)
  set(schemas)
  foreach(name IN LISTS message_names)
    set(schema "${schema_root}/lcm_types/${name}.lcm")
    if(NOT EXISTS "${schema}")
      message(FATAL_ERROR "Native LIO message definition is missing: ${schema}")
    endif()
    list(APPEND schemas "${schema}")
  endforeach()
  get_filename_component(native_schema_directory "${NUWAX_NATIVE_ACK_SCHEMA}" DIRECTORY)
  list(APPEND schemas "${NUWAX_NATIVE_ACK_SCHEMA}" "${native_schema_directory}/LioInputFrame.lcm")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${schemas})
  set(output_dir "${CMAKE_CURRENT_BINARY_DIR}/nuwax-lcm-types")
  file(MAKE_DIRECTORY "${output_dir}")
  execute_process(
    COMMAND "${NUWAX_LCM_GEN_EXECUTABLE}" --cpp --cpp-hpath "${output_dir}" ${schemas}
    RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error)
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "Native LIO message generation failed: ${output}${error}")
  endif()
  set(NUWAX_NATIVE_LCM_INCLUDE_DIR "${output_dir}" PARENT_SCOPE)
endfunction()
