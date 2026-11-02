# 原生定位消息使用当前通信库配套生成器，避免混用不同版本的C++头文件。
set(NUWAX_NATIVE_ACK_SCHEMA "${CMAKE_CURRENT_LIST_DIR}/../../../../lcmtypes/nuwax_msgs/LioInputAck.lcm"
    CACHE FILEPATH "原生扫描处理回执的权威消息定义")
function(nuwax_generate_native_lcm_types schema_root)
  find_program(NUWAX_LCM_GEN_EXECUTABLE NAMES lcm-gen)
  set(generator "${NUWAX_LCM_GEN_EXECUTABLE}")
  if(LCM_VERSION)
    # 生成的C++头文件依赖同版本lcm_coretypes.h：Ubuntu 20.04的LCM 1.3.1头文件不能编译1.5.x生成器的输出。
    # 安装器构建时PATH最前面是Python环境，其中pip附带的lcm-gen可能与系统库不同版本，因此按库版本挑选；
    # 用局部变量，旧构建目录缓存的生成器路径版本不符时不再沿用。
    set(generator "")
    foreach(candidate IN ITEMS "${LCM_PREFIX}/bin/lcm-gen" "${NUWAX_LCM_GEN_EXECUTABLE}")
      if(NOT generator AND EXISTS "${candidate}")
        execute_process(COMMAND "${candidate}" --version
          OUTPUT_VARIABLE candidate_version OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
        if(candidate_version STREQUAL "lcm-gen ${LCM_VERSION}")
          set(generator "${candidate}")
        endif()
      endif()
    endforeach()
    if(NOT generator)
      message(FATAL_ERROR "Native LIO requires lcm-gen ${LCM_VERSION} matching the selected LCM library; "
        "checked ${LCM_PREFIX}/bin/lcm-gen and ${NUWAX_LCM_GEN_EXECUTABLE}")
    endif()
  endif()
  if(NOT generator)
    message(FATAL_ERROR "Native LIO requires lcm-gen matching the selected LCM library")
  endif()
  message(STATUS "Native LIO messages use ${generator} for LCM ${LCM_VERSION}")
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
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    ${schemas} "${generator}")
  set(output_dir "${CMAKE_CURRENT_BINARY_DIR}/nuwax-lcm-types")
  # Generate the complete set before touching consumed headers. Discard only
  # this generator-owned staging directory, including leftovers after failure.
  set(staging_dir "${output_dir}-staging")
  file(REMOVE_RECURSE "${staging_dir}")
  file(MAKE_DIRECTORY "${staging_dir}")
  execute_process(
    COMMAND "${generator}" --cpp --cpp-hpath "${staging_dir}" ${schemas}
    RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error)
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "Native LIO message generation failed: ${output}${error}")
  endif()
  file(GLOB_RECURSE generated_headers RELATIVE "${staging_dir}" "${staging_dir}/*.hpp")
  file(GLOB_RECURSE previous_headers RELATIVE "${output_dir}" "${output_dir}/*.hpp")
  foreach(header IN LISTS generated_headers)
    # COPYONLY preserves mtime when generator output bytes have not changed.
    configure_file("${staging_dir}/${header}" "${output_dir}/${header}" COPYONLY)
  endforeach()
  foreach(header IN LISTS previous_headers)
    if(NOT header IN_LIST generated_headers)
      file(REMOVE "${output_dir}/${header}")
    endif()
  endforeach()
  set(NUWAX_NATIVE_LCM_INCLUDE_DIR "${output_dir}" PARENT_SCOPE)
endfunction()
