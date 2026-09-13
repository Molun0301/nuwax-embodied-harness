# 两种原生定位前端共用同一平面拟合；只在构建目录修改锁定源的副本。
set(_NUWAX_PLANE_FIT_MODULE_DIR "${CMAKE_CURRENT_LIST_DIR}")
function(nuwax_prepare_native_plane_fit source_dir patched_dir)
  configure_file("${source_dir}/include/common_lib.h" "${patched_dir}/src/common_lib.h" COPYONLY)
  configure_file("${_NUWAX_PLANE_FIT_MODULE_DIR}/stable_plane_fit.hpp"
    "${patched_dir}/src/stable_plane_fit.hpp" COPYONLY)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${_NUWAX_PLANE_FIT_MODULE_DIR}/stable-plane-fit.patch")
  execute_process(
    COMMAND "${NUWAX_PATCH_EXECUTABLE}" --batch --forward --fuzz=0 --no-backup-if-mismatch -p1
      -i "${_NUWAX_PLANE_FIT_MODULE_DIR}/stable-plane-fit.patch"
    WORKING_DIRECTORY "${patched_dir}"
    RESULT_VARIABLE patch_result OUTPUT_VARIABLE patch_output ERROR_VARIABLE patch_error)
  if(NOT patch_result EQUAL 0)
    message(FATAL_ERROR "共享平面拟合补丁无法应用：${patch_output}${patch_error}")
  endif()
endfunction()
