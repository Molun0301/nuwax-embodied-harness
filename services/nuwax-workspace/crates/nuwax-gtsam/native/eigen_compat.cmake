# 固定上游的一处新向量别名不兼容Go2的Eigen 3.3；改为完全等价的矩阵类型。
# 只修改获取到的依赖源码，不替换系统Eigen，也不改变优化算法。
set(source "${NUWAX_GTSAM_SOURCE_DIR}/gtsam/nonlinear/WnoaFactorGraph.cpp")
file(READ "${source}" content)
set(original "const Eigen::Vector<double, dim> q_psd_diag")
set(compatible "const Eigen::Matrix<double, dim, 1> q_psd_diag")
string(FIND "${content}" "${original}" original_index)
string(FIND "${content}" "${compatible}" compatible_index)
if(NOT original_index EQUAL -1)
  string(REPLACE "${original}" "${compatible}" content "${content}")
  file(WRITE "${source}" "${content}")
elseif(compatible_index EQUAL -1)
  message(FATAL_ERROR "固定GTSAM源码中的Eigen兼容目标未找到")
endif()
