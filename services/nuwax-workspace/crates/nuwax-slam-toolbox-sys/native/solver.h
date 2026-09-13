#pragma once
#include <karto_sdk/Mapper.h>
#include <ceres/ceres.h>
#include <map>
#include <memory>
#include <set>
#include "solvers/ceres_utils.h"

// 使用上游二维残差和角度流形；拥有方串行调用，不依赖中间件或全局日志。
class NuwaxCeresSolver final : public karto::ScanSolver {
 public:
  explicit NuwaxCeresSolver(unsigned threads);
  void Compute() override;
  const IdPoseVector& GetCorrections() const override { return corrections_; }
  void AddNode(karto::Vertex<karto::LocalizedRangeScan>* vertex) override;
  void AddConstraint(karto::Edge<karto::LocalizedRangeScan>* edge) override;
  void RemoveNode(kt_int32s id) override;
  void RemoveConstraint(kt_int32s from, kt_int32s to) override;
  void Clear() override { corrections_.clear(); }
  void Reset() override;
  uint64_t optimizations() const { return optimizations_; }
 private:
  using Key = std::pair<int, int>;
  struct Constraint {
    ceres::ResidualBlockId block;
    std::unique_ptr<ceres::CostFunction> cost;
  };
  ceres::Solver::Options options_;
  // problem先销毁，随后才释放它引用的残差、角度流形和节点。
  std::map<int, Eigen::Vector3d> nodes_;
  std::unique_ptr<ceres::Manifold> angle_manifold_;
  std::map<Key, std::vector<Constraint>> edges_;
  std::unique_ptr<ceres::Problem> problem_;
  IdPoseVector corrections_;
  uint64_t optimizations_ = 0;
};
