#include "solver.h"
#include <Eigen/Cholesky>
#include <algorithm>
#include <stdexcept>

NuwaxCeresSolver::NuwaxCeresSolver(unsigned threads) {
  options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options_.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  // 保持固定上游的收敛和信赖域策略，线程数由平台资源配置拥有。
  options_.function_tolerance = 1e-3;
  options_.gradient_tolerance = 1e-6;
  options_.parameter_tolerance = 1e-3;
  options_.max_num_consecutive_invalid_steps = 3;
  options_.max_consecutive_nonmonotonic_steps = 3;
  options_.use_nonmonotonic_steps = true;
  options_.jacobi_scaling = true;
  options_.dynamic_sparsity = true;
  options_.num_threads = static_cast<int>(threads);
  options_.max_num_iterations = 50;
  options_.logging_type = ceres::SILENT;
  Reset();
}
void NuwaxCeresSolver::Reset() {
  problem_.reset();
  edges_.clear(); nodes_.clear(); corrections_.clear();
  angle_manifold_.reset(AngleManifold::Create());
  ceres::Problem::Options options;
  options.enable_fast_removal = true;
  // Ceres默认将已移除残差保留到整个Problem析构；定位滑窗必须及时释放。
  options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_ = std::make_unique<ceres::Problem>(options);
}
void NuwaxCeresSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan>* v) {
  if (!v || !v->GetObject()) throw std::runtime_error("图节点为空");
  auto pose = v->GetObject()->GetCorrectedPose();
  const int id = v->GetObject()->GetUniqueId();
  auto [it, inserted] = nodes_.emplace(id, Eigen::Vector3d(pose.GetX(),pose.GetY(),pose.GetHeading()));
  if (!it->second.allFinite()) throw std::runtime_error("图节点位姿非有限");
  if (!inserted) return;
  auto& p = it->second;
  problem_->AddParameterBlock(&p[0],1);
  problem_->AddParameterBlock(&p[1],1);
  problem_->AddParameterBlock(&p[2],1,angle_manifold_.get());
}
void NuwaxCeresSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan>* e) {
  if (!e || !e->GetSource() || !e->GetTarget()) throw std::runtime_error("图约束为空");
  const int from=e->GetSource()->GetObject()->GetUniqueId(), to=e->GetTarget()->GetObject()->GetUniqueId();
  auto& a=nodes_.at(from); auto& b=nodes_.at(to);
  auto* link=dynamic_cast<karto::LinkInfo*>(e->GetLabel());
  if (!link || from==to) throw std::runtime_error("图约束无效");
  Eigen::Matrix3d covariance;
  for(int i=0;i<3;++i) for(int j=0;j<3;++j) covariance(i,j)=link->GetCovariance()(i,j);
  if (!covariance.allFinite() || !covariance.isApprox(covariance.transpose(),1e-6))
    throw std::runtime_error("图约束协方差无效");
  Eigen::LLT<Eigen::Matrix3d> cov_llt(covariance);
  if(cov_llt.info()!=Eigen::Success) throw std::runtime_error("图约束协方差非正定");
  Eigen::LLT<Eigen::Matrix3d> info_llt(cov_llt.solve(Eigen::Matrix3d::Identity()));
  if(info_llt.info()!=Eigen::Success) throw std::runtime_error("图约束信息矩阵分解失败");
  auto pose=link->GetPoseDifference();
  auto residual=std::unique_ptr<ceres::CostFunction>(PoseGraph2dErrorTerm::Create(pose.GetX(),pose.GetY(),pose.GetHeading(),info_llt.matrixU()));
  auto block=problem_->AddResidualBlock(residual.get(),nullptr,&a[0],&a[1],&a[2],&b[0],&b[1],&b[2]);
  // 完整端点键避免原单个散列值碰撞；同一端点的多约束分别拥有。
  edges_[std::minmax(from,to)].push_back({block,std::move(residual)});
}
void NuwaxCeresSolver::RemoveConstraint(int from,int to) {
  auto it=edges_.find(std::minmax(from,to));
  if(it==edges_.end()) return;
  problem_->RemoveResidualBlock(it->second.back().block);
  it->second.pop_back();
  if(it->second.empty())edges_.erase(it);
}
void NuwaxCeresSolver::RemoveNode(int id) {
  for(auto it=edges_.begin();it!=edges_.end();) {
    if(it->first.first==id || it->first.second==id) {
      for(const auto& constraint:it->second) problem_->RemoveResidualBlock(constraint.block);
      it=edges_.erase(it);
    } else ++it;
  }
  auto it=nodes_.find(id);
  if(it==nodes_.end()) return;
  for(int i=0;i<3;++i) problem_->RemoveParameterBlock(&it->second[i]);
  nodes_.erase(it);
}
void NuwaxCeresSolver::Compute() {
  corrections_.clear();
  if(nodes_.empty()) return;
  for(int i=0;i<3;++i) problem_->SetParameterBlockConstant(&nodes_.begin()->second[i]);
  auto before=nodes_;
  ceres::Solver::Summary summary;
  ceres::Solve(options_,problem_.get(),&summary);
  if(!summary.IsSolutionUsable()) {
    for(const auto& [id,p]:before) nodes_.at(id)=p;
    throw std::runtime_error("Ceres未生成可用图优化结果: "+summary.BriefReport());
  }
  for(const auto& [id,p]:nodes_) {
    if(!p.allFinite()) throw std::runtime_error("图优化结果非有限");
    corrections_.emplace_back(id,karto::Pose2(p[0],p[1],p[2]));
  }
  ++optimizations_;
}
