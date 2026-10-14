#include "solver.h"
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
using Point=pcl::PointNormal;
using Cloud=pcl::PointCloud<Point>;

// 单平面等场景的法线不能约束完整位姿。先检验每次迭代实际使用的对应关系，
// 避免点到平面线性求解产生非有限变换，继而在点云库的最近邻查询中触发进程断言。
bool observable(const Cloud& source, const Cloud& target, pcl::KdTreeFLANN<Point>& tree,
                const Eigen::Matrix4f& estimate, double distance) {
    std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> matches;
    matches.reserve(source.size());
    Eigen::Vector3d center=Eigen::Vector3d::Zero();
    std::vector<int> indices(1);std::vector<float> distances(1);
    for (const auto& p:source) {
        const Eigen::Vector3f transformed=(estimate*Eigen::Vector4f(p.x,p.y,p.z,1)).head<3>();
        if (!transformed.allFinite()) return false;
        Point query{};query.x=transformed.x();query.y=transformed.y();query.z=transformed.z();
        if (tree.nearestKSearch(query,1,indices,distances)!=1 || distances[0]>distance*distance) continue;
        const auto& t=target[indices[0]];
        Eigen::Vector3d normal(t.normal_x,t.normal_y,t.normal_z);
        if (!normal.allFinite() || normal.norm()<1e-6) continue;
        normal.normalize();
        const Eigen::Vector3d point=transformed.cast<double>();
        center+=point;matches.emplace_back(point,normal);
    }
    if (matches.size()<6) return false;
    center/=static_cast<double>(matches.size());
    double radius2=0;
    for (const auto& pair:matches) radius2+=(pair.first-center).squaredNorm();
    const double radius=std::sqrt(std::max(radius2/matches.size(),0.25));
    Eigen::Matrix<double,6,6> information=Eigen::Matrix<double,6,6>::Zero();
    for (const auto& pair:matches) {
        Eigen::Matrix<double,6,1> row;
        row.head<3>()=pair.second;
        row.tail<3>()=(pair.first-center).cross(pair.second)/radius;
        information.noalias()+=row*row.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> spectrum(information);
    if (spectrum.info()!=Eigen::Success || !spectrum.eigenvalues().allFinite()) return false;
    return spectrum.eigenvalues()[0]>1e-4*spectrum.eigenvalues()[5];
}
}

extern "C" int nuwax_pcl_point_to_plane(const double* source, std::size_t source_count,
    const double* target, std::size_t target_count, double max_distance, std::uint32_t iterations,
    NuwaxRegistrationResult* result) noexcept {
    if (!result) return 1;
    *result={{{0,0,0},{0,0,0,1}},std::numeric_limits<double>::infinity(),0,0};
    if (!source || !target || source_count<3 || target_count<3) return 0;
    if (source_count>100000 || target_count>1000000 || !std::isfinite(max_distance)
        || max_distance<=0 || !iterations || iterations>300) return 1;
    try {
        using Point=pcl::PointNormal;
        pcl::PointCloud<Point>::Ptr src(new pcl::PointCloud<Point>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::size_t i=0;i<source_count;++i) {
            Point p{}; p.x=source[3*i];p.y=source[3*i+1];p.z=source[3*i+2];
            if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)) return 1;
            src->push_back(p);
        }
        for (std::size_t i=0;i<target_count;++i) {
            pcl::PointXYZ p; p.x=target[3*i];p.y=target[3*i+1];p.z=target[3*i+2];
            if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)) return 1;
            tgt_xyz->push_back(p);
        }
        pcl::KdTreeFLANN<pcl::PointXYZ> tree;
        tree.setInputCloud(tgt_xyz);
        pcl::PointCloud<Point>::Ptr tgt(new pcl::PointCloud<Point>);
        for (const auto& p:*tgt_xyz) {
            std::vector<int> indices;std::vector<float> distances;
            // 对齐Dimos点到平面匹配的0.3米、最多30邻点法线估计。
            if (tree.radiusSearch(p,0.3,indices,distances,30)<3) continue;
            Eigen::Vector4f plane;float curvature;
            if (!pcl::computePointNormal(*tgt_xyz,indices,plane,curvature) || !plane.allFinite()) continue;
            Point n{};n.x=p.x;n.y=p.y;n.z=p.z;n.normal_x=plane[0];n.normal_y=plane[1];n.normal_z=plane[2];
            tgt->push_back(n);
        }
        if (tgt->size()<3) return 0;
        struct Icp : pcl::IterativeClosestPointWithNormals<Point,Point> {
            int iterations() const { return this->nr_iterations_; }
        } icp;
        icp.setInputSource(src);icp.setInputTarget(tgt);
        icp.setMaximumIterations(1);
        icp.setTransformationEpsilon(1e-8);icp.setEuclideanFitnessEpsilon(1e-8);
        pcl::KdTreeFLANN<Point> target_tree;target_tree.setInputCloud(tgt);
        Eigen::Matrix4f matrix=Eigen::Matrix4f::Identity();
        std::uint32_t used_iterations=0;
        const auto refine=[&](double distance) {
            icp.setMaxCorrespondenceDistance(distance);
            for (std::uint32_t i=0;i<iterations;++i) {
                if (!observable(*src,*tgt,target_tree,matrix,distance)) return false;
                Cloud aligned;icp.align(aligned,matrix);
                const Eigen::Matrix4f next=icp.getFinalTransformation();
                if (!icp.hasConverged() || !next.allFinite()) return false;
                const Eigen::Matrix4f delta=next*matrix.inverse();
                if (!delta.allFinite()) return false;
                matrix=next;++used_iterations;
                const Eigen::AngleAxisd angle(delta.block<3,3>(0,0).cast<double>());
                if (delta.block<3,1>(0,3).norm()<1e-5 && std::abs(angle.angle())<1e-5) return true;
            }
            return false;
        };
        if (!refine(max_distance)) return 0;
        // 粗匹配负责捕获漂移，精匹配收紧对应距离；复用同一目标、法线和搜索树。
        const double fine_distance=std::min(max_distance,0.3);
        if (!refine(fine_distance)) return 0;
        // 点云库在此参数上比较的是平方距离。
        const double mse=icp.getFitnessScore(fine_distance*fine_distance);
        if (!std::isfinite(mse)) return 0;
        Eigen::Quaterniond q(matrix.block<3,3>(0,0).cast<double>());q.normalize();
        result->transformation={{matrix(0,3),matrix(1,3),matrix(2,3)},{q.x(),q.y(),q.z(),q.w()}};
        result->mse=mse;result->iterations=used_iterations;result->converged=1;
        return 0;
    } catch (...) { return 1; }
}
