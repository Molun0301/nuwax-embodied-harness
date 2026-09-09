#include "solver.h"
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <cmath>
#include <limits>

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
        icp.setMaximumIterations(static_cast<int>(iterations));icp.setMaxCorrespondenceDistance(max_distance);
        icp.setTransformationEpsilon(1e-8);icp.setEuclideanFitnessEpsilon(1e-8);
        pcl::PointCloud<Point> aligned;icp.align(aligned);
        const auto matrix=icp.getFinalTransformation();
        if (!icp.hasConverged() || !matrix.allFinite()) return 0;
        const double mse=icp.getFitnessScore(max_distance);
        if (!std::isfinite(mse)) return 0;
        Eigen::Quaterniond q(matrix.block<3,3>(0,0).cast<double>());q.normalize();
        result->transformation={{matrix(0,3),matrix(1,3),matrix(2,3)},{q.x(),q.y(),q.z(),q.w()}};
        result->mse=mse;result->iterations=static_cast<std::uint32_t>(icp.iterations());result->converged=1;
        return 0;
    } catch (...) { return 1; }
}
