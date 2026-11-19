#pragma once

#include <pcl/common/centroid.h>
#include <pcl/point_types.h>

#include "pointlio_time.hpp"

namespace nuwax_lio {

struct EIGEN_ALIGN16 PointLioPoint {
    PCL_ADD_POINT4D;
    float intensity = 0;
    SourceTimeNs offset_ns = 0;

    PointLioPoint() {
        x = y = z = 0;
        data[3] = 1;
    }

    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

}

POINT_CLOUD_REGISTER_POINT_STRUCT(nuwax_lio::PointLioPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
)

namespace faster_lio {

// 上游 iVox 头在点类型定义之后加载，先声明其转换模板。
template <typename T, int dim, typename PointType>
inline Eigen::Matrix<T, dim, 1> ToEigen(const PointType& pt);

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3, nuwax_lio::PointLioPoint>(
    const nuwax_lio::PointLioPoint& pt) {
    return pt.getVector3fMap();
}

}

namespace pcl {

// 体素坐标和强度继续用 PCL（点云库）的质心，时间用整数均值取最近纳秒。
template <>
class CentroidPoint<nuwax_lio::PointLioPoint> {
public:
    void add(const nuwax_lio::PointLioPoint& point) {
        // PCL 1.10 没有同时接收坐标和强度的构造函数。
        PointXYZI sample(point.intensity);
        sample.getVector3fMap() = point.getVector3fMap();
        centroid_.add(sample);
        offset_sum_ns_ += point.offset_ns;
    }

    void get(nuwax_lio::PointLioPoint& point) const {
        PointXYZI centroid;
        centroid_.get(centroid);
        point.getVector3fMap() = centroid.getVector3fMap();
        point.intensity = centroid.intensity;
        const auto count = static_cast<nuwax_lio::SourceTimeNs>(centroid_.getSize());
        point.offset_ns = (offset_sum_ns_ + count / 2) / count;
    }

private:
    CentroidPoint<PointXYZI> centroid_;
    nuwax_lio::SourceTimeNs offset_sum_ns_ = 0;
};

}
