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

namespace pcl {

// 体素坐标和强度继续用 PCL（点云库）的质心，时间用整数均值取最近纳秒。
template <>
class CentroidPoint<nuwax_lio::PointLioPoint> {
public:
    void add(const nuwax_lio::PointLioPoint& point) {
        centroid_.add(PointXYZI(point.x, point.y, point.z, point.intensity));
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
