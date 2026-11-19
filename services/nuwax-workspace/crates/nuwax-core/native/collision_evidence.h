#pragma once
#include <stddef.h>
#include <stdint.h>

typedef struct {
    double x, y, bottom, top;
    uint32_t layer;
} CollisionEvidenceCell;

typedef struct {
    double x, y, ground;
    size_t count;
    CollisionEvidenceCell cells[10];
} CollisionEvidence;

#ifdef __cplusplus
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

enum CollisionLayer : uint32_t { OccupiedVoxel, SurfaceContact, BusinessForbidden };

inline thread_local CollisionEvidence* collision_evidence = nullptr;

// 采样仅在本次原生调用范围内生效，控制、规划线程各用自己的记录。
struct CollisionEvidenceScope {
    CollisionEvidence* previous;
    explicit CollisionEvidenceScope(CollisionEvidence* current) : previous(collision_evidence) {
        collision_evidence = current;
    }
    ~CollisionEvidenceScope() { collision_evidence = previous; }
};

inline void recordCollision(double x, double y, double bottom, double top, CollisionLayer layer) {
    auto* evidence = collision_evidence;
    if (!evidence) return;
    const auto key = [evidence](const CollisionEvidenceCell& cell) {
        const double dx = cell.x - evidence->x, dy = cell.y - evidence->y;
        return std::make_tuple(dx * dx + dy * dy, cell.x, cell.y, cell.bottom, cell.top, cell.layer);
    };
    const CollisionEvidenceCell cell{x, y, bottom, top, layer};
    const auto value = key(cell);
    auto* begin = evidence->cells;
    auto* end = begin + evidence->count;
    auto* at = std::lower_bound(begin, end, value,
        [&](const auto& old, const auto& rank) { return key(old) < rank; });
    if (at == begin + 10 || (at != end && key(*at) == value)) return;
    if (end == begin + 10) --end;
    else ++evidence->count;
    std::move_backward(at, end, end + 1);
    *at = cell;
}

inline void recordForbiddenCell(double x, double y) {
    const double infinity = std::numeric_limits<double>::infinity();
    recordCollision(x, y, -infinity, infinity, BusinessForbidden);
}

inline void recordForbiddenPoint(double x, double y, double resolution) {
    if (collision_evidence)
        recordForbiddenCell((std::floor(x / resolution) + .5) * resolution,
                            (std::floor(y / resolution) + .5) * resolution);
}
#endif
