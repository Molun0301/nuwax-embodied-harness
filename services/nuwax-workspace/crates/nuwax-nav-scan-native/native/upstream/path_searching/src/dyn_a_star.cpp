#include "path_searching/dyn_a_star.h"
#include <author_diagnostics.h>
#include <algorithm>
#include <cmath>
#include <chrono>
// Host adaptation: planning elapsed-time budget uses monotonic wall time, not simulation time.

using namespace std;
using namespace Eigen;

AStar::~AStar() = default;

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    CENTER_IDX_ = pool_size / 2;
    node_capacity_ = static_cast<std::size_t>(pool_size.x()) * pool_size.y() * pool_size.z();
    openSet_ = decltype(openSet_){};
    gridPath_.clear();
    nodes_.clear();
    nodes_.reserve(static_cast<std::size_t>(pool_size.x()) * pool_size.y());
    grid_map_ = occ_map;
}

GridNodePtr AStar::nodeFor(const Eigen::Vector3i& index)
{
    const std::array<int, 3> key{index.x(), index.y(), index.z()};
    auto found = nodes_.find(key);
    if (found != nodes_.end()) return &found->second;
    if (nodes_.size() >= node_capacity_) return nullptr;
    auto inserted = nodes_.try_emplace(key);
    inserted.first->second.index = index;
    return &inserted.first->second;
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx, double start_yaw, double end_yaw)
{
    start_idx = Coord2Index(start_pt);
    end_idx = Coord2Index(end_pt);

    Eigen::Vector3d start_to_end = end_pt - start_pt;
    if (start_to_end.norm() < 1e-6)
        return false;
    start_to_end.normalize();

    int occ = checkOccupancy(Index2Coord(start_idx), start_yaw);
    if (occ)
    {
        //ROS_WARN("Start point is insdide an obstacle.");
        do
        {
            start_pt -= start_to_end * step_size_;
            start_idx = Coord2Index(start_pt);

            occ = checkOccupancy(Index2Coord(start_idx), start_yaw);
            if (occ == -1)
            {
                ROS_WARN("[Astar] Start point outside the map region.");
                return false;
            }
        } while (occ);
    }

    occ = checkOccupancy(Index2Coord(end_idx), end_yaw);
    if (occ)
    {
        //ROS_WARN("End point is insdide an obstacle.");
        do
        {
            end_pt += start_to_end * step_size_;
            end_idx = Coord2Index(end_pt);

            occ = checkOccupancy(Index2Coord(end_idx), end_yaw);
            if (occ == -1)
            {
                ROS_WARN("[Astar] End point outside the map region.");
                return false;
            }
        } while (occ);
    }

    return true;
}

ASTAR_RET AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt, double start_yaw, double end_yaw)
{
    const auto time_1 = std::chrono::steady_clock::now();
    ++rounds_;
    // 仅本轮节点可被当前格原点解释，旧路径指针先随原队列退休。
    openSet_ = decltype(openSet_){};
    gridPath_.clear();
    nodes_.clear();

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    // 仅显式诊断时记录原失败原因；不改变搜索、时间或节点预算。
    const auto trace_failure = [&](const char* reason, int iterations) {
        if (!scan_planner::authorTraceEnabled()) return;
        auto out = scan_planner::authorTraceStream();
        out << "{\"event\":\"astar_failure\",\"reason\":\"" << reason
            << "\",\"iterations\":" << iterations << ",\"step\":" << step_size_
            << ",\"start_yaw\":" << start_yaw << ",\"end_yaw\":" << end_yaw
            << ",\"detail\":" << std::quoted(scan_native_diagnostic)
            << ",\"start\":";
        scan_planner::traceMatrix(out, start_pt);
        out << ",\"end\":"; scan_planner::traceMatrix(out, end_pt);
        const auto snapped = [&](const Vector3d& point) -> Vector3d {
            return ((((point - center_) * inv_step_size_).array() + .5).floor() * step_size_).matrix() + center_;
        };
        out << ",\"start_occupancy\":" << checkOccupancy(start_pt, start_yaw)
            << ",\"end_occupancy\":" << checkOccupancy(end_pt, end_yaw)
            << ",\"snapped_start_occupancy\":" << checkOccupancy(snapped(start_pt), start_yaw)
            << ",\"snapped_end_occupancy\":" << checkOccupancy(snapped(end_pt), end_yaw);
        out << "}\n";
    };

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx, start_yaw, end_yaw))
    {
        trace_failure("endpoints", 0);
        ROS_ERROR("Unable to handle the initial or end point, force return!");
        return ASTAR_RET::INIT_ERR;
    }

    const Eigen::Vector3d search_start = Index2Coord(start_idx);
    const Eigen::Vector3d search_end = Index2Coord(end_idx);
    const Eigen::Vector2d search_start_xy = search_start.head<2>();
    const Eigen::Vector2d search_xy_delta = search_end.head<2>() - search_start_xy;
    const double search_xy_len2 = search_xy_delta.squaredNorm();

    auto interpolateZIndexOnSearchPlane = [&](const int x_idx, const int y_idx) -> int {
        if (search_xy_len2 < 1e-8)
            return start_idx(2);

        Eigen::Vector3i sample_idx(x_idx, y_idx, start_idx(2));
        const Eigen::Vector2d sample_xy = Index2Coord(sample_idx).head<2>();
        double ratio = (sample_xy - search_start_xy).dot(search_xy_delta) / search_xy_len2;
        ratio = std::max(0.0, std::min(1.0, ratio));

        double z = search_start(2) + ratio * (search_end(2) - search_start(2));
        if(route_height_query_) {
            // Same ordered-route height as initialization. Preserve adjusted
            // search endpoint heights with only their boundary residuals.
            z=route_height_query_(ratio)
              +(1.-ratio)*(search_start.z()-route_height_query_(0.))
              +ratio*(search_end.z()-route_height_query_(1.));
        }
        return static_cast<int>(std::floor((z - center_(2)) * inv_step_size_ + 0.5)) + CENTER_IDX_(2);
    };

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     std::cerr << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = nodeFor(start_idx);
    GridNodePtr endPtr = nodeFor(end_idx);
    if (!startPtr || !endPtr) {
        ROS_WARN("A star node budget exhausted.");
        trace_failure("node_budget", 0);
        return ASTAR_RET::SEARCH_ERR;
    }

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    endPtr->index = end_idx;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr); //put start in open set

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        // if ( num_iter < 10000 )
        //     std::cerr << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            // const auto time_2 = std::chrono::steady_clock::now();
            // std::fprintf(stderr,"\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, std::chrono::duration<double>(time_2 - time_1).count()*1000);
            // if(std::chrono::duration<double>(time_2 - time_1).count() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", std::chrono::duration<double>(time_2 - time_1).count() );
            gridPath_ = retrievePath(current);
            return ASTAR_RET::SUCCESS;
        }
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
            {
                if (dx == 0 && dy == 0)
                    continue;

                Vector3i neighborIdx;
                neighborIdx(0) = (current->index)(0) + dx;
                neighborIdx(1) = (current->index)(1) + dy;
                neighborIdx(2) = interpolateZIndexOnSearchPlane(neighborIdx(0), neighborIdx(1));

                neighborPtr = nodeFor(neighborIdx);
                if (!neighborPtr) {
                    ROS_WARN("A star node budget exhausted.");
                    trace_failure("node_budget", num_iter);
                    return ASTAR_RET::SEARCH_ERR;
                }
                neighborPtr->index = neighborIdx;

                bool flag_explored = neighborPtr->rounds == rounds_;

                if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                {
                    continue; //in closed set.
                }

                const double neighbor_yaw = std::atan2(static_cast<double>(dy), static_cast<double>(dx));
                if (checkOccupancy(Index2Coord(neighborPtr->index), neighbor_yaw))
                {
                    continue;
                }
                // 一个方向碰撞不能把该位置标为已发现；其它合法入边仍必须可以入队。
                neighborPtr->rounds = rounds_;

                const int dz = neighborIdx(2) - current->index(2);
                double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                tentative_gScore = current->gScore + static_cost;

                if (!flag_explored)
                {
                    //discover a new node
                    neighborPtr->state = GridNode::OPENSET;
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    openSet_.push(neighborPtr); //put neighbor in open set and record it.
                }
                else if (tentative_gScore < neighborPtr->gScore)
                { //in open set and need update
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                }
            }
        const auto time_2 = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(time_2 - time_1).count() > 0.2)
        {
            ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            trace_failure("budget", num_iter);
            return ASTAR_RET::SEARCH_ERR;
        }
    }

    const auto time_2 = std::chrono::steady_clock::now();

    if (std::chrono::duration<double>(time_2 - time_1).count() > 0.1)
        ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", std::chrono::duration<double>(time_2 - time_1).count(), num_iter);

    trace_failure("exhausted", num_iter);
    return ASTAR_RET::SEARCH_ERR;
}

vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}

