// 本地无 ROS 提取与真实感知/机身适配；改动范围和上游归属见 native/NOTICE。
#include "path_searching/dyn_a_star.h"
#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std;
using namespace Eigen;

AStar::~AStar() = default;

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    POOL_SIZE_=pool_size;
    CENTER_IDX_=pool_size/2;
    grid_map_=occ_map;
    grid_nodes_.clear();
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

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx, double start_yaw)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    Eigen::Vector3d start_to_end = end_pt - start_pt;
    if (start_to_end.norm() < 1e-6)
        return false;
    const double path_yaw = std::atan2(start_to_end(1), start_to_end(0));
    start_to_end.normalize();

    const double actual_start_yaw=std::isfinite(start_yaw)?start_yaw:path_yaw;
    int occ = checkOccupancy(start_pt, actual_start_yaw);
    if (occ)
    {
        //ROS_WARN("Start point is insdide an obstacle.");
        do
        {
            start_pt -= start_to_end * step_size_;
            if (!Coord2Index(start_pt, start_idx))
                return false;

            occ = checkOccupancy(Index2Coord(start_idx), actual_start_yaw);
            if (occ == -1)
            {
                SCAN_WARN( "[Astar] Start point outside the map region.");
                return false;
            }
        } while (occ);
    }

    occ = checkOccupancy(Index2Coord(end_idx), path_yaw);
    if (occ)
    {
        //ROS_WARN("End point is insdide an obstacle.");
        do
        {
            end_pt += start_to_end * step_size_;
            if (!Coord2Index(end_pt, end_idx))
                return false;

            occ = checkOccupancy(Index2Coord(end_idx), path_yaw);
            if (occ == -1)
            {
                SCAN_WARN( "[Astar] End point outside the map region.");
                return false;
            }
        } while (occ);
    }

    return true;
}

ASTAR_RET AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt, double start_yaw)
{
    const auto time_1 = std::chrono::steady_clock::now();
    ++rounds_;
    grid_nodes_.clear();
    gridPath_.clear();

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;
    // 搜索位置与已有占据格心同相位，保留窄门中的真实可行通道。
    center_.x()=(std::floor(center_.x()/step_size_)+0.5)*step_size_;
    center_.y()=(std::floor(center_.y()/step_size_)+0.5)*step_size_;

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx, start_yaw))
    {
        SCAN_ERROR(
                     "Unable to handle the initial or end point, force return!");
        return ASTAR_RET::INIT_ERR;
    }

    const Eigen::Vector3d search_start = Index2Coord(start_idx);
    const Eigen::Vector3d search_end = Index2Coord(end_idx);
    const Eigen::Vector2d search_start_xy = search_start.head<2>();
    const Eigen::Vector2d search_xy_delta = search_end.head<2>() - search_start_xy;
    const double search_xy_len2 = search_xy_delta.squaredNorm();

    auto interpolateZIndexOnSearchPlane = [&](const int x_idx, const int y_idx, const double previous_z) -> int {
        if (search_xy_len2 < 1e-8)
            return start_idx(2);

        Eigen::Vector3i sample_idx(x_idx, y_idx, start_idx(2));
        const Eigen::Vector2d sample_xy = Index2Coord(sample_idx).head<2>();
        double ratio = (sample_xy - search_start_xy).dot(search_xy_delta) / search_xy_len2;
        ratio = std::max(0.0, std::min(1.0, ratio));

        const double reference_z = search_start(2) + ratio * (search_end(2) - search_start(2));
        Eigen::Vector3d terrain_sample(sample_xy.x(),sample_xy.y(),previous_z);
        double ground;
        // 相邻搜索格按当前层的实测地形接续；起终点弦高会把平台与楼梯折点压入实体。
        // 缺测列仍使用已有参考高度，不由地面缺测拒绝搜索。
        const double z = grid_map_->measuredGround(terrain_sample,ground)
            ? grid_map_->bodyHeight(terrain_sample) : reference_z;
        return static_cast<int>(std::floor((z - center_(2)) * inv_step_size_ + 0.5)) + CENTER_IDX_(2);
    };

    // 起终点和所有邻接使用同一实测高度索引；否则坡面终点永远不可能命中。
    start_idx(2)=interpolateZIndexOnSearchPlane(start_idx(0),start_idx(1),start_pt.z());
    end_idx(2)=interpolateZIndexOnSearchPlane(end_idx(0),end_idx(1),end_pt.z());

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cerr << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = gridNode(start_idx);
    GridNodePtr endPtr = gridNode(end_idx);

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

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
        //     cerr << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
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
                neighborIdx(2) = interpolateZIndexOnSearchPlane(
                    neighborIdx(0), neighborIdx(1),
                    current == startPtr ? start_pt.z() : Index2Coord(current->index).z());

                if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                {
                    continue;
                }

                neighborPtr = gridNode(neighborIdx);
                neighborPtr->index = neighborIdx;

                bool flag_explored = neighborPtr->rounds == rounds_;

                if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                {
                    continue; //in closed set.
                }

                // 起点只代表实测位置；第一条边的航向由实际起点到格心求出。
                const Eigen::Vector3d from=current==startPtr?start_pt:Index2Coord(current->index);
                const Eigen::Vector3d to=Index2Coord(neighborPtr->index);
                const double neighbor_yaw = std::atan2(to.y()-from.y(),to.x()-from.x());
                if (!grid_map_->traversable(from,to) || checkOccupancy(to, neighbor_yaw))
                {
                    continue;
                }
                // 矩形机身的占据取决于入边航向；失败入边不能把同格的其他可行方向标成已发现。
                neighborPtr->rounds = rounds_;

                const int dz = neighborIdx(2) - current->index(2);
                double static_cost = current==startPtr?(to-from).norm()*inv_step_size_:sqrt(dx * dx + dy * dy + dz * dz);
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
            SCAN_WARN(
                        "Failed in A-star path search: 0.2 second time limit exceeded");
            return ASTAR_RET::SEARCH_ERR;
        }
    }

    const auto time_2 = std::chrono::steady_clock::now();

    const double elapsed = std::chrono::duration<double>(time_2 - time_1).count();
    if (elapsed > 0.1)
        SCAN_WARN(
                    "A-star path search took %.3fs, iter=%d", elapsed, num_iter);

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
