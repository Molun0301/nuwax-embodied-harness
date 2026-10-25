#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <author_host.h>

#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>
#include <limits>
#include <array>
#include <unordered_map>

constexpr double inf = std::numeric_limits<double>::infinity();
struct GridNode;
typedef GridNode *GridNodePtr;

enum ASTAR_RET
{
	SUCCESS,
	INIT_ERR,
	SEARCH_ERR
};

struct GridNode
{
	enum enum_state
	{
		OPENSET = 1,
		CLOSEDSET = 2,
		UNDEFINED = 3
	};

	int rounds{0}; // Distinguish every call
	enum enum_state state
	{
		UNDEFINED
	};
	Eigen::Vector3i index;

	double gScore{inf}, fScore{inf};
	GridNodePtr cameFrom{NULL};
};

class NodeComparator
{
public:
	bool operator()(GridNodePtr node1, GridNodePtr node2)
	{
		return node1->fScore > node2->fScore;
	}
};

class AStar
{
private:
	GridMap::Ptr grid_map_;

	inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

	double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
	double getManhHeu(GridNodePtr node1, GridNodePtr node2);
	double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
	inline double getHeu(GridNodePtr node1, GridNodePtr node2);

	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx, double start_yaw, double end_yaw);

	inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
	inline Eigen::Vector3i Coord2Index(const Eigen::Vector3d &pt) const;

	//bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );

	inline int checkOccupancy(const Eigen::Vector3d &pos, const double yaw) { return grid_map_->getInflateOccupancy(pos, yaw); }

	std::vector<GridNodePtr> retrievePath(GridNodePtr current);

	double step_size_, inv_step_size_;
	Eigen::Vector3d center_;
	Eigen::Vector3i CENTER_IDX_;
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

	std::vector<GridNodePtr> gridPath_;

	struct NodeKeyHash {
		std::size_t operator()(const std::array<int, 3>& key) const noexcept {
			std::size_t hash = 0;
			for (int value : key) hash ^= std::hash<int>{}(value) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
			return hash;
		}
	};
	// 原节点总容量保持；按实际格坐标寻址，不把存储阵列形状当成物理范围。
	std::size_t node_capacity_{0};
	std::unordered_map<std::array<int, 3>, GridNode, NodeKeyHash> nodes_;
	GridNodePtr nodeFor(const Eigen::Vector3i& index);
	std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

	int rounds_{0};

public:
    std::function<double(double)> route_height_query_;
	typedef std::shared_ptr<AStar> Ptr;

	AStar(){};
	~AStar();

	void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);

	ASTAR_RET AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, double start_yaw, double end_yaw);

	std::vector<Eigen::Vector3d> getPath();
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
	return tie_breaker_ * getDiagHeu(node1, node2);
}

inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const
{
	return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

inline Eigen::Vector3i AStar::Coord2Index(const Eigen::Vector3d &pt) const
{
	// 负半轴也取最近格，向零截断会把端点错误地推远一格。
	return (((pt - center_) * inv_step_size_).array() + 0.5).floor().cast<int>().matrix() + CENTER_IDX_;
};

#endif

