// 原始来源及许可见组件 NOTICE；本地修改：取消、固定队列键、分层连接及物理边检查。
#pragma once

#include <Eigen/Core>
#include <unordered_map>
#include <functional>
#include <cstdint>
#include <vector>

#include "common/data_types.h"

enum HeuristicType : int { kEuclidean = 0, kManhattan = 1, kDiagonal = 2 };

class Node {
 public:
  Node() = default;
  Node(Eigen::Vector3i idx, Node* parent) : idx(idx), parent(parent) {}
  ~Node() = default;

  bool operator==(const Node& other) const { return idx == other.idx; }

  void Reset() {
    f = 0.0;
    g = 1e9;
    parent = nullptr;
  }

  double f = 1e9;
  double g = 1e9;
  double height = 0.0;
  double ele = 0;
  double cost = 0.0;
  int layer = 0;
  Eigen::Vector3i idx = Eigen::Vector3i(0, 0, 0);  // layer, row, col
  Node* parent = nullptr;
};

// 入队时冻结优先级，松弛节点不会修改队列里已入堆元素的键。
struct OpenEntry { Node* node; double f; double g; std::uint64_t sequence; };
struct NodeCompare {
  bool operator()(const OpenEntry& a, const OpenEntry& b) const {
    return a.f == b.f ? a.sequence > b.sequence : a.f > b.f;
  }
};

// 原空格的高度、代价、层切换分别为-100、100、0；按行稀疏存储其余节点。
using MultiLayerGridMap = std::vector<std::vector<std::unordered_map<int, Node>>>;

class Astar {
 public:
  Astar(const HeuristicType h_type = kDiagonal) : h_type_(h_type) {}
  ~Astar() = default;

  void Init(const double cost_threshold, const int num_layers,
            const double resolution, const double step_cost_weight,  const Eigen::MatrixXd& cost_map,
            const Eigen::MatrixXd& height_map, const Eigen::MatrixXd& ele_map);

  void Reset();
  void SetTransition(std::function<bool(const Node*, const Node*)> transition) {
    transition_ = std::move(transition);
  }
  void SetCancellation(std::function<bool()> cancelled) { cancelled_ = std::move(cancelled); }
  std::size_t Expanded() const { return expanded_; }
  bool Cancelled() const { return was_cancelled_; }
  void SetHeightLimit(double limit) { height_limit_ = limit; }


  void Debug() { debug_ = true; }

  bool Search(const Eigen::Vector3i& start, const Eigen::Vector3i& goal);

  std::vector<PathPoint> GetPathPoints() const;

  Eigen::MatrixXd GetResultMatrix() const;
  Eigen::MatrixXi GetVisitedSet() const { return visited_set_; }

  Eigen::MatrixXd GetCostLayer(int layer) const;
  Eigen::MatrixXd GetEleLayer(int layer) const;

 private:
  Node* FindNode(int layer, int row, int column);

  double CalculateStepCost(const Node* node1, const Node* node2) const;

  int DecideLayer(const Node* cur_node) const;

  std::int64_t GetHash(const Node* node) const;

  std::vector<Eigen::Vector3i> GetNeighbors(Node* node) const;

  double GetHeuristic(const Node* node1, const Node* node2) const;

  Eigen::MatrixXd PathToMatrix(const std::vector<Eigen::Vector3i>& path);

  void ToPathPoints(const std::vector<Eigen::Vector3i>& path,
                    std::vector<PathPoint>& path_points);

  void ConvertClosedSetToMatrix(
      const std::unordered_map<std::int64_t, Node*>& closed_set);

 private:
  std::function<bool(const Node*, const Node*)> transition_;
  std::function<bool()> cancelled_;
  std::size_t expanded_ = 0;
  bool was_cancelled_ = false;
  double height_limit_ = 0.3;
  HeuristicType h_type_ = kDiagonal;

  int max_x_ = 0;
  int max_y_ = 0;
  int max_layers_ = 0;
  int xy_size_ = 0;
  double resolution_ = 0;
  MultiLayerGridMap grid_map_;
  double cost_threshold_ = 35;
  double step_cost_weight_ = 1.0;

  int search_layer_depth_ = 1;
  std::vector<int> search_layers_offset_;

  bool debug_ = false;
  Eigen::MatrixXi visited_set_;

  // std::vector<Eigen::Vector3i> search_result_;
  std::vector<Node*> search_result_;
};
