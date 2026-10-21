// 原始来源及许可见组件 NOTICE。
#include "a_star/a_star_search.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>

using std::cout;
using std::endl;

// 9 neighbors in 2d
static std::vector<Eigen::Vector2i> kNeighbors = std::vector<Eigen::Vector2i>{
    Eigen::Vector2i(-1, -1), Eigen::Vector2i(-1, 0), Eigen::Vector2i(-1, 1),
    Eigen::Vector2i(0, -1),  Eigen::Vector2i(0, 1),  Eigen::Vector2i(1, -1),
    Eigen::Vector2i(1, 0),   Eigen::Vector2i(1, 1),
};

void Astar::Init(const double cost_threshold, const int num_layers,
                 const double resolution,  const double step_cost_weight, const Eigen::MatrixXd& cost_map,
                 const Eigen::MatrixXd& height_map,
                 const Eigen::MatrixXd& ele_map) {
  cost_threshold_ = cost_threshold;
  resolution_ = resolution;
step_cost_weight_  = step_cost_weight;

  max_x_ = cost_map.cols();
  max_y_ = cost_map.rows() / num_layers;
  max_layers_ = num_layers;
  xy_size_ = max_x_ * max_y_;

  int row_offset = 0;
  grid_map_.resize(max_layers_);
  for (int i = 0; i < max_layers_; ++i) {
    row_offset = i * max_y_;
    grid_map_[i].resize(max_y_);
    for (int j = 0; j < max_y_; ++j) {
      grid_map_[i][j].clear();
      for (int k = 0; k < max_x_; ++k) {
        double height = height_map(j + row_offset, k);
        const double cost = cost_map(j + row_offset, k);
        const double ele = ele_map(j + row_offset, k);
        if (height == -100.0 && cost == 100.0 && ele == 0.0) continue;
        double z = static_cast<int>(height / resolution);
        auto& node = grid_map_[i][j].emplace(k, Node(Eigen::Vector3i(z, j, k), nullptr)).first->second;
        node.cost = cost;
        node.height = height;
        node.ele = ele;
        node.layer = i;
      }
    }
  }

  search_layers_offset_.clear();
  search_layers_offset_.emplace_back(0);
  for (int i = 0; i < search_layer_depth_; ++i) {
    search_layers_offset_.emplace_back(-(i + 1));
    search_layers_offset_.emplace_back(i + 1);
  }


}

void Astar::Reset() {
  search_result_.clear();
  expanded_ = 0;
  was_cancelled_ = false;
  for (size_t i = 0; i < grid_map_.size(); ++i) {
    for (size_t j = 0; j < grid_map_[i].size(); ++j) {
      for (auto& entry : grid_map_[i][j]) entry.second.Reset();
    }
  }
}

Node* Astar::FindNode(int layer, int row, int column) {
  auto& nodes = grid_map_[layer][row];
  const auto found = nodes.find(column);
  return found == nodes.end() ? nullptr : &found->second;
}

std::int64_t Astar::GetHash(const Node* node) const {
  // 使用实际层号，避免不同层的接近高度被原来的高度哈希合并。
  return (static_cast<std::int64_t>(node->layer) * max_y_ + node->idx[1]) * max_x_ + node->idx[2];
}

bool Astar::Search(const Eigen::Vector3i& start, const Eigen::Vector3i& goal) {
  // 失败也可能更新过节点代价；每次搜索都必须清除上次状态。
  Reset();
  auto start_node = FindNode(start[0], start[2], start[1]);
  auto goal_node = FindNode(goal[0], goal[2], goal[1]);
  if (!start_node || !goal_node) return false;
  if (start_node->cost > cost_threshold_ || goal_node->cost > cost_threshold_) return false;
  start_node->g = 0.0;
  start_node->f = GetHeuristic(start_node, goal_node);
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, NodeCompare> open_set;
  std::unordered_map<std::int64_t, Node*> closed_set;
  std::uint64_t sequence = 0;
  open_set.push({start_node, start_node->f, start_node->g, sequence++});
  while (!open_set.empty()) {
    if (cancelled_ && cancelled_()) { was_cancelled_ = true; return false; }
    const auto entry = open_set.top();
    open_set.pop();
    Node* current_node = entry.node;
    if (entry.g != current_node->g) continue;
    auto closed = closed_set.find(GetHash(current_node));
    if (closed != closed_set.end() && closed->second->g < entry.g) continue;
    ++expanded_;
    if (current_node == goal_node) {
      while (current_node != nullptr) {
        search_result_.emplace_back(current_node);
        current_node = current_node->parent;
      }
      std::reverse(search_result_.begin(), search_result_.end());
      if (debug_) ConvertClosedSetToMatrix(closed_set);
      return true;
    }
    closed_set[GetHash(current_node)] = current_node;
    const int preferred_layer = DecideLayer(current_node);
    for (const auto& neighbor : kNeighbors) {
      const int i = current_node->idx[1] + neighbor[0];
      const int j = current_node->idx[2] + neighbor[1];
      if (i < 0 || i >= max_y_ || j < 0 || j >= max_x_) continue;
      // 保留原层切换优先级，同时检查相邻位置的有效实测层，连接连续地形与台阶。
      for (int pass = 0; pass <= max_layers_; ++pass) {
        const int layer = pass == 0 ? preferred_layer : pass - 1;
        if (pass != 0 && layer == preferred_layer) continue;
        auto neighbor_node = FindNode(layer, i, j);
        if (!neighbor_node) continue;
        if (neighbor_node->cost > cost_threshold_) continue;
        if (std::abs(neighbor_node->height - current_node->height) > height_limit_) continue;
        if (transition_ && !transition_(current_node, neighbor_node)) continue;
        const auto diff = neighbor_node->idx - current_node->idx;
        double step_cost = step_cost_weight_ * neighbor_node->cost;
        if (step_cost < 5) step_cost = 0.0;
        const double dz = (neighbor_node->height - current_node->height) / resolution_;
        const double tentative_g = current_node->g +
            std::sqrt(dz * dz + diff[1] * diff[1] + diff[2] * diff[2]) + step_cost;
        if (tentative_g < neighbor_node->g) {
          neighbor_node->g = tentative_g;
          neighbor_node->f = tentative_g + GetHeuristic(neighbor_node, goal_node);
          neighbor_node->parent = current_node;
          open_set.push({neighbor_node, neighbor_node->f, neighbor_node->g, sequence++});
        }
      }
    }
  }
  if (debug_) ConvertClosedSetToMatrix(closed_set);
  return false;
}

int Astar::DecideLayer(const Node* cur_node) const {
  int layer = cur_node->layer;
  int i = cur_node->idx[1];
  int j = cur_node->idx[2];
  double cur_height = cur_node->height;

  int true_layer = layer;

  for (const auto offset : search_layers_offset_) {
    int cur_layer = layer + offset;

    if (cur_layer < 0 || cur_layer >= max_layers_) {
      continue;
    }

    const auto found = grid_map_[cur_layer][i].find(j);
    if (found == grid_map_[cur_layer][i].end()) continue;
    const Node& search_node = found->second;

    if (abs(search_node.height - cur_height) > 0.2) {
      continue;
    }

    if (search_node.ele > 0.5) {
      true_layer = std::min(cur_layer + 1, max_layers_ - 1);
      break;
    } else if (search_node.ele < -0.5) {
      true_layer = std::max(cur_layer - 1, 0);
      break;
    }
  }

  return true_layer;
}

double Astar::CalculateStepCost(const Node* node1, const Node* node2) const {
  (void)node1;
  (void)node2;
  return 0.0;
}

double Astar::GetHeuristic(const Node* node1, const Node* node2) const {
  const double dx = node1->idx[2] - node2->idx[2];
  const double dy = node1->idx[1] - node2->idx[1];
  const double dz = (node1->height - node2->height) / resolution_;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<PathPoint> Astar::GetPathPoints() const {
  std::vector<PathPoint> path_points;

  auto size = search_result_.size();
  path_points.resize(size);

  if (size == 0) {
    printf("path is empty\n, convert to path points failed\n");
    return path_points;
  }

  for (size_t i = 0; i < size; ++i) {
    // path_points[i].layer = search_result_[i][0];
    // path_points[i].x = search_result_[i][2];
    // path_points[i].y = search_result_[i][1];
    // if (i > 0) {
    //   path_points[i].heading =
    //       std::atan2(search_result_[i][1] - search_result_[i - 1][1],
    //                  search_result_[i][2] - search_result_[i - 1][2]);
    // }
    path_points[i].layer = search_result_[i]->layer;
    path_points[i].x = search_result_[i]->idx(2);
    path_points[i].y = search_result_[i]->idx(1);
    path_points[i].height = search_result_[i]->height;
    if (i > 0) {
      path_points[i].heading =
          std::atan2(search_result_[i]->idx(1) - search_result_[i - 1]->idx(1),
                     search_result_[i]->idx(2) - search_result_[i - 1]->idx(2));
    }
  }

  if (size > 1) {
    path_points[0].heading = path_points[1].heading;
  }

  return path_points;
}

Eigen::MatrixXd Astar::GetResultMatrix() const {
  if (search_result_.empty()) {
    printf("path is empty\n, convert to matrix failed\n");
    return Eigen::MatrixXd();
  }

  Eigen::MatrixXd path_matrix(search_result_.size(), 3);
  for (size_t i = 0; i < search_result_.size(); ++i) {
    path_matrix(i, 0) = search_result_[i]->layer;
    path_matrix(i, 1) = search_result_[i]->idx[1];
    path_matrix(i, 2) = search_result_[i]->idx[2];
  }
  return path_matrix;
}

void Astar::ConvertClosedSetToMatrix(
    const std::unordered_map<std::int64_t, Node*>& closed_set) {
  visited_set_ = Eigen::MatrixXi(closed_set.size(), 3);
  int count = 0;
  for (auto i = closed_set.begin(); i != closed_set.end(); ++i) {
    visited_set_(count, 0) = i->second->layer;
    visited_set_(count, 1) = i->second->idx[1];
    visited_set_(count, 2) = i->second->idx[2];
    count += 1;
  }
}

std::vector<Eigen::Vector3i> Astar::GetNeighbors(Node* node) const {
  (void)node;
  return std::vector<Eigen::Vector3i>();
}

Eigen::MatrixXd Astar::GetCostLayer(int layer) const {
  Eigen::MatrixXd cost_layer = Eigen::MatrixXd::Constant(max_y_, max_x_, 100.0);
  for (int i = 0; i < max_y_; ++i) {
    for (const auto& entry : grid_map_[layer][i]) cost_layer(i, entry.first) = entry.second.cost;
  }
  return cost_layer;
}
Eigen::MatrixXd Astar::GetEleLayer(int layer) const {
  Eigen::MatrixXd ele_layer = Eigen::MatrixXd::Zero(max_y_, max_x_);
  for (int i = 0; i < max_y_; ++i) {
    for (const auto& entry : grid_map_[layer][i]) ele_layer(i, entry.first) = entry.second.ele;
  }
  return ele_layer;
}
