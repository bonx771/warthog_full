#include <explore/frontier_search.h>    // Header khai báo class FrontierSearch

#include <mutex>                        // Dùng cho std::lock_guard

#include <costmap_2d/cost_values.h>   // Định nghĩa FREE_SPACE, NO_INFORMATION, LETHAL_OBSTACLE
#include <costmap_2d/costmap_2d.h>  // Class costmap2D
#include <geometry_msgs/Point.h>  //// Kiểu dữ liệu điểm (x,y,z)

#include <explore/costmap_tools.h>

namespace frontier_exploration
{
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

// ham khoi tao class
FrontierSearch::FrontierSearch(costmap_2d::Costmap2D* costmap,
                               double potential_scale, double gain_scale,
                               double min_frontier_size)
  : costmap_(costmap)                   // luu con tro trong costmap
  , potential_scale_(potential_scale) // he so khoang cach
  , gain_scale_(gain_scale)           // he so kich thuoc
  , min_frontier_size_(min_frontier_size) // nguong frontier nho nhat
{
}

//Trả về danh sách frontier, bắt đầu tìm từ vị trí robot
std::vector<Frontier> FrontierSearch::searchFrom(geometry_msgs::Point position)
{
  std::vector<Frontier> frontier_list;    // Danh sách frontier kết quả

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;  // Nếu ngoài map → trả về rỗng
  }
  // chuyển tọa độ world (m) sang grid (cell)
  // make sure map is consistent and locked for duration of search
  std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  // lay du lieu map
  map_ = costmap_->getCharMap();      // Mảng 1D lưu giá trị cost
  size_x_ = costmap_->getSizeInCellsX();  // Số cell theo trục X
  size_y_ = costmap_->getSizeInCellsY();  // Số cell theo trục Y

  // initialize flag arrays to keep track of visited and frontier cells
  // Đánh dấu cell nào đã thuộc frontier.
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  // Đánh dấu BFS đã duyệt cell nào.
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  // initialize breadth first search
  std::queue<unsigned int> bfs;

  // find closest clear cell to start search
  unsigned int clear, pos = costmap_->getIndex(mx, my); // pos = index 1D từ (mx,my)
  if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
    ROS_WARN("Could not find nearby clear cell to start search");
  }
  visited_flag[bfs.front()] = true;
  // Nếu tìm được cell FREE gần nhất, Bắt đầu BFS từ đó. Neu khong tim dược thì đánh dấu cell đầu tiên là đã thăm.
  while (!bfs.empty()) { //Lặp đến khi queue rỗng.
    unsigned int idx = bfs.front(); 
    bfs.pop();  //Lấy cell đầu queue.

    // iterate over 4-connected neighbourhood
    for (unsigned nbr : nhood4(idx, *costmap_)) {
      // nbr = 4 cell trên/dưới/trái/phải.
      // initialized on non-free cell
      if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
        // Điều kiện để lan BFS vùng FREE.
        visited_flag[nbr] = true;
        bfs.push(nbr);
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
        // neighbour)
        // Nếu là UNKNOWN + có neighbor FREE.
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;  //Đánh dấu đã thuộc frontier.
        // Gom cả cụm frontier.
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
        // Loại frontier nhỏ.
        if (new_frontier.size * costmap_->getResolution() >=
            min_frontier_size_) {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }

  // set costs of frontiers
  // tinh cost
  for (auto& frontier : frontier_list) {
    frontier.cost = frontierCost(frontier);
  }
  // sap xep: frontier tốt nhất ở đầu vector.
  std::sort(
      frontier_list.begin(), frontier_list.end(),
      [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

  return frontier_list;
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          std::vector<bool>& frontier_flag)
{
  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 1;
  output.min_distance = std::numeric_limits<double>::infinity();

  // record initial contact point for frontier
  unsigned int ix, iy;
  // Lấy tọa độ world của cell đầu tiên
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // cache reference position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // try adding cells in 8-connected neighborhood to frontier
    // Gom frontier theo 8 hướng.
    for (unsigned int nbr : nhood8(idx, *costmap_)) {
      // check if neighbour is a potential frontier cell
      if (isNewFrontierCell(nbr, frontier_flag)) {
        // mark cell as frontier
        // thêm vào frontier.
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        geometry_msgs::Point point;
        point.x = wx;
        point.y = wy;
        output.points.push_back(point);

        // update frontier size
        output.size++;

        // update centroid of frontier
        output.centroid.x += wx;
        output.centroid.y += wy;

        // determine frontier's distance from robot, going by closest gridcell
        // to robot
        double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                               pow((double(reference_y) - double(wy)), 2.0));
        if (distance < output.min_distance) {
          output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
        }

        // add to queue for breadth first search
        bfs.push(nbr);
      }
    }
  }

  // average out frontier centroid
  output.centroid.x /= output.size;
  output.centroid.y /= output.size;
  return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx,
                                       const std::vector<bool>& frontier_flag)
{
  // check that cell is unknown and not already marked as frontier
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood
  // that is free
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] == FREE_SPACE) {
      return true;
    }
  }

  return false;
}

double FrontierSearch::frontierCost(const Frontier& frontier)
{
  return (potential_scale_ * frontier.min_distance *
          costmap_->getResolution()) -
         (gain_scale_ * frontier.size * costmap_->getResolution());
}
}
