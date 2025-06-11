#include <explore/costmap_tools.h>
#include <explore/frontier_search.h>

#include <geometry_msgs/msg/point.hpp>
#include <mutex>

#include "nav2_costmap_2d/cost_values.hpp"

namespace frontier_exploration
{
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

FrontierSearch::FrontierSearch(nav2_costmap_2d::Costmap2D* costmap,
                               double potential_scale, double gain_scale,
                               double min_frontier_size)
  : costmap_(costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , min_frontier_size_(min_frontier_size)
{
}

std::vector<Frontier>
FrontierSearch::searchFrom(geometry_msgs::msg::Point position,
                           geometry_msgs::msg::Point human_position)
{
  std::vector<Frontier> frontier_list;

  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    RCLCPP_ERROR(rclcpp::get_logger("FrontierSearch"),
                 "Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  std::queue<unsigned int> bfs;

  unsigned int clear, pos = costmap_->getIndex(mx, my);
  if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
    RCLCPP_WARN(rclcpp::get_logger("FrontierSearch"),
                "Could not find nearby clear cell to start search");
  }

  visited_flag[bfs.front()] = true;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    for (unsigned nbr : nhood4(idx, *costmap_)) {
      if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag, human_position,position);
        if (new_frontier.size * costmap_->getResolution() >= min_frontier_size_) {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }

  // Filter out human-near frontiers
  // eliminateHumanFrontiers(frontier_list, position, human_position);

  // Assign costs
  for (auto& frontier : frontier_list) {
    frontier.cost = frontierCost(frontier);
  }

  std::sort(frontier_list.begin(), frontier_list.end(),
            [](const Frontier& f1, const Frontier& f2) {
              return f1.cost < f2.cost;
            });

  return frontier_list;
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          std::vector<bool>& frontier_flag,
                                          const geometry_msgs::msg::Point& human_position,
                                          const geometry_msgs::msg::Point& robot_position)
{
  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 1;
  output.min_distance = std::numeric_limits<double>::infinity();

  // record initial contact point for frontier
  unsigned int ix, iy;
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
    for (unsigned int nbr : nhood8(idx, *costmap_)) {
      // check if neighbour is a potential frontier cell
      if (isNewFrontierCell(nbr, frontier_flag)) {
        // mark cell as frontier
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        geometry_msgs::msg::Point point;
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
  // calculate distance to human and robot
  double dx_human = output.centroid.x - human_position.x;
  double dy_human = output.centroid.y - human_position.y;
  double dist_to_human = sqrt(dx_human * dx_human + dy_human * dy_human);

  double dx_robot = output.centroid.x - robot_position.x;
  double dy_robot = output.centroid.y - robot_position.y;
  double dist_to_robot = sqrt(dx_robot * dx_robot + dy_robot * dy_robot);

  // assign closeto field
  output.closeto = (dist_to_robot < dist_to_human) ? 1 : 0;
  // RCLCPP_INFO(rclcpp::get_logger("FrontierSearch"),
  //              "Frontier x: %f y: %f | human x: %f y: %f | robot x: %f y: %f",
  //              output.centroid.x, output.centroid.y,
  //              human_position.x, human_position.y,
  //              robot_position.x, robot_position.y);
  // RCLCPP_INFO(rclcpp::get_logger("FrontierSearch"),
  //              "Frontier distance to human: %f, robot: %f sorting to %d",
  //              dist_to_human, dist_to_robot,output.closeto);
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

void FrontierSearch::eliminateHumanFrontiers(std::vector<Frontier>& frontiers,
  const geometry_msgs::msg::Point& robot_position,
  const geometry_msgs::msg::Point& human_position)
{
frontiers.erase(
std::remove_if(frontiers.begin(), frontiers.end(),
[&](const Frontier& f) {
double fx = f.centroid.x;
double fy = f.centroid.y;

double dist_to_robot = std::hypot(fx - robot_position.x, fy - robot_position.y);
double dist_to_human = std::hypot(fx - human_position.x, fy - human_position.y);

return dist_to_human < dist_to_robot;
}),
frontiers.end());
}

bool FrontierSearch::isCloserToHuman(const Frontier& f,
  const geometry_msgs::msg::Point& robot_position,
  const geometry_msgs::msg::Point& human_position)
{
double fx = f.centroid.x;
double fy = f.centroid.y;

double dist_to_robot = std::hypot(fx - robot_position.x, fy - robot_position.y);
double dist_to_human = std::hypot(fx - human_position.x, fy - human_position.y);

return dist_to_human < dist_to_robot;
}

double FrontierSearch::frontierCost(const Frontier& frontier)
{
  return (potential_scale_ * frontier.min_distance *
          costmap_->getResolution()) -
         (gain_scale_ * frontier.size * costmap_->getResolution());
}
}  // namespace frontier_exploration
