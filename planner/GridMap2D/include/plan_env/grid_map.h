/**
 * @file      grid_map.h
 * @brief     SCAN-Planner robo-centric local occupancy grid map.
 * @author    juchunyu <juchunyu@qq.com>
 * @date      2026-09-20 22:00:01
 * @copyright Copyright (c) 2025-2026 Institute of Robotics Planning and Control (IRPC).
 *            All rights reserved.
 * This library provides real-time occupancy queries for navigation and collision avoidance.
 */

#ifndef GRID_MAP_2D_H
#define GRID_MAP_2D_H

#include <Eigen/Dense>
#include <memory>
#include <vector>

// Pure 2D adaptation of SCAN's raw/inflated local occupancy map.
// origin() is the lower corner; mapSize() and indices are (x, y).
class GridMap2D {
public:
    enum class CellState { kOutOfMap = -1, kFree = 0, kOccupied = 1 };
    // world_size is in metres, retained for compatibility with the planner.
    GridMap2D(double resolution, const Eigen::Vector2i& world_size);
    void setCurPose(double x, double y);
    void resetMap();
    void resetGrids() { resetMap(); }
    void setObstacle(const Eigen::Vector2i& index, bool occupied = true);
    void inflateObstacles(double radius);
    void inflate() { inflateObstacles(inflate_radius_); }
    void setInflateRadius(double radius);
    void setDoubleCircleOffsets(double front_offset, double rear_offset);
    double circleRadius() const { return inflate_radius_; }
    CellState queryRaw(const Eigen::Vector2d& position) const;
    CellState queryInflated(const Eigen::Vector2d& position) const;
    CellState queryDoubleCircle(const Eigen::Vector2d& position, double yaw) const;
    // Out-of-map and invalid queries are occupied (fail closed).
    bool isObstacle(const Eigen::Vector2d& position) const;
    // Single circle-centre query for the geometric A* guide.
    bool getInflateOccupancy(const Eigen::Vector2d& position) const;
    bool getInflateOccupancy(const Eigen::Vector2d& position, double yaw) const;
    Eigen::Vector2i worldToGrid(const Eigen::Vector2d& position) const;
    Eigen::Vector2d gridToWorld(const Eigen::Vector2i& index) const;
    bool isIndexValid(const Eigen::Vector2i& index) const;
    std::vector<Eigen::Vector2d> getObstaclePointCloud(bool inflated = true) const;
    double resolution() const { return resolution_; }
    double getResolution() const { return resolution_; }
    const Eigen::Vector2i& mapSize() const { return map_size_; }
    const Eigen::Vector2d& origin() const { return origin_; }
private:
    std::size_t address(const Eigen::Vector2i& index) const;
    CellState query(const Eigen::Vector2d& position, const std::vector<unsigned char>& buffer) const;
    void rebuildOffsets();
    double resolution_;
    double inflate_radius_ = 0.5;
    double front_circle_offset_ = 0.5;
    double rear_circle_offset_ = 0.5;
    Eigen::Vector2i world_size_;
    Eigen::Vector2i map_size_ = Eigen::Vector2i::Zero();
    Eigen::Vector2d origin_ = Eigen::Vector2d::Zero();
    std::vector<unsigned char> raw_, inflated_;
    std::vector<Eigen::Vector2i> inflation_offsets_;
};
using GridMap2DPtr = std::shared_ptr<GridMap2D>;
#endif
