#include "plan_env/grid_map.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

GridMap2D::GridMap2D(double resolution, const Eigen::Vector2i& world_size)
    : resolution_(resolution), world_size_(world_size) {
    if (!std::isfinite(resolution) || resolution <= 0 || (world_size.array() <= 0).any())
        throw std::invalid_argument("Map resolution and dimensions must be positive");
    map_size_ = (world_size.cast<double>() / resolution).array().ceil().cast<int>();
    raw_.resize(static_cast<std::size_t>(map_size_.x()) * map_size_.y());
    inflated_.resize(raw_.size());
    rebuildOffsets();
    setCurPose(0, 0);
}

void GridMap2D::setCurPose(double x, double y) {
    if (!std::isfinite(x) || !std::isfinite(y))
        throw std::invalid_argument("Map centre must be finite");
    origin_ = Eigen::Vector2d(x, y) - world_size_.cast<double>() * 0.5;
    // A changed origin invalidates all previously indexed obstacles.
    resetMap();
}
void GridMap2D::resetMap() {
    std::fill(raw_.begin(), raw_.end(), 0);
    std::fill(inflated_.begin(), inflated_.end(), 0);
}
std::size_t GridMap2D::address(const Eigen::Vector2i& i) const {
    return static_cast<std::size_t>(i.y()) * map_size_.x() + i.x();
}
bool GridMap2D::isIndexValid(const Eigen::Vector2i& i) const {
    return (i.array() >= 0).all() && (i.array() < map_size_.array()).all();
}
Eigen::Vector2i GridMap2D::worldToGrid(const Eigen::Vector2d& p) const {
    if (!p.allFinite()) return Eigen::Vector2i::Constant(-1);
    const Eigen::Vector2d local = (p - origin_) / resolution_;
    if ((local.array() < 0).any() || (local.array() >= map_size_.cast<double>().array()).any())
        return Eigen::Vector2i::Constant(-1);
    return local.array().floor().cast<int>();
}
Eigen::Vector2d GridMap2D::gridToWorld(const Eigen::Vector2i& i) const {
    return origin_ + (i.cast<double>() + Eigen::Vector2d::Constant(0.5)) * resolution_;
}
void GridMap2D::setObstacle(const Eigen::Vector2i& i, bool occupied) {
    if (!isIndexValid(i)) return;
    raw_[address(i)] = occupied;
    inflated_[address(i)] = occupied;
    // Call inflate() after batching changes, including deletions.
}
void GridMap2D::setInflateRadius(double radius) {
    if (!std::isfinite(radius) || radius < 0)
        throw std::invalid_argument("Circle radius must be finite and nonnegative");
    inflate_radius_ = radius;
    rebuildOffsets();
    inflate();
}
void GridMap2D::setDoubleCircleOffsets(double front_offset, double rear_offset) {
    if (!std::isfinite(front_offset) || front_offset < 0 ||
        !std::isfinite(rear_offset) || rear_offset < 0)
        throw std::invalid_argument("Circle offsets must be finite and nonnegative");
    front_circle_offset_ = front_offset;
    rear_circle_offset_ = rear_offset;
}
void GridMap2D::rebuildOffsets() {
    inflation_offsets_.clear();
    const int steps = static_cast<int>(std::ceil(inflate_radius_ / resolution_)) + 1;
    for (int x = -steps; x <= steps; ++x)
        for (int y = -steps; y <= steps; ++y) {
            // Minimum distance between the two cell squares. Conservatively
            // cover every possible obstacle/query position inside their cells.
            const double dx = std::max(0, std::abs(x) - 1) * resolution_;
            const double dy = std::max(0, std::abs(y) - 1) * resolution_;
            if ((inflate_radius_ == 0 && x == 0 && y == 0) ||
                (inflate_radius_ > 0 && std::hypot(dx, dy) <= inflate_radius_))
                inflation_offsets_.emplace_back(x, y);
        }
}
void GridMap2D::inflateObstacles(double radius) {
    if (!std::isfinite(radius) || radius < 0)
        throw std::invalid_argument("Circle radius must be finite and nonnegative");
    if (radius != inflate_radius_) {
        inflate_radius_ = radius;
        rebuildOffsets();
    }
    inflated_ = raw_;
    for (int y = 0; y < map_size_.y(); ++y)
        for (int x = 0; x < map_size_.x(); ++x) {
            const Eigen::Vector2i i(x, y);
            if (!raw_[address(i)]) continue;
            for (const auto& offset : inflation_offsets_) {
                const Eigen::Vector2i j = i + offset;
                if (isIndexValid(j)) inflated_[address(j)] = 1;
            }
        }
}
GridMap2D::CellState GridMap2D::query(const Eigen::Vector2d& p,
                                     const std::vector<unsigned char>& buffer) const {
    const auto i = worldToGrid(p);
    if (!isIndexValid(i)) return CellState::kOutOfMap;
    return buffer[address(i)] ? CellState::kOccupied : CellState::kFree;
}
GridMap2D::CellState GridMap2D::queryRaw(const Eigen::Vector2d& p) const { return query(p, raw_); }
GridMap2D::CellState GridMap2D::queryInflated(const Eigen::Vector2d& p) const {
    if (!p.allFinite()) return CellState::kOutOfMap;
    // The entire circle must remain inside the known local map.
    const Eigen::Vector2d local = p - origin_;
    const Eigen::Vector2d size = map_size_.cast<double>() * resolution_;
    if ((local.array() < inflate_radius_).any() ||
        (local.array() + inflate_radius_ >= size.array()).any()) return CellState::kOutOfMap;
    return query(p, inflated_);
}
GridMap2D::CellState GridMap2D::queryDoubleCircle(const Eigen::Vector2d& world_position, double yaw) const {
    if (!std::isfinite(yaw)) return CellState::kOutOfMap;
    const Eigen::Vector2d heading(std::cos(yaw), std::sin(yaw));
    const Eigen::Vector2d front = world_position + front_circle_offset_ * heading;
    const Eigen::Vector2d rear = world_position - rear_circle_offset_ * heading;
    // std::cout << "GridMap2D::queryDoubleCircle front = " << front << " rear = " << rear << std::endl;
    // 与 SCAN 一致：先查询前圆，前圆空闲时再查询后圆。
    const CellState front_state = queryInflated(front);
    if (front_state != CellState::kFree) {
        return front_state;
    }
    return queryInflated(rear);
}
bool GridMap2D::isObstacle(const Eigen::Vector2d& p) const { return queryRaw(p) != CellState::kFree; }
bool GridMap2D::getInflateOccupancy(const Eigen::Vector2d& p) const { return queryInflated(p) != CellState::kFree; }
bool GridMap2D::getInflateOccupancy(const Eigen::Vector2d& p, double yaw) const {
    return queryDoubleCircle(p, yaw) != CellState::kFree;
}
std::vector<Eigen::Vector2d> GridMap2D::getObstaclePointCloud(bool inflated) const {
    std::vector<Eigen::Vector2d> points;
    const auto& buffer = inflated ? inflated_ : raw_;
    for (int y = 0; y < map_size_.y(); ++y)
        for (int x = 0; x < map_size_.x(); ++x)
            if (buffer[address(Eigen::Vector2i(x, y))]) points.push_back(gridToWorld(Eigen::Vector2i(x, y)));
    return points;
}
