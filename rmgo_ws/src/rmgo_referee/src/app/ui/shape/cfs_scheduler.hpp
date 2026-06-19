#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>
#include <span>

#include "app/ui/shape/red_black_tree.hpp"
#include "app/ui/ui.hpp"

namespace rmgo_referee::ui {

template <typename ShapeT>
class CfsScheduler {
public:
    static constexpr std::uint64_t vruntime_period = 65536;

    class __attribute__((packed, aligned(sizeof(void*)))) Entity
        : private RedBlackTree<Entity>::Node {
    public:
        friend class CfsScheduler;
        friend class RedBlackTree<Entity>;

        bool queued() const noexcept { return is_in_run_queue(); }

    private:
        bool is_in_run_queue() const noexcept {
            return !RedBlackTree<Entity>::Node::is_dangling();
        }

        bool operator<(const Entity& other) const noexcept {
            if (vruntime_ != other.vruntime_) {
                return vruntime_ < other.vruntime_;
            }
            if (queued_weight_ != other.queued_weight_) {
                return queued_weight_ > other.queued_weight_;
            }
            return this < &other;
        }

        static constexpr std::uint64_t initial_vruntime = vruntime_period;
        std::uint64_t vruntime_ : 48 = initial_vruntime;
        std::uint16_t queued_weight_ = 0;
    };

    bool empty() const noexcept { return run_queue_.empty(); }
    void clear() noexcept;
    void clear_entity(ShapeT& shape) noexcept;
    void insert(ShapeT& shape);
    void erase(ShapeT& shape) noexcept;
    ShapeT* next(std::optional<FrameKind> frame_kind, std::span<ShapeT* const> ignored) const;
    void select(ShapeT& shape);

private:
    RedBlackTree<Entity> run_queue_;
    std::uint64_t min_vruntime_ = 0;
};

template <typename ShapeT>
inline void CfsScheduler<ShapeT>::clear() noexcept {
    while (auto* entity = run_queue_.first()) {
        run_queue_.erase(*entity);
    }
    min_vruntime_ = 0;
}

template <typename ShapeT>
inline void CfsScheduler<ShapeT>::clear_entity(ShapeT& shape) noexcept {
    erase(shape);
}

template <typename ShapeT>
inline void CfsScheduler<ShapeT>::insert(ShapeT& shape) {
    auto& entity = static_cast<Entity&>(shape);
    const auto weight = shape.scheduling_weight();
    if (entity.queued_weight_ != weight) {
        entity.vruntime_ += entity.queued_weight_;
        entity.vruntime_ -= weight;
        if (entity.vruntime_ < min_vruntime_) {
            entity.vruntime_ = min_vruntime_;
        }

        entity.queued_weight_ = weight;

        if (entity.is_in_run_queue()) {
            run_queue_.erase(entity);
        }
    } else if (entity.is_in_run_queue()) {
        return;
    }

    run_queue_.insert(entity);
}

template <typename ShapeT>
inline void CfsScheduler<ShapeT>::erase(ShapeT& shape) noexcept {
    auto& entity = static_cast<Entity&>(shape);
    if (entity.is_in_run_queue()) {
        run_queue_.erase(entity);
    }
}

template <typename ShapeT>
inline ShapeT* CfsScheduler<ShapeT>::next(
    std::optional<FrameKind> frame_kind, std::span<ShapeT* const> ignored) const {
    for (auto* entity = run_queue_.first(); entity != nullptr; entity = entity->next()) {
        auto* shape = static_cast<ShapeT*>(entity);
        if (std::find(ignored.begin(), ignored.end(), shape) != ignored.end()) {
            continue;
        }
        if (frame_kind.has_value() && shape->frame_kind() != *frame_kind) {
            continue;
        }
        return shape;
    }
    return nullptr;
}

template <typename ShapeT>
inline void CfsScheduler<ShapeT>::select(ShapeT& shape) {
    auto& entity = static_cast<Entity&>(shape);
    if (!entity.is_in_run_queue()) {
        return;
    }

    min_vruntime_ = entity.vruntime_;
    const auto shift = std::max<std::uint64_t>(1, vruntime_period - entity.queued_weight_);
    entity.vruntime_ += shift;
    run_queue_.erase(entity);
}

} // namespace rmgo_referee::ui
