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
    class Entity {
    public:
        bool queued() const noexcept { return queued_; }

    private:
        void clear_queued() noexcept {
            queued_ = false;
            queued_weight_ = 0;
        }

        friend class CfsScheduler<ShapeT>;

        static constexpr std::uint64_t initial_vruntime = 65536;
        std::uint64_t vruntime_ = initial_vruntime;
        std::uint16_t queued_weight_ = 0;
        bool queued_ = false;
    };

    bool empty() const noexcept { return run_queue_.empty(); }
    void clear() noexcept;
    void clear_entity(ShapeT& shape) noexcept;
    void insert(ShapeT& shape);
    void erase(ShapeT& shape) noexcept;
    ShapeT* next(std::optional<FrameKind> frame_kind, std::span<ShapeT* const> ignored) const;
    void select(ShapeT& shape);

    static constexpr std::uint64_t vruntime_period = 65536;

private:
    struct ShapeCompare {
        bool operator()(const ShapeT* lhs, const ShapeT* rhs) const noexcept;
    };

    RedBlackTree<ShapeT, ShapeCompare> run_queue_;
    std::uint64_t min_vruntime_ = 0;
};

template <typename ShapeT>
inline bool CfsScheduler<ShapeT>::ShapeCompare::operator()(
    const ShapeT* lhs, const ShapeT* rhs) const noexcept {
    const auto& lhs_entity = static_cast<const Entity&>(*lhs);
    const auto& rhs_entity = static_cast<const Entity&>(*rhs);
    if (lhs_entity.vruntime_ != rhs_entity.vruntime_) {
        return lhs_entity.vruntime_ < rhs_entity.vruntime_;
    }
    if (lhs->priority() != rhs->priority()) {
        return lhs->priority() > rhs->priority();
    }
    return lhs < rhs;
}

template <typename ShapeT>
inline void CfsScheduler<ShapeT>::clear() noexcept {
    for (auto* shape = run_queue_.first(); shape != nullptr; shape = run_queue_.next(*shape)) {
        auto& entity = static_cast<Entity&>(*shape);
        entity.clear_queued();
    }
    run_queue_.clear();
    min_vruntime_ = 0;
}

template <typename ShapeT>
inline void CfsScheduler<ShapeT>::clear_entity(ShapeT& shape) noexcept {
    auto& entity = static_cast<Entity&>(shape);
    entity.clear_queued();
}

template <typename ShapeT>
inline void CfsScheduler<ShapeT>::insert(ShapeT& shape) {
    auto& entity = static_cast<Entity&>(shape);
    const auto weight = shape.scheduling_weight();
    if (entity.queued_) {
        if (entity.queued_weight_ != weight) {
            if (entity.queued_weight_ > weight) {
                entity.vruntime_ += static_cast<std::uint64_t>(entity.queued_weight_ - weight);
            } else {
                const auto delta = static_cast<std::uint64_t>(weight - entity.queued_weight_);
                entity.vruntime_ =
                    delta < entity.vruntime_ ? entity.vruntime_ - delta : std::uint64_t{0};
            }
            run_queue_.erase(shape);
            entity.vruntime_ = std::max(entity.vruntime_, min_vruntime_);
            entity.queued_weight_ = weight;
            run_queue_.insert(shape);
        }
        return;
    }

    entity.vruntime_ = std::max(entity.vruntime_, min_vruntime_);
    entity.queued_weight_ = weight;
    entity.queued_ = true;
    run_queue_.insert(shape);
}

template <typename ShapeT>
inline void CfsScheduler<ShapeT>::erase(ShapeT& shape) noexcept {
    auto& entity = static_cast<Entity&>(shape);
    if (!entity.queued_) {
        return;
    }
    run_queue_.erase(shape);
    entity.clear_queued();
}

template <typename ShapeT>
inline ShapeT* CfsScheduler<ShapeT>::next(
    std::optional<FrameKind> frame_kind, std::span<ShapeT* const> ignored) const {
    for (auto* shape = run_queue_.first(); shape != nullptr; shape = run_queue_.next(*shape)) {
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
    if (!entity.queued_) {
        return;
    }
    min_vruntime_ = entity.vruntime_;
    const auto shift = std::max<std::uint64_t>(1, vruntime_period - entity.queued_weight_);
    entity.vruntime_ += shift;
    erase(shape);
}

} // namespace rmgo_referee::ui
