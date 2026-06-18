#pragma once

#include <array>
#include <cstdint>

#include "app/ui/shape/red_black_tree.hpp"

namespace rmgo_referee::ui {

template <typename ShapeT>
class RemoteShape {
public:
    class Descriptor;

private:
    struct DescriptorCompare {
        bool operator()(const Descriptor* lhs, const Descriptor* rhs) const noexcept;
    };

public:
    class Descriptor {
    public:
        Descriptor() = default;
        Descriptor(const Descriptor&) = delete;
        Descriptor& operator=(const Descriptor&) = delete;

        bool has_id() const noexcept { return id_ != 0; }

        bool try_assign_id() {
            if (has_id()) {
                return false;
            }

            if (auto* victim = swapping_queue_.first(); victim != nullptr) {
                swapping_queue_.erase(*victim);
                victim->swapping_ = false;
                swap_id(*victim);
                return true;
            }

            if (next_id_ > id_assignment_max) {
                return false;
            }
            assign_id();
            return true;
        }

        bool predict_try_assign_id(std::uint8_t& existence_confidence) const {
            if (has_id()) {
                return false;
            }

            if (auto* victim = swapping_queue_.first(); victim != nullptr) {
                existence_confidence = victim->existence_confidence_;
                return true;
            }

            return next_id_ <= id_assignment_max;
        }

        bool swapping_enabled() const noexcept { return swapping_; }

        void enable_swapping() {
            if (swapping_) {
                return;
            }
            swapping_queue_.insert(*this);
            swapping_ = true;
        }

        void disable_swapping() noexcept {
            if (!swapping_) {
                return;
            }
            swapping_queue_.erase(*this);
            swapping_ = false;
        }

        std::uint8_t id() const noexcept { return id_; }
        std::uint8_t existence_confidence() const noexcept { return existence_confidence_; }

        std::uint8_t increase_existence_confidence() {
            ++existence_confidence_;
            if (swapping_) {
                disable_swapping();
                enable_swapping();
            }
            return existence_confidence_;
        }

    private:
        friend class RemoteShape<ShapeT>;
        friend struct DescriptorCompare;

        void assign_id() {
            id_ = next_id_++;
            assigned_list_[id_ - 1] = this;
        }

        void swap_id(Descriptor& victim) {
            id_ = victim.id_;
            existence_confidence_ = victim.existence_confidence_;
            assigned_list_[id_ - 1] = this;
            victim.revoke_id();
        }

        void revoke_id() {
            disable_swapping();
            id_ = 0;
            existence_confidence_ = 0;
            static_cast<ShapeT*>(this)->id_revoked();
        }

        std::uint8_t id_ = 0;
        std::uint8_t existence_confidence_ = 0;
        bool swapping_ = false;
    };

    static void force_revoke_all_id() {
        for (auto* descriptor : assigned_list_) {
            if (descriptor != nullptr) {
                descriptor->revoke_id();
            }
        }
        assigned_list_.fill(nullptr);
        swapping_queue_.clear();
        next_id_ = 1;
    }

private:
    static constexpr std::uint8_t id_assignment_max = 201;

    static inline std::uint8_t next_id_ = 1;
    static inline std::array<Descriptor*, id_assignment_max> assigned_list_{};
    static inline RedBlackTree<Descriptor, DescriptorCompare> swapping_queue_;
};

template <typename ShapeT>
inline bool RemoteShape<ShapeT>::DescriptorCompare::operator()(
    const Descriptor* lhs, const Descriptor* rhs) const noexcept {
    if (lhs->existence_confidence_ != rhs->existence_confidence_) {
        return lhs->existence_confidence_ < rhs->existence_confidence_;
    }
    return lhs < rhs;
}

} // namespace rmgo_referee::ui
