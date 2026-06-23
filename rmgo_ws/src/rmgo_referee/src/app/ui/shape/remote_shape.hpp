#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

#include "app/ui/shape/red_black_tree.hpp"

namespace rmgo_referee::ui {

template <typename ShapeT>
class RemoteShape {
private:
    static constexpr std::uint8_t id_assignment_max = 201;

public:
    class Allocator;

    class Descriptor : private RedBlackTree<Descriptor>::Node {
    public:
        friend class RemoteShape;
        friend class RedBlackTree<Descriptor>;
        friend class Allocator;

        Descriptor() = default;
        Descriptor(const Descriptor&) = delete;
        Descriptor& operator=(const Descriptor&) = delete;
        Descriptor(Descriptor&&) = delete;
        Descriptor& operator=(Descriptor&&) = delete;

        bool has_id() const noexcept { return id_ != 0; }

        bool swapping_enabled() const noexcept {
            return !RedBlackTree<Descriptor>::Node::is_dangling();
        }

        std::uint8_t id() const noexcept { return id_; }
        std::uint8_t existence_confidence() const noexcept { return existence_confidence_; }

    private:
        friend ShapeT;

        void mark_id_revoked() {
            id_ = 0;
            existence_confidence_ = 0;
            static_cast<ShapeT*>(this)->id_revoked();
        }

        bool operator<(const Descriptor& other) const noexcept {
            if (existence_confidence_ != other.existence_confidence_) {
                return existence_confidence_ < other.existence_confidence_;
            }
            return this < &other;
        }

        std::uint8_t id_ = 0;
        std::uint8_t existence_confidence_ = 0;
    };

    class Allocator {
    public:
        bool try_assign_id(Descriptor& descriptor) {
            if (descriptor.has_id()) {
                return false;
            }

            if (auto* victim = swapping_queue_.first(); victim != nullptr) {
                swapping_queue_.erase(*victim);
                swap_id(descriptor, *victim);
                return true;
            }

            return assign_id(descriptor);
        }

        bool predict_try_assign_id(
            const Descriptor& descriptor, std::uint8_t& existence_confidence) const {
            if (descriptor.has_id()) {
                return false;
            }

            if (auto* victim = swapping_queue_.first(); victim != nullptr) {
                existence_confidence = victim->existence_confidence_;
                return true;
            }

            return has_available_id();
        }

        void enable_swapping(Descriptor& descriptor) {
            if (descriptor.swapping_enabled()) {
                return;
            }
            swapping_queue_.insert(descriptor);
        }

        void disable_swapping(Descriptor& descriptor) noexcept {
            if (!descriptor.swapping_enabled()) {
                return;
            }
            swapping_queue_.erase(descriptor);
        }

        void revoke_id(Descriptor& descriptor) {
            disable_swapping(descriptor);
            const auto revoked_id = descriptor.id_;
            if (revoked_id != 0 && assigned_list_[revoked_id - 1] == &descriptor) {
                assigned_list_[revoked_id - 1] = nullptr;
                next_id_ = std::min(next_id_, revoked_id);
            }
            descriptor.mark_id_revoked();
        }

        std::uint8_t increase_existence_confidence(Descriptor& descriptor) {
            ++descriptor.existence_confidence_;
            if (descriptor.swapping_enabled()) {
                disable_swapping(descriptor);
                enable_swapping(descriptor);
            }
            return descriptor.existence_confidence_;
        }

        void force_revoke_all_id() {
            for (auto* descriptor : assigned_list_) {
                if (descriptor != nullptr) {
                    revoke_id(*descriptor);
                }
            }
            assigned_list_.fill(nullptr);
            swapping_queue_.clear();
            next_id_ = 1;
        }

    private:
        bool assign_id(Descriptor& descriptor) {
            if (assign_first_available_id(descriptor, next_id_, id_assignment_max + 1U)) {
                return true;
            }
            return assign_first_available_id(descriptor, 1U, next_id_);
        }

        void swap_id(Descriptor& descriptor, Descriptor& victim) {
            descriptor.id_ = victim.id_;
            descriptor.existence_confidence_ = victim.existence_confidence_;
            assigned_list_[descriptor.id_ - 1] = &descriptor;
            revoke_id(victim);
        }

        bool assign_first_available_id(
            Descriptor& descriptor, std::uint16_t begin, std::uint16_t end) {
            for (auto id = begin; id < end; ++id) {
                if (assigned_list_[id - 1] != nullptr) {
                    continue;
                }
                descriptor.id_ = static_cast<std::uint8_t>(id);
                assigned_list_[descriptor.id_ - 1] = &descriptor;
                next_id_ = static_cast<std::uint8_t>(id + 1U);
                return true;
            }
            return false;
        }

        bool has_available_id() const noexcept {
            for (const auto* descriptor : assigned_list_) {
                if (descriptor == nullptr) {
                    return true;
                }
            }
            return false;
        }

        std::uint8_t next_id_ = 1;
        std::array<Descriptor*, id_assignment_max> assigned_list_{};
        RedBlackTree<Descriptor> swapping_queue_;
    };
};

} // namespace rmgo_referee::ui
