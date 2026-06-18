#pragma once

#include <set>

namespace rmgo_referee::ui {

template <typename T, typename Compare>
class RedBlackTree {
public:
    bool empty() const noexcept { return tree_.empty(); }
    void clear() noexcept { tree_.clear(); }
    void insert(T& value) { tree_.insert(&value); }
    void erase(T& value) noexcept { tree_.erase(&value); }

    T* first() const noexcept { return tree_.empty() ? nullptr : *tree_.begin(); }

    T* next(const T& value) const noexcept {
        const auto iterator = tree_.upper_bound(const_cast<T*>(&value));
        return iterator == tree_.end() ? nullptr : *iterator;
    }

private:
    std::set<T*, Compare> tree_;
};

} // namespace rmgo_referee::ui
