#pragma once

#include <cstdint>
#include <type_traits>

namespace rmgo_referee::ui {

#define RMGO_REFEREE_WRITE_ONCE(x, val) x = (val)

class BasicRedBlackTree {
public:
    enum class Color : std::uint8_t { red = 0, black = 1 };

    class Node {
    public:
        friend class BasicRedBlackTree;

        Node() = default;

        Color color() const {
            return static_cast<Color>(parent_and_color_ & static_cast<std::uintptr_t>(1));
        }
        bool is_red() const { return color() == Color::red; }
        bool is_black() const { return color() == Color::black; }

        Node* parent() const {
            return reinterpret_cast<Node*>(parent_and_color_ & ~static_cast<std::uintptr_t>(1));
        }

        Node* next() const {
            Node *parent = nullptr, *node = const_cast<Node*>(this);

            if (node->right_ != nullptr) {
                node = node->right_;
                while (node->left_ != nullptr) {
                    node = node->left_;
                }
                return node;
            }

            while ((parent = node->parent()) != nullptr && node == parent->right_) {
                node = parent;
            }
            return parent;
        }

        Node* prev() const {
            Node *parent = nullptr, *node = const_cast<Node*>(this);

            if (node->left_ != nullptr) {
                node = node->left_;
                while (node->right_ != nullptr) {
                    node = node->right_;
                }
                return node;
            }

            while ((parent = node->parent()) != nullptr && node == parent->left_) {
                node = parent;
            }
            return parent;
        }

        void set_parent_and_color(Node* parent, Color color) {
            parent_and_color_ =
                reinterpret_cast<std::uintptr_t>(parent) | static_cast<std::uintptr_t>(color);
        }

        void set_parent(Node* parent) { set_parent_and_color(parent, color()); }

        void set_red() { parent_and_color_ &= ~static_cast<std::uintptr_t>(1); }
        void set_black() { parent_and_color_ |= static_cast<std::uintptr_t>(1); }
        void set_color(Color color) {
            if (color == Color::red) {
                set_red();
            } else {
                set_black();
            }
        }

        Node* red_get_parent() const { return reinterpret_cast<Node*>(parent_and_color_); }

    public:
        std::uintptr_t parent_and_color_ = 0;
        Node* right_ = nullptr;
        Node* left_ = nullptr;
    };

    static void link_node(Node* node, Node* parent, Node** link) {
        node->set_parent_and_color(parent, Color::red);
        node->left_ = node->right_ = nullptr;
        *link = node;
    }

    void insert_color(Node* node) {
        Node *parent = node->parent(), *gparent = nullptr, *tmp = nullptr;

        while (true) {
            if (parent == nullptr) {
                node->set_parent_and_color(nullptr, Color::black);
                break;
            }

            if (parent->is_black()) {
                break;
            }

            gparent = parent->red_get_parent();

            tmp = gparent->right_;
            if (parent != tmp) {
                if (tmp != nullptr && tmp->is_red()) {
                    tmp->set_parent_and_color(gparent, Color::black);
                    parent->set_parent_and_color(gparent, Color::black);
                    node = gparent;
                    parent = node->parent();
                    node->set_parent_and_color(parent, Color::red);
                    continue;
                }

                tmp = parent->right_;
                if (node == tmp) {
                    tmp = node->left_;
                    RMGO_REFEREE_WRITE_ONCE(parent->right_, tmp);
                    RMGO_REFEREE_WRITE_ONCE(node->left_, parent);
                    if (tmp != nullptr) {
                        tmp->set_parent_and_color(parent, Color::black);
                    }
                    parent->set_parent_and_color(node, Color::red);
                    parent = node;
                    tmp = node->right_;
                }

                RMGO_REFEREE_WRITE_ONCE(gparent->left_, tmp);
                RMGO_REFEREE_WRITE_ONCE(parent->right_, gparent);
                if (tmp != nullptr) {
                    tmp->set_parent_and_color(gparent, Color::black);
                }
                rotate_set_parents(gparent, parent, Color::red);
                break;
            }

            tmp = gparent->left_;
            if (tmp != nullptr && tmp->is_red()) {
                tmp->set_parent_and_color(gparent, Color::black);
                parent->set_parent_and_color(gparent, Color::black);
                node = gparent;
                parent = node->parent();
                node->set_parent_and_color(parent, Color::red);
                continue;
            }

            tmp = parent->left_;
            if (node == tmp) {
                tmp = node->right_;
                RMGO_REFEREE_WRITE_ONCE(parent->left_, tmp);
                RMGO_REFEREE_WRITE_ONCE(node->right_, parent);
                if (tmp != nullptr) {
                    tmp->set_parent_and_color(parent, Color::black);
                }
                parent->set_parent_and_color(node, Color::red);
                parent = node;
                tmp = node->left_;
            }

            RMGO_REFEREE_WRITE_ONCE(gparent->right_, tmp);
            RMGO_REFEREE_WRITE_ONCE(parent->left_, gparent);
            if (tmp != nullptr) {
                tmp->set_parent_and_color(gparent, Color::black);
            }
            rotate_set_parents(gparent, parent, Color::red);
            break;
        }
    }

    void erase(Node* node) {
        Node* rebalance = erase_node(node);
        if (rebalance != nullptr) {
            erase_color(rebalance);
        }
    }

    Node* first() const {
        auto* node = root_;
        if (node == nullptr) {
            return nullptr;
        }
        while (node->left_ != nullptr) {
            node = node->left_;
        }
        return node;
    }

    Node* last() const {
        auto* node = root_;
        if (node == nullptr) {
            return nullptr;
        }
        while (node->right_ != nullptr) {
            node = node->right_;
        }
        return node;
    }

    Node* root_ = nullptr;

private:
    void change_child(Node* old_node, Node* new_node, Node* parent) {
        if (parent != nullptr) {
            if (parent->left_ == old_node) {
                RMGO_REFEREE_WRITE_ONCE(parent->left_, new_node);
            } else {
                RMGO_REFEREE_WRITE_ONCE(parent->right_, new_node);
            }
        } else {
            RMGO_REFEREE_WRITE_ONCE(root_, new_node);
        }
    }

    void rotate_set_parents(Node* old_node, Node* new_node, Color color) {
        Node* parent = old_node->parent();
        new_node->parent_and_color_ = old_node->parent_and_color_;
        old_node->set_parent_and_color(new_node, color);
        change_child(old_node, new_node, parent);
    }

    Node* erase_node(Node* node) {
        Node* child = node->right_;
        Node* tmp = node->left_;
        Node *parent = nullptr, *rebalance = nullptr;
        std::uintptr_t parent_color = 0;

        if (tmp == nullptr) {
            parent_color = node->parent_and_color_;
            parent = reinterpret_cast<Node*>(parent_color & ~static_cast<std::uintptr_t>(3));
            change_child(node, child, parent);
            if (child != nullptr) {
                child->parent_and_color_ = parent_color;
                rebalance = nullptr;
            } else {
                rebalance = (parent_color & 1U) != 0U ? parent : nullptr;
            }
            tmp = parent;
        } else if (child == nullptr) {
            tmp->parent_and_color_ = parent_color = node->parent_and_color_;
            parent = reinterpret_cast<Node*>(parent_color & ~static_cast<std::uintptr_t>(3));
            change_child(node, tmp, parent);
            rebalance = nullptr;
            tmp = parent;
        } else {
            Node *successor = child, *child2 = nullptr;

            tmp = child->left_;
            if (tmp == nullptr) {
                parent = successor;
                child2 = successor->right_;
            } else {
                do {
                    parent = successor;
                    successor = tmp;
                    tmp = tmp->left_;
                } while (tmp != nullptr);
                child2 = successor->right_;
                RMGO_REFEREE_WRITE_ONCE(parent->left_, child2);
                RMGO_REFEREE_WRITE_ONCE(successor->right_, child);
                child->set_parent(successor);
            }

            tmp = node->left_;
            RMGO_REFEREE_WRITE_ONCE(successor->left_, tmp);
            tmp->set_parent(successor);

            parent_color = node->parent_and_color_;
            tmp = reinterpret_cast<Node*>(parent_color & ~static_cast<std::uintptr_t>(3));
            change_child(node, successor, tmp);

            if (child2 != nullptr) {
                child2->set_parent_and_color(parent, Color::black);
                rebalance = nullptr;
            } else {
                rebalance = successor->is_black() ? parent : nullptr;
            }
            successor->parent_and_color_ = parent_color;
            tmp = successor;
        }

        return rebalance;
    }

    void erase_color(Node* parent) {
        Node *node = nullptr, *sibling = nullptr, *tmp1 = nullptr, *tmp2 = nullptr;

        while (true) {
            sibling = parent->right_;
            if (node != sibling) {
                if (sibling->is_red()) {
                    tmp1 = sibling->left_;
                    RMGO_REFEREE_WRITE_ONCE(parent->right_, tmp1);
                    RMGO_REFEREE_WRITE_ONCE(sibling->left_, parent);
                    tmp1->set_parent_and_color(parent, Color::black);
                    rotate_set_parents(parent, sibling, Color::red);
                    sibling = tmp1;
                }
                tmp1 = sibling->right_;
                if (tmp1 == nullptr || tmp1->is_black()) {
                    tmp2 = sibling->left_;
                    if (tmp2 == nullptr || tmp2->is_black()) {
                        sibling->set_parent_and_color(parent, Color::red);
                        if (parent->is_red()) {
                            parent->set_black();
                        } else {
                            node = parent;
                            parent = node->parent();
                            if (parent != nullptr) {
                                continue;
                            }
                        }
                        break;
                    }
                    tmp1 = tmp2->right_;
                    RMGO_REFEREE_WRITE_ONCE(sibling->left_, tmp1);
                    RMGO_REFEREE_WRITE_ONCE(tmp2->right_, sibling);
                    RMGO_REFEREE_WRITE_ONCE(parent->right_, tmp2);
                    if (tmp1 != nullptr) {
                        tmp1->set_parent_and_color(sibling, Color::black);
                    }
                    tmp1 = sibling;
                    sibling = tmp2;
                }
                tmp2 = sibling->left_;
                RMGO_REFEREE_WRITE_ONCE(parent->right_, tmp2);
                RMGO_REFEREE_WRITE_ONCE(sibling->left_, parent);
                tmp1->set_parent_and_color(sibling, Color::black);
                if (tmp2 != nullptr) {
                    tmp2->set_parent(parent);
                }
                rotate_set_parents(parent, sibling, Color::black);
                break;
            }

            sibling = parent->left_;
            if (sibling->is_red()) {
                tmp1 = sibling->right_;
                RMGO_REFEREE_WRITE_ONCE(parent->left_, tmp1);
                RMGO_REFEREE_WRITE_ONCE(sibling->right_, parent);
                tmp1->set_parent_and_color(parent, Color::black);
                rotate_set_parents(parent, sibling, Color::red);
                sibling = tmp1;
            }
            tmp1 = sibling->left_;
            if (tmp1 == nullptr || tmp1->is_black()) {
                tmp2 = sibling->right_;
                if (tmp2 == nullptr || tmp2->is_black()) {
                    sibling->set_parent_and_color(parent, Color::red);
                    if (parent->is_red()) {
                        parent->set_black();
                    } else {
                        node = parent;
                        parent = node->parent();
                        if (parent != nullptr) {
                            continue;
                        }
                    }
                    break;
                }
                tmp1 = tmp2->left_;
                RMGO_REFEREE_WRITE_ONCE(sibling->right_, tmp1);
                RMGO_REFEREE_WRITE_ONCE(tmp2->left_, sibling);
                RMGO_REFEREE_WRITE_ONCE(parent->left_, tmp2);
                if (tmp1 != nullptr) {
                    tmp1->set_parent_and_color(sibling, Color::black);
                }
                tmp1 = sibling;
                sibling = tmp2;
            }
            tmp2 = sibling->right_;
            RMGO_REFEREE_WRITE_ONCE(parent->left_, tmp2);
            RMGO_REFEREE_WRITE_ONCE(sibling->right_, parent);
            tmp1->set_parent_and_color(sibling, Color::black);
            if (tmp2 != nullptr) {
                tmp2->set_parent(parent);
            }
            rotate_set_parents(parent, sibling, Color::black);
            break;
        }
    }
};

template <typename T>
class RedBlackTree final {
public:
    class Node : private BasicRedBlackTree::Node {
    public:
        friend class RedBlackTree;

        Node() { set_dangling(); }

        bool is_dangling() const { return BasicRedBlackTree::Node::parent_and_color_ == 0; }

        T* next() const {
            return static_cast<T*>(static_cast<Node*>(BasicRedBlackTree::Node::next()));
        }

        T* prev() const {
            return static_cast<T*>(static_cast<Node*>(BasicRedBlackTree::Node::prev()));
        }

    private:
        void set_dangling() { BasicRedBlackTree::Node::parent_and_color_ = 0; }
    };

    bool insert(T& node) requires(std::is_base_of_v<Node, T>) {
        if (!static_cast<Node&>(node).is_dangling()) {
            return false;
        }

        BasicRedBlackTree::Node** link = &tree_.root_;
        BasicRedBlackTree::Node* parent = nullptr;

        while (*link != nullptr) {
            parent = *link;

            auto& current = *static_cast<T*>(static_cast<Node*>(*link));
            if (node < current) {
                link = &((*link)->left_);
            } else {
                link = &((*link)->right_);
            }
        }

        tree_.link_node(static_cast<Node*>(&node), parent, link);
        tree_.insert_color(static_cast<Node*>(&node));
        return true;
    }

    bool erase(T& node) requires(std::is_base_of_v<Node, T>) {
        if (static_cast<Node&>(node).is_dangling()) {
            return false;
        }

        tree_.erase(static_cast<Node*>(&node));
        static_cast<Node&>(node).set_dangling();
        return true;
    }

    void clear() requires(std::is_base_of_v<Node, T>) {
        while (auto* node = first()) {
            erase(*node);
        }
    }

    bool empty() const requires(std::is_base_of_v<Node, T>) { return tree_.root_ == nullptr; }

    T* first() const requires(std::is_base_of_v<Node, T>) {
        return static_cast<T*>(static_cast<Node*>(tree_.first()));
    }

    T* last() const requires(std::is_base_of_v<Node, T>) {
        return static_cast<T*>(static_cast<Node*>(tree_.last()));
    }

private:
    BasicRedBlackTree tree_;
};

#undef RMGO_REFEREE_WRITE_ONCE

} // namespace rmgo_referee::ui
