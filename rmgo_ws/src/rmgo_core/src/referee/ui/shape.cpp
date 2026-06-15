#include "referee/ui/ui_internal.hpp"

#include <algorithm>
#include <bit>
#include <cmath>

#include "referee/interaction.hpp"

namespace rmgo_core::referee::ui {

void CfsScheduler::clear() noexcept {
  run_queue_.clear();
  min_vruntime_ = 0;
}

void CfsScheduler::insert(Shape &shape) {
  const auto weight = shape.scheduling_weight();
  if (shape.queued_) {
    if (shape.queued_weight_ != weight) {
      if (shape.queued_weight_ > weight) {
        shape.vruntime_ +=
            static_cast<std::uint64_t>(shape.queued_weight_ - weight);
      } else {
        const auto delta =
            static_cast<std::uint64_t>(weight - shape.queued_weight_);
        shape.vruntime_ = delta < shape.vruntime_ ? shape.vruntime_ - delta
                                                  : std::uint64_t{0};
      }
      shape.vruntime_ = std::max(shape.vruntime_, min_vruntime_);
      shape.queued_weight_ = weight;
    }
    return;
  }

  shape.vruntime_ = std::max(shape.vruntime_, min_vruntime_);
  shape.queued_weight_ = weight;
  shape.queued_ = true;
  run_queue_.push_back(&shape);
}

void CfsScheduler::erase(Shape &shape) noexcept {
  if (!shape.queued_) {
    return;
  }
  std::erase(run_queue_, &shape);
  shape.queued_ = false;
}

Shape *CfsScheduler::next(std::optional<FrameKind> frame_kind,
                          std::span<Shape *const> ignored) const {
  auto best = run_queue_.end();
  for (auto candidate = run_queue_.begin(); candidate != run_queue_.end();
       ++candidate) {
    if (std::find(ignored.begin(), ignored.end(), *candidate) !=
        ignored.end()) {
      continue;
    }
    if (frame_kind.has_value() && (*candidate)->frame_kind() != *frame_kind) {
      continue;
    }
    if (best == run_queue_.end() ||
        (*candidate)->vruntime_ < (*best)->vruntime_ ||
        ((*candidate)->vruntime_ == (*best)->vruntime_ &&
         (*candidate)->priority() > (*best)->priority())) {
      best = candidate;
    }
  }
  if (best == run_queue_.end()) {
    return nullptr;
  }

  return *best;
}

void CfsScheduler::select(Shape &shape) {
  if (!shape.queued_) {
    return;
  }
  min_vruntime_ = shape.vruntime_;
  shape.on_selected_for_update();
  erase(shape);
}

void RemoteShapeRegistry::register_shape(Shape &shape) {
  shapes_.push_back(&shape);
}

void RemoteShapeRegistry::unregister_shape(Shape &shape) noexcept {
  disable_id_reuse(shape);
  std::erase(shapes_, &shape);
}

void RemoteShapeRegistry::reset_ids() noexcept {
  reusable_id_queue_.clear();
  next_id_ = 1;
}

bool RemoteShapeRegistry::assign_or_reuse_id(Shape &shape) {
  if (shape.id_ != 0) {
    return true;
  }

  if (!reusable_id_queue_.empty()) {
    const auto victim = std::min_element(
        reusable_id_queue_.begin(), reusable_id_queue_.end(),
        [](const Shape *lhs, const Shape *rhs) {
          return lhs->existence_confidence_ < rhs->existence_confidence_;
        });
    auto *reused = *victim;
    reusable_id_queue_.erase(victim);
    reused->reusable_id_ = false;

    shape.id_ = reused->id_;
    shape.existence_confidence_ = reused->existence_confidence_;
    reused->revoke_id();
    return shape.id_ != 0;
  }

  shape.id_ = assign_id();
  return shape.id_ != 0;
}

bool RemoteShapeRegistry::predict_assign_id(
    const Shape &shape, std::uint8_t &existence_confidence) const {
  if (shape.id_ != 0) {
    return true;
  }

  if (!reusable_id_queue_.empty()) {
    const auto victim = std::min_element(
        reusable_id_queue_.begin(), reusable_id_queue_.end(),
        [](const Shape *lhs, const Shape *rhs) {
          return lhs->existence_confidence_ < rhs->existence_confidence_;
        });
    existence_confidence = (*victim)->existence_confidence_;
    return true;
  }

  return next_id_ <= Shape::id_assignment_max;
}

void RemoteShapeRegistry::disable_id_reuse(Shape &shape) noexcept {
  if (!shape.reusable_id_) {
    return;
  }
  std::erase(reusable_id_queue_, &shape);
  shape.reusable_id_ = false;
}

void RemoteShapeRegistry::enable_id_reuse(Shape &shape) {
  if (shape.reusable_id_ || shape.id_ == 0 || shape.visible_) {
    return;
  }
  shape.reusable_id_ = true;
  reusable_id_queue_.push_back(&shape);
}

std::uint8_t RemoteShapeRegistry::assign_id() noexcept {
  if (next_id_ > Shape::id_assignment_max) {
    return 0;
  }
  const auto id = next_id_;
  ++next_id_;
  return id;
}

namespace {

constexpr auto vruntime_period = std::uint64_t{65536};

void append_graphic_description(std::span<std::byte> payload,
                                std::size_t &written, std::uint8_t name,
                                Operation operation, ShapeType type,
                                std::uint8_t layer, Color color,
                                std::uint16_t detail_a, std::uint16_t detail_b,
                                std::uint16_t width, std::uint16_t x,
                                std::uint16_t y, std::uint32_t detail_cde) {
  payload[written++] = static_cast<std::byte>(name);
  payload[written++] = std::byte{0xEF};
  payload[written++] = std::byte{0xFE};

  const auto part1 = static_cast<std::uint32_t>(operation) |
                     (static_cast<std::uint32_t>(type) << 3U) |
                     ((static_cast<std::uint32_t>(layer) & 0x0FU) << 6U) |
                     ((static_cast<std::uint32_t>(color) & 0x0FU) << 10U) |
                     ((static_cast<std::uint32_t>(detail_a) & 0x01FFU) << 14U) |
                     ((static_cast<std::uint32_t>(detail_b) & 0x01FFU) << 23U);
  const auto part2 = (static_cast<std::uint32_t>(width) & 0x03FFU) |
                     ((static_cast<std::uint32_t>(x) & 0x07FFU) << 10U) |
                     ((static_cast<std::uint32_t>(y) & 0x07FFU) << 21U);

  write_u32_le(payload, written, part1);
  write_u32_le(payload, written, part2);
  write_u32_le(payload, written, detail_cde);
}

std::uint32_t pack_detail_cde(std::uint16_t radius, std::uint16_t end_x,
                              std::uint16_t end_y) noexcept {
  return (static_cast<std::uint32_t>(radius) & 0x03FFU) |
         ((static_cast<std::uint32_t>(end_x) & 0x07FFU) << 10U) |
         ((static_cast<std::uint32_t>(end_y) & 0x07FFU) << 21U);
}

void append_text_content(std::span<std::byte> payload, std::size_t &written,
                         std::string_view content) {
  const auto size = std::min(content.size(), text_content_size);
  for (std::size_t index = 0; index < text_content_size; ++index) {
    payload[written++] = index < size ? static_cast<std::byte>(content[index])
                                      : std::byte{' '};
  }
}

} // namespace

void Shape::write_no_operation_description(std::span<std::byte> payload,
                                           std::size_t &written) {
  append_graphic_description(payload, written, 0, Operation::none,
                             ShapeType::line, 0, Color::white, 0, 0, 0, 0, 0,
                             0);
}

Shape::Shape(InteractionUi &interaction_ui, Color color, std::uint8_t layer,
             std::uint16_t width)
    : interaction_ui_(interaction_ui), color_(color), layer_(layer),
      width_(width) {
  interaction_ui_.register_shape(*this);
}

Shape::~Shape() { interaction_ui_.remove(*this); }

void Shape::set_visible(bool visible) {
  if (visible_ == visible) {
    return;
  }
  visible_ = visible;
  if (!visible_) {
    if (existence_confidence_ == 0) {
      interaction_ui_.disable_id_reuse(*this);
      interaction_ui_.unmark_modified(*this);
      sync_confidence_ = max_update_times;
      return;
    }
    interaction_ui_.enable_id_reuse(*this);
  } else {
    interaction_ui_.disable_id_reuse(*this);
  }
  sync_confidence_ = 0;
  interaction_ui_.mark_modified(*this);
}

void Shape::set_color(Color color) {
  if (color_ == color) {
    return;
  }
  color_ = color;
  set_modified();
}

void Shape::set_layer(std::uint8_t layer) {
  layer = std::min<std::uint8_t>(layer, 9);
  if (layer_ == layer) {
    return;
  }
  layer_ = layer;
  set_modified();
}

void Shape::set_priority(std::uint8_t priority) {
  if (priority_ == priority) {
    return;
  }
  priority_ = priority;
  if (queued_) {
    interaction_ui_.run_queue_insert(*this);
  }
}

void Shape::set_width(std::uint16_t width) {
  width = std::min<std::uint16_t>(width, 1023);
  if (width_ == width) {
    return;
  }
  width_ = width;
  set_modified();
}

void Shape::set_xy(std::uint16_t x, std::uint16_t y) {
  x = std::min<std::uint16_t>(x, 2047);
  y = std::min<std::uint16_t>(y, 2047);
  if (x_ == x && y_ == y) {
    return;
  }
  x_ = x;
  y_ = y;
  set_modified();
}

void Shape::set_modified() {
  if (!visible_) {
    return;
  }
  interaction_ui_.disable_id_reuse(*this);
  sync_confidence_ = 0;
  interaction_ui_.mark_modified(*this);
}

bool Shape::write_update(std::span<std::byte> payload, std::size_t &written,
                         Operation &operation) {
  if (!visible_ && existence_confidence_ == 0) {
    return false;
  }
  if (visible_ && !ensure_id()) {
    sync_confidence_ = max_update_times;
    visible_ = false;
    return false;
  }

  auto predicted_sync = sync_confidence_;
  if (existence_confidence_ == 0) {
    sync_confidence_ = max_update_times;
    predicted_sync = max_update_times;
  }

  if (visible_ &&
      (existence_confidence_ <= predicted_sync ||
       (last_time_modified_ && existence_confidence_ < max_update_times))) {
    operation = Operation::add;
    write_description(payload, written, operation);
  } else {
    operation = Operation::modify;
    if (visible_) {
      write_description(payload, written, operation);
    } else {
      write_invisible_description(payload, written, operation);
    }
  }
  return true;
}

Operation Shape::predict_update() const {
  if (!visible_ && existence_confidence_ == 0) {
    return Operation::none;
  }

  auto predicted_existence = existence_confidence_;
  auto predicted_sync = sync_confidence_;
  if (id_ == 0 &&
      !interaction_ui_.predict_assign_id(*this, predicted_existence)) {
    return Operation::none;
  }

  if (predicted_existence == 0) {
    predicted_sync = max_update_times;
  }

  if (visible_ &&
      (predicted_existence <= predicted_sync ||
       (last_time_modified_ && predicted_existence < max_update_times))) {
    return Operation::add;
  }
  return Operation::modify;
}

bool Shape::ensure_id() {
  if (id_ != 0) {
    return true;
  }
  return interaction_ui_.assign_or_reuse_id(*this);
}

void Shape::mark_sent(Operation operation) {
  if (operation == Operation::add) {
    last_time_modified_ = false;
    if (existence_confidence_ < max_update_times) {
      ++existence_confidence_;
    }
    if (existence_confidence_ < max_update_times ||
        sync_confidence_ < max_update_times) {
      interaction_ui_.mark_modified(*this);
    }
    return;
  }

  if (operation == Operation::modify) {
    last_time_modified_ = true;
    if (sync_confidence_ < max_update_times) {
      ++sync_confidence_;
    }
    if (sync_confidence_ < max_update_times) {
      interaction_ui_.mark_modified(*this);
    }
  }
}

void Shape::forget_remote_state() {
  id_ = 0;
  existence_confidence_ = 0;
  sync_confidence_ = 0;
  last_time_modified_ = false;
  reusable_id_ = false;
  if (visible_) {
    interaction_ui_.mark_modified(*this);
  } else {
    interaction_ui_.unmark_modified(*this);
  }
}

void Shape::revoke_id() {
  interaction_ui_.disable_id_reuse(*this);
  id_ = 0;
  existence_confidence_ = 0;
  sync_confidence_ = visible_ ? 0 : max_update_times;
  last_time_modified_ = false;
  if (visible_) {
    interaction_ui_.mark_modified(*this);
  } else {
    interaction_ui_.unmark_modified(*this);
  }
}

void Shape::on_selected_for_update() noexcept {
  const auto shift =
      std::max<std::uint64_t>(1, vruntime_period - queued_weight_);
  vruntime_ += shift;
}

std::uint16_t Shape::scheduling_weight() const noexcept {
  const auto confidence =
      std::min<std::uint8_t>(std::min(existence_confidence_, sync_confidence_),
                             3);
  const auto priority = std::min<std::uint8_t>(priority_, 255);
  const auto penalty = std::min<std::uint32_t>(
      (256U - priority) << (4U * confidence),
      static_cast<std::uint32_t>(vruntime_period - 1U));
  return static_cast<std::uint16_t>((vruntime_period - 1U) - penalty);
}

void Shape::write_invisible_description(std::span<std::byte> payload,
                                        std::size_t &written,
                                        Operation operation) const {
  append_graphic_description(payload, written, id(), operation, ShapeType::line,
                             layer(), Color::white, 0, 0, 0, 0, 0, 0);
}

Line::Line(InteractionUi &interaction_ui, Color color, std::uint16_t width,
           std::uint16_t start_x, std::uint16_t start_y, std::uint16_t end_x,
           std::uint16_t end_y, bool visible)
    : Shape(interaction_ui, color, 0, width) {
  set_xy(start_x, start_y);
  set_end_xy(end_x, end_y);
  set_visible(visible);
}

void Line::set_end_xy(std::uint16_t x, std::uint16_t y) {
  x = std::min<std::uint16_t>(x, 2047);
  y = std::min<std::uint16_t>(y, 2047);
  if (end_x_ == x && end_y_ == y) {
    return;
  }
  end_x_ = x;
  end_y_ = y;
  set_modified();
}

void Line::write_description(std::span<std::byte> payload,
                             std::size_t &written,
                             Operation operation) const {
  append_graphic_description(payload, written, id(), operation, ShapeType::line,
                             layer(), color(), 0, 0, width(), x(), y(),
                             pack_detail_cde(0, end_x_, end_y_));
}

Rectangle::Rectangle(InteractionUi &interaction_ui, Color color, std::uint16_t width,
                     std::uint16_t start_x, std::uint16_t start_y,
                     std::uint16_t end_x, std::uint16_t end_y, bool visible)
    : Shape(interaction_ui, color, 0, width) {
  set_xy(start_x, start_y);
  set_end_xy(end_x, end_y);
  set_visible(visible);
}

void Rectangle::set_end_xy(std::uint16_t x, std::uint16_t y) {
  x = std::min<std::uint16_t>(x, 2047);
  y = std::min<std::uint16_t>(y, 2047);
  if (end_x_ == x && end_y_ == y) {
    return;
  }
  end_x_ = x;
  end_y_ = y;
  set_modified();
}

void Rectangle::write_description(std::span<std::byte> payload,
                                  std::size_t &written,
                                  Operation operation) const {
  append_graphic_description(
      payload, written, id(), operation, ShapeType::rectangle, layer(), color(),
      0, 0, width(), x(), y(), pack_detail_cde(0, end_x_, end_y_));
}

Circle::Circle(InteractionUi &interaction_ui, Color color, std::uint16_t width,
               std::uint16_t x, std::uint16_t y, std::uint16_t radius,
               bool visible)
    : Shape(interaction_ui, color, 0, width) {
  set_xy(x, y);
  set_radius(radius);
  set_visible(visible);
}

void Circle::set_radius(std::uint16_t radius) {
  radius = std::min<std::uint16_t>(radius, 1023);
  if (radius_ == radius) {
    return;
  }
  radius_ = radius;
  set_modified();
}

void Circle::write_description(std::span<std::byte> payload,
                               std::size_t &written,
                               Operation operation) const {
  append_graphic_description(payload, written, id(), operation,
                             ShapeType::circle, layer(), color(), 0, 0,
                             width(), x(), y(), pack_detail_cde(radius_, 0, 0));
}

Integer::Integer(InteractionUi &interaction_ui, Color color, std::uint16_t font_size,
                 std::uint16_t width, std::uint16_t x, std::uint16_t y,
                 std::int32_t value, bool visible)
    : Shape(interaction_ui, color, 0, width), font_size_(font_size), value_(value) {
  set_xy(x, y);
  set_visible(visible);
}

void Integer::set_value(std::int32_t value) {
  if (value_ == value) {
    return;
  }
  value_ = value;
  set_modified();
}

void Integer::set_font_size(std::uint16_t font_size) {
  font_size = std::min<std::uint16_t>(font_size, 511);
  if (font_size_ == font_size) {
    return;
  }
  font_size_ = font_size;
  set_modified();
}

void Integer::set_center_x(std::uint16_t x) {
  auto value = value_;
  auto digits = value <= 0 ? 1 : 0;
  while (value != 0) {
    value /= 10;
    ++digits;
  }
  const auto centered = static_cast<int>(x) -
                        static_cast<int>(font_size_ * digits / 2) +
                        static_cast<int>(font_size_ / 5);
  set_xy(static_cast<std::uint16_t>(std::clamp(centered, 0, 2047)), y());
}

void Integer::write_description(std::span<std::byte> payload,
                                std::size_t &written,
                                Operation operation) const {
  append_graphic_description(
      payload, written, id(), operation, ShapeType::integer, layer(), color(),
      font_size_, 0, width(), x(), y(), std::bit_cast<std::uint32_t>(value_));
}

Text::Text(InteractionUi &interaction_ui, Color color, std::uint16_t font_size,
           std::uint16_t width, std::uint16_t x, std::uint16_t y,
           std::string content, bool visible)
    : Shape(interaction_ui, color, 0, width), font_size_(font_size) {
  set_xy(x, y);
  set_content(content);
  set_visible(visible);
}

void Text::set_content(std::string_view content) {
  if (content.size() > text_content_size) {
    content = content.substr(0, text_content_size);
  }
  if (content_ == content) {
    return;
  }
  content_.assign(content);
  set_modified();
}

void Text::set_font_size(std::uint16_t font_size) {
  font_size = std::min<std::uint16_t>(font_size, 511);
  if (font_size_ == font_size) {
    return;
  }
  font_size_ = font_size;
  set_modified();
}

void Text::write_description(std::span<std::byte> payload,
                             std::size_t &written,
                             Operation operation) const {
  append_graphic_description(
      payload, written, id(), operation, ShapeType::text, layer(), color(),
      font_size_, static_cast<std::uint16_t>(content_.size()), width(), x(),
      y(), 0);
  append_text_content(payload, written, content_);
}

void Text::write_invisible_description(std::span<std::byte> payload,
                                       std::size_t &written,
                                       Operation operation) const {
  append_graphic_description(payload, written, id(), operation, ShapeType::text,
                             layer(), Color::white, 0, 0, 0, 0, 0, 0);
  append_text_content(payload, written, {});
}


} // namespace rmgo_core::referee::ui
