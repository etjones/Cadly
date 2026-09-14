#include "cadly/ui/CameraController.h"

#include "cadly/scene/DefaultView.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace cadly::ui {

namespace {
// Safety margin on the scene's bounding sphere for the perspective
// stay-outside rule: the eye is kept at ≥ radius × this, so the model's
// extreme points (which touch the sphere) can never reach the near plane.
constexpr float kOutsideMargin = 1.05f;
}

RotationPivot TargetPivotResolver::resolve(const scene::Camera& camera,
                                           QPoint) const {
  return RotationPivot{camera.target};
}

CameraController::CameraController(QObject* parent)
  : QObject(parent),
    pivot_resolver_(std::make_unique<TargetPivotResolver>()) {}

void CameraController::set_viewport(int w, int h) {
  viewport_w_ = std::max(1, w);
  viewport_h_ = std::max(1, h);
  camera_.aspect = static_cast<float>(viewport_w_) /
                   static_cast<float>(viewport_h_);
  emit changed();
}

void CameraController::frame_bounds(const scene::vec3& min,
                                    const scene::vec3& max) {
  // Cache the scene's bounding sphere so subsequent zoom/orbit/pan can
  // (a) keep the camera outside the model and (b) keep near/far adapted
  // so the model never gets clipped, no matter how close the user zooms.
  scene_center_ = 0.5f * (min + max);
  scene_radius_ = std::max(0.5f * glm::length(max - min), 1e-4f);
  camera_.aspect = static_cast<float>(viewport_w_) /
                   static_cast<float>(viewport_h_);
  camera_.frame_bounds(min, max);
  update_clip_planes();
  emit changed();
}

void CameraController::apply_default_view(const scene::vec3& min,
                                          const scene::vec3& max) {
  scene_center_ = 0.5f * (min + max);
  scene_radius_ = std::max(0.5f * glm::length(max - min), 1e-4f);
  camera_.aspect = static_cast<float>(viewport_w_) /
                   static_cast<float>(viewport_h_);
  scene::apply_default_view(camera_, min, max);
  update_clip_planes();
  emit changed();
}

void CameraController::restore_camera(const scene::Camera& camera,
                                      const scene::vec3& min,
                                      const scene::vec3& max) {
  scene_center_ = 0.5f * (min + max);
  scene_radius_ = std::max(0.5f * glm::length(max - min), 1e-4f);
  camera_ = camera;
  camera_.aspect = static_cast<float>(viewport_w_) /
                   static_cast<float>(viewport_h_);
  update_clip_planes();
  emit changed();
}

float CameraController::min_outside_distance() const {
  // Smallest `Camera::distance` for which the eye sits outside the scene's
  // bounding sphere (inflated by kOutsideMargin). The eye moves along the
  // ray  position(d) = target − forward·d  as the user zooms, and `target`
  // is generally NOT the sphere centre (cursor-anchored zoom and panning
  // move it), so this is a ray/sphere intersection, not a plain radius
  // comparison. With o = target − centre and f = forward, solving
  // |o − f·d| = r for d gives  d = o·f ± sqrt((o·f)² − |o|² + r²);  the eye
  // is inside the sphere exactly between the two roots, so the larger root
  // is the zoom-in limit. A negative discriminant means the eye ray misses
  // the sphere entirely — the user is zooming past the model, which can
  // never put the eye inside it, so no limit applies.
  if (scene_radius_ <= 0.0f) return 0.0f;
  const scene::vec3 o = camera_.target - scene_center_;
  const scene::vec3 f = camera_.forward();
  const float r    = scene_radius_ * kOutsideMargin;
  const float of   = glm::dot(o, f);
  const float disc = of * of - glm::dot(o, o) + r * r;
  if (disc <= 0.0f) return 0.0f;
  return of + std::sqrt(disc);
}

float CameraController::clamp_distance(float requested) const {
  constexpr float kAbsoluteMin = 1e-4f;
  if (camera_.projection_mode == scene::Projection::Orthographic) {
    // Under parallel projection the eye position has no optical meaning —
    // zoom is pure magnification (the ortho extents derive from `distance`),
    // and update_clip_planes() fits the depth slab around the whole model no
    // matter where the eye sits. So ortho zoom is unlimited: the user can
    // magnify a single feature indefinitely and the model is never cut open.
    return std::max(requested, kAbsoluteMin);
  }
  // Perspective is different: the near plane must sit at z > 0 in front of
  // the eye, so once the eye enters the model the near plane inevitably
  // slices it ("inside the model" cutaway). Keep the eye outside the scene's
  // bounding sphere instead — zooming in runs up to the model and stops at
  // its surface rather than passing through. The sphere is a conservative
  // stand-in for the real surface until depth-based picking exists. If the
  // eye is somehow already inside (camera restored from an older session),
  // don't yank it outward — just refuse to go deeper.
  const float d_min = std::min(min_outside_distance(), camera_.distance);
  return std::max(std::max(requested, d_min), kAbsoluteMin);
}

void CameraController::set_projection(scene::Projection mode) {
  camera_.projection_mode = mode;
  if (mode == scene::Projection::Perspective) {
    // Deep ortho zoom may have parked the eye inside the model — invisible
    // and harmless under parallel projection, but perspective gives the eye
    // position optical meaning again. Hop it back outside the bounding
    // sphere; the apparent zoom level changes, which beats slicing the model
    // open the moment the user presses P.
    camera_.distance = std::max(camera_.distance, min_outside_distance());
  }
  update_clip_planes();
  emit changed();
}

void CameraController::update_clip_planes() {
  // Re-derive near/far from the actual camera and scene geometry on every
  // interaction. A static near/far set once at frame time becomes wrong as
  // soon as the user zooms.
  if (scene_radius_ <= 0.0f) return;
  const float r = scene_radius_;
  if (camera_.projection_mode == scene::Projection::Orthographic) {
    // Fit the clip slab around the entire model, wherever the eye happens to
    // be. `near` may legitimately go negative (model behind the eye plane):
    // GL orthographic projection allows it, and it is exactly what makes
    // deep ortho zoom a magnification instead of a cutaway — the model can
    // never poke out of the slab, so it is never sliced by the near plane.
    const float d_center = glm::dot(scene_center_ - camera_.position(),
                                    camera_.forward());
    camera_.near_z = d_center - r * kOutsideMargin;
    camera_.far_z  = d_center + r * kOutsideMargin;
    return;
  }
  // Perspective: near must stay positive. clamp_distance() keeps the eye
  // outside the bounding sphere, so `near_to_face` (eye to the sphere's
  // front) stays positive too; half of it gives the model headroom while
  // the absolute floor (a small fraction of `distance`) keeps depth
  // precision sane at the closest allowed approach.
  const float d_to_center  = glm::length(camera_.position() - scene_center_);
  const float near_to_face = std::max(d_to_center - r, 0.0f);
  camera_.near_z = std::max(near_to_face * 0.5f, camera_.distance * 0.001f);
  camera_.far_z  = (d_to_center + r) * 2.0f + 1.0f;
  if (camera_.far_z <= camera_.near_z) camera_.far_z = camera_.near_z + 1.0f;
}

void CameraController::set_rotation_pivot_resolver(
    std::unique_ptr<RotationPivotResolver> r) {
  pivot_resolver_ = r ? std::move(r)
                      : std::make_unique<TargetPivotResolver>();
}

void CameraController::apply_navigation(const input::NavigationCommand& command) {
  using Type = input::CommandType;
  switch (command.type) {
    case Type::None:
      return;
    case Type::BeginOrbit:
      if (rotating_) return;
      rotation_pivot_ = pivot_resolver_->resolve(
        camera_, QPoint(command.position.x, command.position.y)).world_position;
      rotating_ = true;
      emit rotation_pivot_visibility_changed(rotation_pivot_, true);
      return;
    case Type::EndOrbit:
      if (!rotating_) return;
      rotating_ = false;
      emit rotation_pivot_visibility_changed(rotation_pivot_, false);
      return;
    case Type::OrbitFree:
    case Type::OrbitTurntable:
      if (!rotating_ || !std::isfinite(command.x) || !std::isfinite(command.y)) return;
      if (command.type == Type::OrbitTurntable) {
        camera_.orbit_turntable(command.x, command.y, rotation_pivot_);
      } else {
        camera_.orbit(command.x, command.y, rotation_pivot_);
      }
      break;
    case Type::Pan:
      if (!std::isfinite(command.x) || !std::isfinite(command.y)) return;
      camera_.target += camera_.right() * (command.x * camera_.distance);
      camera_.target += camera_.up() * (command.y * camera_.distance);
      break;
    case Type::Zoom:
    case Type::ZoomAtCursor:
      if (!std::isfinite(command.factor) || command.factor < 0.0f) return;
      if (command.type == Type::ZoomAtCursor) {
        zoom_at(QPoint(command.position.x, command.position.y), command.factor);
        return;
      }
      camera_.distance = clamp_distance(camera_.distance * command.factor);
      break;
  }
  update_clip_planes();
  emit changed();
}

void CameraController::zoom_at(QPoint cursor_pos, float factor) {
  const float old_distance = camera_.distance;
  const float new_distance = clamp_distance(old_distance * factor);
  const float ratio        = (old_distance > 0.0f) ? (new_distance / old_distance) : 1.0f;

  // Cursor in NDC (Qt y-down → GL y-up).
  const float ndc_x = (2.0f * static_cast<float>(cursor_pos.x()) /
                       static_cast<float>(viewport_w_)) - 1.0f;
  const float ndc_y = 1.0f - (2.0f * static_cast<float>(cursor_pos.y()) /
                              static_cast<float>(viewport_h_));

  // Focal-plane extents — same formula Camera::projection() uses for the ortho
  // path, which keeps the anchor consistent across both projection modes. The
  // anchor sits on the plane through `target` perpendicular to `forward`; for
  // perspective this plane is exactly where view-space depth equals `distance`,
  // so when we scale (target − cursor_world) by `ratio` the cursor's
  // homogeneous divide cancels (depth and offset both scale by ratio) and the
  // world point stays under the cursor. For ortho the proof is even simpler:
  // half_w/half_h scale linearly with `distance`, so the cursor's NDC is
  // preserved by construction.
  const float half_h = old_distance * std::tan(0.5f * camera_.fov_y);
  const float half_w = half_h * camera_.aspect;
  const scene::vec3 cursor_world =
      camera_.target + camera_.right() * (ndc_x * half_w)
                     + camera_.up()    * (ndc_y * half_h);

  // Shrink/expand the (target − cursor_world) offset by `ratio` so the world
  // point under the cursor projects to the same pixel after the distance
  // change. When ratio == 1 (distance clamped) target is unchanged.
  camera_.target   = cursor_world + (camera_.target - cursor_world) * ratio;
  camera_.distance = new_distance;
  update_clip_planes();
  emit changed();
}

float CameraController::world_per_logical_pixel() const {
  // Same relation draw_edges/draw_scale_bar use on the renderer side: the focal
  // plane's world height divided by the viewport's pixel height. Valid in both
  // projection modes because Camera::projection() derives the orthographic
  // half-height from `distance * tan(fov_y/2)` too.
  const float screen_height_world =
    2.0f * std::max(camera_.distance, 1e-6f) * std::tan(0.5f * camera_.fov_y);
  return screen_height_world /
         std::max(static_cast<float>(viewport_h_), 1.0f);
}

ScreenRay CameraController::screen_ray(QPoint widget_pos) const {
  // Cursor in NDC (Qt y-down -> GL y-up), then a point on the focal plane
  // through `target` — the same construction zoom_at() anchors its zoom on.
  const float ndc_x = (2.0f * static_cast<float>(widget_pos.x()) /
                       static_cast<float>(viewport_w_)) - 1.0f;
  const float ndc_y = 1.0f - (2.0f * static_cast<float>(widget_pos.y()) /
                              static_cast<float>(viewport_h_));
  const float half_h = camera_.distance * std::tan(0.5f * camera_.fov_y);
  const float half_w = half_h * camera_.aspect;
  const scene::vec3 on_focal_plane =
      camera_.target + camera_.right() * (ndc_x * half_w)
                     + camera_.up()    * (ndc_y * half_h);

  ScreenRay ray;
  if (camera_.projection_mode == scene::Projection::Perspective) {
    ray.origin    = camera_.position();
    ray.direction = glm::normalize(on_focal_plane - ray.origin);
  } else {
    // Parallel projection: every ray shares the view direction, and the pixel
    // selects the origin rather than the direction. Pull the origin back along
    // the view so `t = 0` is safely in front of anything in the scene.
    ray.direction = camera_.forward();
    ray.origin    = on_focal_plane - ray.direction * camera_.distance;
  }
  return ray;
}

scene::vec2 CameraController::project_to_screen(const scene::vec3& world,
                                               bool* out_behind) const {
  const scene::vec4 clip = camera_.view_proj() * scene::vec4(world, 1.0f);
  const bool behind = clip.w <= 1e-6f;
  if (out_behind) *out_behind = behind;
  if (behind) return scene::vec2(0.0f);
  const scene::vec2 ndc(clip.x / clip.w, clip.y / clip.w);
  return scene::vec2(
    (ndc.x * 0.5f + 0.5f) * static_cast<float>(viewport_w_),
    (0.5f - ndc.y * 0.5f) * static_cast<float>(viewport_h_));
}

void CameraController::set_view(float yaw_deg, float pitch_deg) {
  // Standard-view presets reorient only; preserving target+distance keeps the
  // user's current focal point and zoom, which matches what CAD tools do when
  // you tap a view-cube face. Clip planes still need a refresh because the
  // camera position is derived from orientation, so it shifts relative to the
  // scene center even though `distance` is unchanged.
  camera_.set_orientation_yaw_pitch(glm::radians(yaw_deg),
                                    glm::radians(pitch_deg));
  update_clip_planes();
  emit changed();
}

} // namespace cadly::ui
