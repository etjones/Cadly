#pragma once

#include "cadly/input/NavigationCommand.h"
#include "cadly/scene/Camera.h"

#include <QObject>
#include <QPoint>

#include <memory>

namespace cadly::ui {

// Where a rotate-drag should pivot. Resolvers (below) build one of these at
// the moment the user presses the rotate button; the controller then uses it
// for every mouse-move event in the same drag.
struct RotationPivot {
  scene::vec3 world_position{0.0f};
};

// A ray through a viewport pixel, in world space. Perspective gives a true eye
// ray; orthographic gives a parallel ray whose origin sits on the focal plane
// through the camera target. Either way `direction` is unit length and points
// away from the viewer, so `origin + direction * t` walks into the scene.
struct ScreenRay {
  scene::vec3 origin   {0.0f};
  scene::vec3 direction{0.0f, 0.0f, -1.0f};
};

// Strategy interface for choosing the rotation pivot. The default
// implementation (`TargetPivotResolver`) returns the camera's current target,
// which reproduces the historical "orbit around target" behaviour. Future
// resolvers can supply different pivots without touching the controller:
//
//   - PickedPointPivotResolver — ray-cast against the scene at `screen_pos`
//     and pivot around the hit point, so the model rotates "under the
//     cursor".
//   - SelectionPivotResolver   — pivot around the bounding-box centre of the
//     current selection.
//   - WorldOriginPivotResolver — pivot around (0, 0, 0) for layout work.
//
// All a new resolver needs to do is implement `resolve()` and hand an
// instance to `CameraController::set_rotation_pivot_resolver()`.
class RotationPivotResolver {
public:
  virtual ~RotationPivotResolver() = default;
  virtual RotationPivot resolve(const scene::Camera& camera,
                                QPoint                screen_pos) const = 0;
};

// Default pivot strategy: rotate around the camera's current target. Kept as
// a concrete class (not an inline lambda) so callers can re-create it after
// swapping in a custom resolver and back.
class TargetPivotResolver final : public RotationPivotResolver {
public:
  RotationPivot resolve(const scene::Camera& camera, QPoint) const override;
};

// Orbit/pan/zoom executor. Owns no widget or input preferences; the host feeds
// resolved navigation commands in and consults `camera()` after each step.
// Rotation uses quaternions around a pivot resolved at the start of each drag — see
// `set_rotation_pivot_resolver()` for how to swap in custom pivot strategies.
class CameraController : public QObject {
  Q_OBJECT
public:
  explicit CameraController(QObject* parent = nullptr);

  scene::Camera& camera() { return camera_; }
  const scene::Camera& camera() const { return camera_; }

  void set_viewport(int w, int h);
  void frame_bounds(const scene::vec3& min, const scene::vec3& max);
  // Orient to the platform's default view for a freshly opened model
  // (scene::platform_default_view) and frame the bounds. Used once per
  // document; later fits keep whatever orientation the user has orbited to.
  void apply_default_view(const scene::vec3& min, const scene::vec3& max);
  // Restore a document's camera while refreshing the controller's cached
  // bounds for zoom and clip-plane calculations.
  void restore_camera(const scene::Camera& camera,
                      const scene::vec3& min, const scene::vec3& max);

  // Input policy (buttons, modifiers, sensitivity, drag ownership) stays in
  // Cadly::Input. This boundary only applies camera-space motion, resolves the
  // pivot at BeginOrbit, and enforces the existing projection/clip policies.
  void apply_navigation(const input::NavigationCommand& command);

  // Re-orient the camera to a fixed yaw/pitch (in degrees) without moving the
  // target or changing distance. Drives the View > Standard Views actions
  // (Front, Top, Right, Iso, …). Yaw/pitch follow the same convention as
  // scene::Camera::set_orientation_yaw_pitch: yaw=0,pitch=0 places the camera
  // on -Z looking toward +Z (the "front" face), pitch=-90 looks straight down.
  void set_view(float yaw_deg, float pitch_deg);

  // Switch the projection mode. Goes through the controller (rather than
  // writing Camera::projection_mode directly) because the two modes have
  // different zoom/clip policies: entering Perspective re-clamps the distance
  // so an eye parked inside the model by deep ortho zoom (harmless there)
  // pops back outside the bounding sphere before the near plane could slice
  // the model open.
  void set_projection(scene::Projection mode);

  // Replace the rotation-pivot strategy. Passing `nullptr` restores the
  // default `TargetPivotResolver`. Safe to call mid-session; the next
  // BeginOrbit command will use the new resolver.
  void set_rotation_pivot_resolver(std::unique_ptr<RotationPivotResolver> r);

  // True while an orbit drag is in progress.
  bool is_rotating() const { return rotating_; }

  // Screen <-> world, for viewport manipulators (the section-plane handle
  // today; picking when it lands). Both work in LOGICAL pixels — the space
  // QMouseEvent::pos() uses and the space set_viewport() is fed — not the
  // device pixels the renderer's glViewport uses. A caller that needs to match
  // a renderer-side "constant pixel size" must scale by the device pixel ratio
  // itself; see ViewportWidget.
  //
  // Both use the same NDC + focal-plane construction as cursor-anchored zoom,
  // which is what makes them behave identically in orthographic and perspective
  // mode: Camera::projection() derives the ortho half-height from
  // `distance * tan(fov_y/2)`, so one formula covers both.
  ScreenRay screen_ray(QPoint widget_pos) const;

  // Project a world point to logical viewport pixels (origin top-left, Qt
  // convention). `out_behind` reports a point behind a perspective eye, whose
  // projection is meaningless — callers hit-testing a manipulator must reject
  // those rather than trust the coordinates.
  scene::vec2 project_to_screen(const scene::vec3& world,
                                bool* out_behind = nullptr) const;

  // World units per logical pixel at the focal plane. The scale manipulators use
  // to hold a constant on-screen size.
  float world_per_logical_pixel() const;

  // The world-space pivot captured at the start of the current orbit drag.
  // Only meaningful while `is_rotating()` is true; for non-rotation states
  // the value is the most recent pivot (kept around so the renderer can fade
  // the indicator out if desired).
  const scene::vec3& rotation_pivot() const { return rotation_pivot_; }

signals:
  void changed();

  // Emitted when the rotation pivot becomes visible (mouse-down on orbit) or
  // hidden (mouse-up). Carries the world-space pivot so listeners can drive
  // an overlay or marker without having to know about the controller's
  // internal state.
  void rotation_pivot_visibility_changed(scene::vec3 pivot, bool visible);

private:
  // Keep the point on the focal plane under `cursor_pos` fixed on screen.
  void zoom_at(QPoint cursor_pos, float factor);

  // Zoom-distance policy, split by projection mode. Orthographic: zoom is
  // pure magnification (the eye position is optically meaningless), so only
  // an absolute epsilon floor applies — detail zoom is unlimited and
  // update_clip_planes() guarantees the model is never sliced. Perspective:
  // the eye must stay outside the scene's bounding sphere (small margin) so
  // the positive near plane can never cut the model open — zooming in stops
  // at the model instead of passing through it.
  float clamp_distance(float requested) const;

  // The smallest Camera::distance that keeps the eye outside the scene's
  // bounding sphere along the current view ray (0 when there is no scene or
  // the ray misses the sphere). Backs both the perspective zoom clamp and
  // the ortho→perspective transition.
  float min_outside_distance() const;

  // Recompute Camera::near_z / far_z from the current camera position and
  // the cached scene bounds. Orthographic fits the slab around the whole
  // model (near may go negative) so no zoom level can clip it; perspective
  // adapts positive near/far to the eye-to-model distance. Cheap; safe to
  // call on every input event.
  void  update_clip_planes();

  scene::Camera camera_;
  bool          rotating_{false};
  int           viewport_w_{1};
  int           viewport_h_{1};

  std::unique_ptr<RotationPivotResolver> pivot_resolver_;
  scene::vec3                            rotation_pivot_{0.0f};

  // Cached scene bounding sphere — populated by frame_bounds(). The
  // controller uses this to (a) clamp zoom so the camera can't pass through
  // the model, and (b) keep near/far adapted to the current view so close-up
  // zoom doesn't clip the model's front face.
  scene::vec3 scene_center_{0.0f};
  float       scene_radius_{0.0f};
};

} // namespace cadly::ui
