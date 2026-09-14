#pragma once

#include "cadly/scene/Math.h"

namespace cadly::scene {

// Orbit-style camera tailored to CAD inspection: a target point, a quaternion
// orientation, and a distance. View math is derived on demand so callers can
// freely tweak the state without ordering concerns.
//
// The orientation is stored as a quaternion (rather than yaw+pitch Euler
// angles) so rotations compose without gimbal headaches and so the camera can
// be orbited around an arbitrary world-space pivot — see `rotate_around()`.
// Interactive rotation is a screen-space tumble ("free orbit"): `orbit()`
// spins about the camera's *own* up/right axes, so the drag response is
// uniform over the whole sphere — no pole to stall on, no orientation where
// it reverses. See orbit() for the trade-offs.
enum class Projection {
  Orthographic,
  Perspective,
};

struct Camera {
  vec3 target{0.0f};

  // Camera-to-world rotation. The identity points the camera down -Z (standard
  // OpenGL convention); the default value here matches the previous
  // (yaw=30°, pitch=-22°) Euler defaults so the initial view is unchanged.
  quat orientation{
    glm::angleAxis(glm::pi<float>() + glm::radians(30.0f), vec3(0.0f, 1.0f, 0.0f)) *
    glm::angleAxis(glm::radians(-22.0f), vec3(1.0f, 0.0f, 0.0f))
  };

  float distance{5.0f};

  float fov_y  {glm::radians(45.0f)};
  float near_z {0.05f};
  float far_z  {1500.0f};
  float aspect {1.0f};

  // CAD inspection traditionally defaults to orthographic so that parallel
  // features stay parallel on screen and measurements are not foreshortened.
  // Ortho extents are derived from `distance` and `fov_y` (see projection()),
  // which keeps zoom-via-distance and frame_bounds() working in both modes.
  Projection projection_mode{Projection::Orthographic};

  vec3 position() const;
  vec3 forward()  const;   // unit direction from eye toward target
  vec3 right()    const;
  vec3 up()       const;

  mat4 view()       const;
  mat4 projection() const;
  mat4 view_proj()  const { return projection() * view(); }

  // Rotate the camera around `pivot` by `delta` (a world-space rotation).
  // Both `target` and the implicit eye position rotate around the pivot;
  // `distance` is preserved by construction. When `pivot == target` this is
  // pure orbit-around-target, but the pivot may also live anywhere else,
  // which is what the controller exploits for "rotate around picked point"
  // and similar features.
  void rotate_around(const vec3& pivot, const quat& delta);

  // Turntable orbit: yaw spins about the *world* up axis, pitch about the
  // camera's right axis with the elevation clamped short of the poles. The
  // complement of orbit(): "up" never tilts and the pivot reads as fixed,
  // at the price of the classic turntable limits — you cannot roll past
  // vertical to inspect the underside in one gesture, and near the poles
  // yaw response compresses. Preserves any roll already present (e.g. from
  // an earlier free orbit); the standard views restore upright exactly.
  void orbit_turntable(float yaw_delta, float pitch_delta, const vec3& pivot);

  // Screen-space tumble for orbit-style mouse input: one rotation about the
  // screen-plane axis `up()*yaw_delta + right()*pitch_delta` (perpendicular
  // to the drag direction), applied around `pivot`. The response follows the
  // hand identically in every orientation — no pitch clamp, no reversal.
  // Roll accumulates on curved drag paths (inherent to screen-space orbit);
  // the standard views / set_orientation_yaw_pitch() restore upright.
  // Use this from input controllers.
  void orbit(float yaw_delta, float pitch_delta, const vec3& pivot);

  // Reset the orientation from a yaw/pitch pair. Convenient for view-cube
  // buttons (front, top, isometric, …) and for any caller that thinks in
  // Euler angles.
  void set_orientation_yaw_pitch(float yaw, float pitch);

  // Reset the orientation from a look-at basis: the camera sits along
  // `eye_direction` from the target (magnitude ignored) and treats `up` as
  // up. Unlike set_orientation_yaw_pitch this carries no +Y-up assumption,
  // so a +Z-up convention can be expressed directly. Target and distance are
  // untouched. A view direction parallel to `up` (a plan view) picks an
  // arbitrary perpendicular for right.
  void look_from(const vec3& eye_direction, const vec3& up);

  // Frame the given world-space bounds with a small margin. `fit_factor` of
  // 1.0 places the box exactly inside the view frustum; larger numbers leave
  // padding around it.
  void frame_bounds(const vec3& min, const vec3& max, float fit_factor = 1.4f);
};

} // namespace cadly::scene
