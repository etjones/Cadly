#include "cadly/scene/Camera.h"

#include <algorithm>
#include <cmath>

namespace cadly::scene {

namespace {
constexpr vec3 kWorldUp{0.0f, 1.0f, 0.0f};
}

vec3 Camera::position() const {
  return target - forward() * distance;
}

vec3 Camera::forward() const {
  // OpenGL convention: camera looks along its local -Z.
  return orientation * vec3(0.0f, 0.0f, -1.0f);
}

vec3 Camera::right() const {
  return orientation * vec3(1.0f, 0.0f, 0.0f);
}

vec3 Camera::up() const {
  return orientation * vec3(0.0f, 1.0f, 0.0f);
}

mat4 Camera::view() const {
  // view = inverse(translate(position) * rotate(orientation))
  //      = rotate(inverse(orientation)) * translate(-position)
  return glm::mat4_cast(glm::inverse(orientation)) *
         glm::translate(mat4(1.0f), -position());
}

mat4 Camera::projection() const {
  const float a = std::max(aspect, 0.0001f);
  if (projection_mode == Projection::Perspective) {
    return glm::perspective(fov_y, a, near_z, far_z);
  }
  // Match the apparent height at the target plane so toggling preserves
  // on-screen scale, and so distance-based zoom keeps behaving the same.
  const float half_h = distance * std::tan(0.5f * fov_y);
  const float half_w = half_h * a;
  return glm::ortho(-half_w, half_w, -half_h, half_h, near_z, far_z);
}

void Camera::rotate_around(const vec3& pivot, const quat& delta) {
  // Rotate the target offset from the pivot, then update the orientation by
  // the same delta. Because `position()` is derived as
  //     position = target - (orientation * -Z) * distance
  // applying `delta` to both `target - pivot` and `orientation` yields
  //     new_position - pivot = delta * (position - pivot)
  // i.e. the eye orbits the pivot too, and `distance` is preserved.
  target      = pivot + delta * (target - pivot);
  orientation = glm::normalize(delta * orientation);
}

void Camera::orbit(float yaw_delta, float pitch_delta, const vec3& pivot) {
  // Screen-space tumble ("free orbit") — the scheme mainstream mechanical
  // CAD uses for its rotate drag (SolidWorks, NX, Creo, Fusion's free
  // orbit). The drag rotates the model about a single axis that lies in the
  // screen plane, perpendicular to the drag direction:
  //
  //   yaw_delta   spins about the camera's own up axis    (screen vertical)
  //   pitch_delta spins about the camera's own right axis (screen horizontal)
  //
  // and a diagonal drag combines them into one rotation about
  // `up*yaw + right*pitch` — exactly like rolling a ball under the cursor.
  // Because the axes are re-read from the current orientation on every
  // event, the response is uniform over the whole sphere: dragging right
  // always moves the model's near side right, at every elevation. There is
  // no pole to stall on and no upside-down regime where the response
  // reverses — the failure modes of the turntable schemes this replaces
  // (yaw about *world* up, first clamped at ±89°, then mirrored when
  // upside down; both read as bugs during free inspection of a part).
  //
  // The price is that roll can accumulate: a circular drag path slowly
  // rotates the model about the view axis, so the horizon may end up
  // tilted. That is inherent to any screen-space scheme (the commercial
  // packages above drift the same way) and is cheap to undo — the standard
  // views (keys 1-7) and Fit (F) restore an upright orientation.
  //
  // Per mouse-move the deltas are a few milliradians, so folding both spins
  // into one angleAxis about the scaled-axis sum is exact to O(θ²); larger
  // programmatic steps stay well-behaved because the axis still lies in the
  // screen plane. Right/up are orthonormal, so the combined angle is just
  // the Euclidean norm of the two deltas.
  const float angle = std::sqrt(yaw_delta * yaw_delta +
                                pitch_delta * pitch_delta);
  if (angle <= 0.0f) return;
  const vec3 axis = (up() * yaw_delta + right() * pitch_delta) / angle;
  rotate_around(pivot, glm::angleAxis(angle, axis));
}

void Camera::orbit_turntable(float yaw_delta, float pitch_delta,
                             const vec3& pivot) {
  // Pitch first, about the current right axis, with the elevation clamped
  // short of the poles (at exactly ±90° the yaw axis and the view axis
  // coincide and yaw degenerates into roll). Elevation is recovered from
  // the forward vector each call, so no separate yaw/pitch state can drift
  // out of sync with the quaternion.
  constexpr float kElevationLimit = glm::radians(89.5f);
  const float sin_elevation =
    glm::clamp(glm::dot(forward(), vec3(0.0f, 1.0f, 0.0f)), -1.0f, 1.0f);
  const float elevation = std::asin(sin_elevation);
  const float applied_pitch =
    glm::clamp(elevation + pitch_delta, -kElevationLimit, kElevationLimit) -
    elevation;
  if (applied_pitch != 0.0f) {
    rotate_around(pivot, glm::angleAxis(applied_pitch, right()));
  }
  if (yaw_delta != 0.0f) {
    rotate_around(pivot, glm::angleAxis(yaw_delta, kWorldUp));
  }
}

void Camera::set_orientation_yaw_pitch(float yaw, float pitch) {
  // The +π on the yaw rotates the camera-local -Z (which is "look direction")
  // to face +Z at yaw=0, matching the historical Euler convention so that
  // existing presets and frame_bounds() output land in the same view.
  const quat q_yaw   = glm::angleAxis(yaw + glm::pi<float>(), kWorldUp);
  const quat q_pitch = glm::angleAxis(pitch, vec3(1.0f, 0.0f, 0.0f));
  orientation = glm::normalize(q_yaw * q_pitch);
}

void Camera::look_from(const vec3& eye_direction, const vec3& up) {
  const vec3 forward = -glm::normalize(eye_direction);
  vec3 right = glm::cross(forward, up);
  if (glm::length(right) < 1e-5f) {
    const vec3 fallback = std::abs(forward.y) < 0.9f ? vec3(0.0f, 1.0f, 0.0f)
                                                     : vec3(0.0f, 0.0f, 1.0f);
    right = glm::cross(forward, fallback);
  }
  right = glm::normalize(right);
  const vec3 true_up = glm::cross(right, forward);
  // Camera-to-world rotation. The identity camera looks down -Z, so the
  // basis columns are (right, up, backward).
  orientation = glm::normalize(glm::quat_cast(mat3(right, true_up, -forward)));
}

void Camera::frame_bounds(const vec3& min, const vec3& max, float fit_factor) {
  const vec3 center = 0.5f * (min + max);
  const vec3 extent = max - min;
  const float radius = 0.5f * glm::length(extent);
  target = center;

  // Distance such that the bounding sphere fits the smaller of the two FOVs.
  const float fov_x = 2.0f * std::atan(std::tan(0.5f * fov_y) * aspect);
  const float fov = std::min(fov_y, fov_x);
  distance = (radius / std::sin(0.5f * fov)) * fit_factor;

  // Reasonable near/far around the bounding sphere.
  near_z = std::max(distance - radius * 4.0f, radius * 0.005f);
  far_z  = distance + radius * 8.0f;
  if (far_z <= near_z) far_z = near_z + 1.0f;
}

} // namespace cadly::scene
