#include "cadly/scene/DefaultView.h"

#include "cadly/scene/Camera.h"

namespace cadly::scene {

DefaultView platform_default_view() {
#if defined(__APPLE__)
  // Quick Look renders STL/OBJ/USD through SceneKit (+Y up, which Cadly
  // shares). SceneKit's *bare* default camera is a flat on-axis view, but the
  // Quick Look extension does not use it -- so this was measured rather than
  // read: render an axis-marker STL through QLThumbnailGenerator and solve the
  // projection from the marker centroids. 45.5 deg of azimuth off +Z toward
  // +X, 27.3 deg of elevation. Reproduces SceneKit's basis to within 0.01 on
  // all three vectors.
  return {{0.634f, 0.458f, 0.624f}, {0.0f, 1.0f, 0.0f}};
#elif defined(_WIN32)
  // PowerToys' "Stereolithography" File Explorer add-on -- the current
  // first-party path, now that 3D Viewer is no longer preinstalled. Its
  // StlThumbnailProvider builds a HelixToolkit camera:
  //   Position = new Point3D(1, 2, 1), UpDirection = new Vector3D(0, 0, 1)
  // Note this views from +Y, which is the *back* under CAD's -Y-is-front
  // convention. Matching the platform means matching that too.
  return {{1.0f, 2.0f, 1.0f}, {0.0f, 0.0f, 1.0f}};
#else
  // No Linux desktop ships an STL thumbnailer; stl-thumb is the one most
  // commonly packaged. From its source:
  //   const CAM_POSITION: Point3<f32> = Point3 { x: 2.0, y: -4.0, z: 2.0 };
  //   look_at_rh(CAM_POSITION, Point3::origin(), Vector3::unit_z())
  // The only one of the three that shows the conventional CAD
  // front-right-above three-quarter view.
  return {{2.0f, -4.0f, 2.0f}, {0.0f, 0.0f, 1.0f}};
#endif
}

void apply_default_view(Camera& camera, const vec3& min, const vec3& max) {
  const DefaultView view = platform_default_view();
  camera.look_from(vec3(view.eye[0], view.eye[1], view.eye[2]),
                   vec3(view.up[0],  view.up[1],  view.up[2]));
  camera.frame_bounds(min, max);
}

} // namespace cadly::scene
