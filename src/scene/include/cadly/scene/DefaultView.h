#pragma once

// The default camera for a freshly opened model, chosen per host platform.
//
// Every desktop previews STL (the one 3D format its file manager already
// handles) from its own canonical angle, and a STEP file that opens facing
// the opposite way from its STL twin reads as a bug -- in the Quick Look
// preview and in the app alike, which is why this lives in scene rather than
// in either shell. One policy, consumed by both.

#include "cadly/scene/Math.h"

namespace cadly::scene {

struct Camera;

// Written the way the reference previewer's own source writes it: a camera
// position looking at the model centre, plus the up axis that previewer
// uses. Six numbers, and they are the only thing to change if a platform
// moves its default -- no trigonometry to redo. Magnitudes are ignored;
// frame_bounds() derives the distance, so {2,-4,2} and {1,-2,1} are the
// same view.
struct DefaultView {
  float eye[3];  // camera position, looking toward the model centre
  float up[3];   // the axis that previewer treats as "up"
};

// This build's policy, selected per platform at compile time. See the .cpp
// for each entry and the source it was taken from.
DefaultView platform_default_view();

// Orient `camera` per platform_default_view() and frame `min..max`. Set
// `camera.aspect` first. Callers that only want to re-fit an orientation the
// user has since changed should call `camera.frame_bounds()` instead.
void apply_default_view(Camera& camera, const vec3& min, const vec3& max);

} // namespace cadly::scene
