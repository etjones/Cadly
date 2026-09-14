#pragma once

#include "cadly/cad/ICadImporter.h"
#include "cadly/scene/Mesh.h"

#include <TDF_Label.hxx>
#include <TopoDS_Shape.hxx>
#include <Quantity_Color.hxx>

#include <memory>
#include <optional>
#include <vector>

class TDocStd_Document;

namespace cadly::cad::occt {

// Per-call accumulators populated as we walk the topology. Exposed so the
// importer can fold them into ImportSummary without owning conversion state.
struct ConversionStats {
  std::size_t face_count    {0};
  std::size_t triangle_count{0};
  std::size_t vertex_count  {0};
  double model_extent{0.0};
  double resolved_linear_deflection{0.0};
  TessellationMode tessellation_mode{TessellationMode::VisualRelative};
  std::vector<ImportTiming> timings;
  std::vector<Diagnostic> diagnostics;
};

// Convert a single OCCT shape to a renderer-ready Mesh. Caller decides what
// "shape" means: a face, a solid, or the full assembly aggregated. The mesh
// is already triangulated (BRepMesh_IncrementalMesh runs inside).
//
// Colour is expressed exclusively through per-submesh materials assigned by
// the caller; vertex colours are written as white (a neutral multiplier for
// the shader's u_base_color * v_vertex_color composition). Baking a colour
// into the vertices AND assigning it to the submesh material used to square
// the colour on screen.
//
// Returns an empty pointer if the shape has no usable triangulation, or if
// `progress` reports cancellation before or during the safety-net mesher
// (which is bridged into OCCT so a budgeted import can abort mid-shape).
std::shared_ptr<scene::Mesh>
shape_to_mesh(const TopoDS_Shape& shape,
              const ImportOptions& options,
              ConversionStats& stats,
              IProgressSink* progress = nullptr);

// Walk an XDE document and emit one Mesh per shape entry, plus a flat scene
// hierarchy that mirrors the assembly structure. The returned scene already
// has world transforms updated. `unit_to_meters` reflects the document's
// declared length unit.
//
// If `doc` is null, the function returns an empty scene.
std::shared_ptr<scene::Scene>
document_to_scene(const opencascade::handle<TDocStd_Document>& doc,
                  const TopoDS_Shape& fallback_shape,
                  const ImportOptions& options,
                  float unit_to_meters,
                  ConversionStats& stats,
                  IProgressSink& progress);

} // namespace cadly::cad::occt
