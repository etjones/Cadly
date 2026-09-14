#include "OcctShapeToMesh.h"

#include "OcctProgressBridge.h"

#include "cadly/cad/TessellationPolicy.h"
#include "cadly/platform/Log.h"
#include "cadly/scene/Aabb.h"
#include "cadly/scene/Math.h"
#include "cadly/scene/Node.h"
#include "cadly/scene/Scene.h"

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <IMeshTools_Parameters.hxx>
#include <Message_ProgressRange.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <Standard_Version.hxx>
#include <StdPrs_ToolTriangulatedShape.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace cadly::cad::occt {

namespace {

std::uint32_t pack_color(const Quantity_Color& c, float alpha = 1.0f) {
  auto clamp01 = [](double v) {
    return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
  };
  const auto r = static_cast<std::uint32_t>(clamp01(c.Red())   * 255.0 + 0.5);
  const auto g = static_cast<std::uint32_t>(clamp01(c.Green()) * 255.0 + 0.5);
  const auto b = static_cast<std::uint32_t>(clamp01(c.Blue())  * 255.0 + 0.5);
  const auto a = static_cast<std::uint32_t>(clamp01(alpha)     * 255.0 + 0.5);
  return r | (g << 8) | (b << 16) | (a << 24);
}

// Tessellate a shape in place. OCCT mutates the BRep with per-face Poly_*.
// The parameter constructor runs the mesh algorithm itself ("Automatically
// calls method Perform" per the OCCT header) — do not call Perform() again;
// the second call re-walks the whole shape only to discover there is nothing
// left to do. The explicit IMeshTools_Parameters form (rather than the
// 4-scalar convenience constructor, which fills the same four fields) is the
// only overload that also accepts a progress range, which is how the batch
// document pass stays cancellable.
void tessellate(const TopoDS_Shape& shape,
                const ImportOptions& opts,
                const Message_ProgressRange& range = Message_ProgressRange()) {
  IMeshTools_Parameters params;
  params.Deflection = opts.linear_deflection;
  params.Angle      = opts.angular_deflection;
  params.Relative   = opts.relative_deflection;
  params.InParallel = opts.parallel_meshing;
  BRepMesh_IncrementalMesh mesher(shape, params, range);
}

// True when every face of the shape already carries a triangulation. Used to
// skip the per-shape safety mesher after the whole-document batch pass: the
// batch ran with the same resolved deflections, so existing triangulations
// are adequate by construction and re-running the mesher would only pay a
// full topology walk to conclude there is nothing to do (~4.5 s across the
// 60k faces of a 231 MB assembly). Faces the batch failed to mesh return
// false and the safety net still runs for their shape.
bool fully_triangulated(const TopoDS_Shape& shape) {
  TopExp_Explorer ex(shape, TopAbs_FACE);
  if (!ex.More()) return false;   // no faces -> nothing meshable anyway
  for (; ex.More(); ex.Next()) {
    TopLoc_Location loc;
    if (BRep_Tool::Triangulation(TopoDS::Face(ex.Current()), loc).IsNull()) {
      return false;
    }
  }
  return true;
}

void add_timing(ConversionStats& stats,
                const char* stage,
                std::chrono::steady_clock::duration duration) {
  if (!duration.count()) return;
  const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
    duration);
  for (auto& timing : stats.timings) {
    if (timing.stage == stage) {
      timing.duration += elapsed;
      return;
    }
  }
  stats.timings.push_back({stage, elapsed});
}

// Convert an OCCT bounding box into a finite scene Aabb. Returns false for
// boxes that must not contribute to model sizing: void boxes, boxes flagged
// "open" on any side, and boxes whose corners sit beyond any plausible model
// coordinate. OCCT models unbounded geometry as an open box whose Get()
// corners are ±1e100 sentinels — seen in the wild on conical faces whose
// STEP trim failed to translate, leaving the surface's natural infinite
// parameter range in charge. Cast to float those sentinels become ±inf and
// poison every extent/deflection computation downstream.
bool box_to_finite_aabb(const Bnd_Box& box, scene::Aabb& out) {
  if (box.IsVoid()) return false;
  if (box.IsOpenXmin() || box.IsOpenXmax() ||
      box.IsOpenYmin() || box.IsOpenYmax() ||
      box.IsOpenZmin() || box.IsOpenZmax()) {
    return false;
  }
  Standard_Real xmin = 0.0, ymin = 0.0, zmin = 0.0;
  Standard_Real xmax = 0.0, ymax = 0.0, zmax = 0.0;
  box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
  // 1e30 is far beyond any sane model in mm yet comfortably inside float
  // range, so the casts below can never overflow to inf.
  constexpr Standard_Real kMaxCoord = 1e30;
  for (const Standard_Real v : {xmin, ymin, zmin, xmax, ymax, zmax}) {
    if (!std::isfinite(v) || std::abs(v) > kMaxCoord) return false;
  }
  out = scene::Aabb::empty();
  out.expand({static_cast<float>(xmin),
              static_cast<float>(ymin),
              static_cast<float>(zmin)});
  out.expand({static_cast<float>(xmax),
              static_cast<float>(ymax),
              static_cast<float>(zmax)});
  return true;
}

// Whole-shape bounds for tessellation sizing. When the aggregate box is
// unusable (some face carries unbounded geometry) fall back to accumulating
// per-face boxes, skipping the unbounded offenders, so sizing still sees the
// real extent of the healthy geometry. `unbounded_faces` is incremented by
// the number of faces dropped so callers can surface a diagnostic. A shape
// with no usable box at all yields an empty Aabb, which the tessellation
// policy treats as "fall back to absolute deflection".
scene::Aabb shape_bounds(const TopoDS_Shape& shape,
                         std::size_t* unbounded_faces = nullptr) {
  if (shape.IsNull()) return scene::Aabb::empty();
  Bnd_Box box;
  BRepBndLib::Add(shape, box);
  scene::Aabb out = scene::Aabb::empty();
  if (box_to_finite_aabb(box, out)) return out;

  std::size_t skipped = 0;
  for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
    Bnd_Box face_box;
    BRepBndLib::Add(ex.Current(), face_box);
    scene::Aabb face_aabb;
    if (box_to_finite_aabb(face_box, face_aabb)) {
      out.expand(face_aabb);
    } else {
      ++skipped;
    }
  }
  if (unbounded_faces) *unbounded_faces += skipped;
  return out;
}

void note_unbounded_faces(ConversionStats& stats, std::size_t count) {
  if (!count) return;
  stats.diagnostics.push_back({DiagnosticSeverity::Warning,
    std::to_string(count) + " face(s) have unbounded geometry; they were "
    "excluded from tessellation sizing."});
}

void apply_resolved_tessellation(ConversionStats& stats,
                                 const ResolvedTessellation& resolved) {
  stats.model_extent = resolved.model_extent;
  stats.resolved_linear_deflection =
    resolved.resolved_linear_deflection;
  stats.tessellation_mode = resolved.options.tessellation_mode;
}

// Area of a face's existing triangulation. The tiny-face filter only needs
// a coarse magnitude to reject degenerate slivers, but it used to get one
// from BRepGProp::SurfaceProperties — exact surface integration that cost
// ~26 s (22 % of the whole import) across the 60k faces of a 231 MB
// assembly. The triangulation the mesher just produced is an adequate
// estimate and costs milliseconds in total. Triangle areas are invariant
// under the location's rigid part; only a scale factor (rare, but gp_Trsf
// allows one) affects them.
double triangulation_area(const Handle(Poly_Triangulation)& tri,
                          const gp_Trsf& trsf) {
  double twice_area = 0.0;
  const Standard_Integer tri_count = tri->NbTriangles();
  for (Standard_Integer i = 1; i <= tri_count; ++i) {
    Standard_Integer a = 0, b = 0, c = 0;
#if OCC_VERSION_HEX >= 0x070600
    tri->Triangle(i).Get(a, b, c);
    const gp_Pnt pa = tri->Node(a);
    const gp_Pnt pb = tri->Node(b);
    const gp_Pnt pc = tri->Node(c);
#else
    tri->Triangles().Value(i).Get(a, b, c);
    const gp_Pnt pa = tri->Nodes().Value(a);
    const gp_Pnt pb = tri->Nodes().Value(b);
    const gp_Pnt pc = tri->Nodes().Value(c);
#endif
    const gp_Vec ab(pa, pb);
    const gp_Vec ac(pa, pc);
    twice_area += ab.Crossed(ac).Magnitude();
  }
  const double scale = trsf.ScaleFactor();
  return 0.5 * twice_area * scale * scale;
}

// Push one face's triangulation into the running Mesh buffers. Returns the
// number of triangles emitted. `seen_edges` is shared across all faces of a
// shape and tracks which TopoDS_Edges have already had their mesh-coupled
// polyline emitted — an edge bordering two faces would otherwise be drawn
// twice from each face's own copy of the boundary nodes.
std::size_t append_face(scene::Mesh& mesh,
                        const TopoDS_Face& face,
                        const ImportOptions& opts,
                        ConversionStats& stats,
                        std::uint32_t source_face_id,
                        std::unordered_set<const void*>& seen_edges) {
  using clock = std::chrono::steady_clock;
  TopLoc_Location loc;
  Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
  if (tri.IsNull()) return 0;

  if (opts.min_face_area > 0.0) {
    const auto phase_start = clock::now();
    const double area = triangulation_area(tri, loc.Transformation());
    if (opts.profile_timings) {
      add_timing(stats, "face area checks", clock::now() - phase_start);
    }
    if (area < opts.min_face_area) return 0;
  }

  const gp_Trsf& trsf = loc.Transformation();
  const bool reversed = (face.Orientation() == TopAbs_REVERSED);

  // Submesh slot allocated up-front; index_offset filled now, count later.
  scene::Submesh sub;
  sub.material_index   = 0;   // default; importer can later override
  sub.source_face_id   = source_face_id;
  sub.index_offset     = static_cast<std::uint32_t>(mesh.indices.size());
  sub.bounds           = scene::Aabb::empty();

  const std::uint32_t base_vertex = static_cast<std::uint32_t>(mesh.vertices.size());
  // White — a neutral multiplier. Colour comes from the submesh material
  // only; see shape_to_mesh's doc comment.
  const std::uint32_t color_packed = 0xFFFFFFFFu;

  // OCCT often leaves imported triangulations without nodal normals. A
  // triangle-area average is a reasonable fallback for ordinary faces, but
  // it is wrong at periodic seams: the duplicated seam vertices are averaged
  // from only one side and acquire slightly different normals. When a view's
  // silhouette lands on that seam, the GPU contour pass sees no sign change
  // and drops the entire generator line. Use OCCT's surface/UV-aware normal
  // computation first so periodic faces receive the same analytic normal at
  // both copies; retain the triangle average below for triangulations that
  // have no UV data.
  if (!tri->HasNormals() && opts.compute_missing_normals && tri->HasUVNodes()) {
    BRepAdaptor_Surface surface(face, Standard_False);
    const bool periodic_surface =
      surface.IsUClosed() || surface.IsVClosed() ||
      surface.IsUPeriodic() || surface.IsVPeriodic();
    if (periodic_surface) {
      const auto normal_phase_start = clock::now();
      StdPrs_ToolTriangulatedShape::ComputeNormals(face, tri);
      if (opts.profile_timings) {
        add_timing(stats, "surface normal generation",
                   clock::now() - normal_phase_start);
      }
    }
  }

#if OCC_VERSION_HEX >= 0x070600
  const Standard_Integer node_count = tri->NbNodes();
  auto phase_start = clock::now();
  for (Standard_Integer i = 1; i <= node_count; ++i) {
    gp_Pnt p = tri->Node(i);
    if (!loc.IsIdentity()) p.Transform(trsf);
    scene::Vertex v{};
    v.position = scene::vec3(static_cast<float>(p.X()),
                             static_cast<float>(p.Y()),
                             static_cast<float>(p.Z()));
    v.color_rgba8 = color_packed;
    if (tri->HasNormals()) {
      gp_Dir n = tri->Normal(i);
      if (!loc.IsIdentity()) n.Transform(trsf);
      if (reversed) n.Reverse();
      v.normal = scene::vec3(static_cast<float>(n.X()),
                             static_cast<float>(n.Y()),
                             static_cast<float>(n.Z()));
    }
    sub.bounds.expand(v.position);
    mesh.vertices.push_back(v);
  }
  if (opts.profile_timings) {
    add_timing(stats, "face vertex extraction", clock::now() - phase_start);
  }
#else
  const auto& nodes = tri->Nodes();
  const Standard_Integer node_count = nodes.Length();
  auto phase_start = clock::now();
  for (Standard_Integer i = 1; i <= node_count; ++i) {
    gp_Pnt p = nodes(i);
    if (!loc.IsIdentity()) p.Transform(trsf);
    scene::Vertex v{};
    v.position = scene::vec3(static_cast<float>(p.X()),
                             static_cast<float>(p.Y()),
                             static_cast<float>(p.Z()));
    v.color_rgba8 = color_packed;
    sub.bounds.expand(v.position);
    mesh.vertices.push_back(v);
  }
  if (opts.profile_timings) {
    add_timing(stats, "face vertex extraction", clock::now() - phase_start);
  }
#endif

  const Standard_Integer tri_count = tri->NbTriangles();
  std::size_t emitted = 0;
  phase_start = clock::now();
  for (Standard_Integer i = 1; i <= tri_count; ++i) {
    Standard_Integer a = 0, b = 0, c = 0;
#if OCC_VERSION_HEX >= 0x070600
    const Poly_Triangle& t = tri->Triangle(i);
#else
    const Poly_Triangle& t = tri->Triangles().Value(i);
#endif
    t.Get(a, b, c);
    // Convert to 0-based and apply face orientation.
    const std::uint32_t i0 = base_vertex + static_cast<std::uint32_t>(a - 1);
    const std::uint32_t i1 = base_vertex + static_cast<std::uint32_t>(b - 1);
    const std::uint32_t i2 = base_vertex + static_cast<std::uint32_t>(c - 1);
    if (reversed) {
      mesh.indices.push_back(i0);
      mesh.indices.push_back(i2);
      mesh.indices.push_back(i1);
    } else {
      mesh.indices.push_back(i0);
      mesh.indices.push_back(i1);
      mesh.indices.push_back(i2);
    }
    ++emitted;
  }
  if (opts.profile_timings) {
    add_timing(stats, "triangle index extraction", clock::now() - phase_start);
  }

  // If OCCT didn't give us normals, derive flat normals per triangle.
  if (!tri->HasNormals() && opts.compute_missing_normals) {
    phase_start = clock::now();
    for (std::uint32_t i = base_vertex; i < mesh.vertices.size(); ++i) {
      mesh.vertices[i].normal = scene::vec3(0.0f);
    }
    for (std::uint32_t i = sub.index_offset;
         i + 2 < sub.index_offset + emitted * 3;
         i += 3) {
      auto& v0 = mesh.vertices[mesh.indices[i + 0]];
      auto& v1 = mesh.vertices[mesh.indices[i + 1]];
      auto& v2 = mesh.vertices[mesh.indices[i + 2]];
      const scene::vec3 e1 = v1.position - v0.position;
      const scene::vec3 e2 = v2.position - v0.position;
      scene::vec3 n = glm::cross(e1, e2);
      const float len2 = glm::dot(n, n);
      if (len2 > 1e-12f) n /= std::sqrt(len2);
      v0.normal += n; v1.normal += n; v2.normal += n;
    }
    for (std::uint32_t i = base_vertex; i < mesh.vertices.size(); ++i) {
      const float len2 = glm::dot(mesh.vertices[i].normal,
                                  mesh.vertices[i].normal);
      if (len2 > 1e-12f) {
        mesh.vertices[i].normal /= std::sqrt(len2);
      } else {
        mesh.vertices[i].normal = scene::vec3(0.0f, 1.0f, 0.0f);
      }
    }
    if (opts.profile_timings) {
      add_timing(stats, "normal generation", clock::now() - phase_start);
    }
  }

  sub.index_count = static_cast<std::uint32_t>(emitted * 3);
  mesh.submeshes.push_back(sub);
  mesh.bounds.expand(sub.bounds);

  // Mesh-coupled BRep edge polylines. Each TopoDS_Edge of the face carries
  // a Poly_PolygonOnTriangulation: a 1-based array of integers indexing
  // into THIS face's node array. Emit one GL_LINE_STRIP per edge,
  // terminated by the 0xFFFFFFFF primitive-restart sentinel, with indices
  // pointing into the global `mesh.vertices` (offset by `base_vertex`).
  // The resulting edge vertices are literally the same memory as the
  // corresponding face vertices, so the "shaded with edges" overlay needs
  // only glPolygonOffset to keep edges visible: their depth values are
  // identical before offset.
  //
  // Strip + restart instead of GL_LINES pairs: the old layout repeated each
  // interior node in two consecutive segments, and draw_edges' alpha-blended
  // ink rasterised the joint pixel twice, leaving a row of dark "dots" along
  // every curved edge. Strips render each joint once and the dots disappear.
  //
  // Dedup is keyed on TShape*. An edge shared by two faces produces a
  // valid PolygonOnTriangulation under each face's triangulation, but
  // OCCT seeds both with the same boundary sample points so it does not
  // matter which face we use — picking the first one we see is correct.
  phase_start = clock::now();
  for (TopExp_Explorer ex_e(face, TopAbs_EDGE); ex_e.More(); ex_e.Next()) {
    const TopoDS_Edge& edge = TopoDS::Edge(ex_e.Current());
    if (BRep_Tool::Degenerated(edge)) continue;
    // Parametric seam edges (the closing edge of a periodic face: cylinder
    // and cone walls, sphere meridians, torus rings) are an artifact of the
    // surface parametrization, not part geometry — commercial CAD (Creo,
    // SolidWorks, NX) never inks them. The renderer's silhouette pass now
    // gives curved faces their view-dependent contour, so dropping seams
    // costs nothing visually and stops the hidden-line style from painting
    // a stray lengthwise line on every cylinder.
    if (BRep_Tool::IsClosed(edge, face)) continue;
    const void* key = edge.TShape().get();
    if (!seen_edges.insert(key).second) continue;

    Handle(Poly_PolygonOnTriangulation) poly =
      BRep_Tool::PolygonOnTriangulation(edge, tri, loc);
    if (poly.IsNull()) continue;
    const TColStd_Array1OfInteger& nodes = poly->Nodes();
    if (nodes.Length() < 2) continue;
    for (Standard_Integer i = nodes.Lower(); i <= nodes.Upper(); ++i) {
      mesh.edge_strip_indices.push_back(
        base_vertex + static_cast<std::uint32_t>(nodes.Value(i) - 1));
    }
    mesh.edge_strip_indices.push_back(0xFFFFFFFFu);
  }
  if (opts.profile_timings) {
    add_timing(stats, "mesh-coupled edge strips", clock::now() - phase_start);
  }

  stats.triangle_count += emitted;
  stats.vertex_count   += node_count;
  ++stats.face_count;
  return emitted;
}

std::optional<Quantity_Color>
resolve_face_color(const Handle(XCAFDoc_ColorTool)& color_tool,
                   const TopoDS_Shape& face) {
  if (color_tool.IsNull()) return std::nullopt;
  Quantity_Color c;
  if (color_tool->GetColor(face, XCAFDoc_ColorSurf, c)) return c;
  if (color_tool->GetColor(face, XCAFDoc_ColorGen,  c)) return c;
  return std::nullopt;
}

std::optional<Quantity_Color>
resolve_shape_color(const Handle(XCAFDoc_ColorTool)& color_tool,
                    const TDF_Label& label,
                    const TopoDS_Shape& shape) {
  if (color_tool.IsNull()) return std::nullopt;
  Quantity_Color c;
  if (!label.IsNull()) {
    if (color_tool->GetColor(label, XCAFDoc_ColorGen,  c)) return c;
    if (color_tool->GetColor(label, XCAFDoc_ColorSurf, c)) return c;
  }
  if (color_tool->GetColor(shape, XCAFDoc_ColorGen,  c)) return c;
  if (color_tool->GetColor(shape, XCAFDoc_ColorSurf, c)) return c;
  return std::nullopt;
}

// Colour attached to the label itself, ignoring the underlying shape. This
// is how XCAF represents occurrence styling: a colour on a reference label
// applies to that one instance, while prototype/shape colours are shared by
// every occurrence of the part.
std::optional<Quantity_Color>
resolve_label_color(const Handle(XCAFDoc_ColorTool)& color_tool,
                    const TDF_Label& label) {
  if (color_tool.IsNull() || label.IsNull()) return std::nullopt;
  Quantity_Color c;
  if (color_tool->GetColor(label, XCAFDoc_ColorGen,  c)) return c;
  if (color_tool->GetColor(label, XCAFDoc_ColorSurf, c)) return c;
  return std::nullopt;
}

std::string read_label_name(const TDF_Label& label) {
  if (label.IsNull()) return {};
  Handle(TDataStd_Name) name_attr;
  if (label.FindAttribute(TDataStd_Name::GetID(), name_attr)) {
    const TCollection_ExtendedString& es = name_attr->Get();
    std::string out;
    out.reserve(static_cast<std::size_t>(es.Length()));
    for (Standard_Integer i = 1; i <= es.Length(); ++i) {
      const Standard_ExtCharacter ch = es.Value(i);
      out.push_back(ch < 128 ? static_cast<char>(ch) : '?');
    }
    return out;
  }
  return {};
}

// Sample one edge's 3D curve at the given deflection tolerances and append
// the resulting polyline to `lod` as a GL_LINE_STRIP run terminated by the
// 0xFFFFFFFF primitive-restart sentinel. Points are in the shape's local
// frame because BRepAdaptor_Curve folds in edge.Location() internally.
// Returns true if a non-degenerate polyline was emitted.
//
// Why a strip + restart and NOT pair-indices for GL_LINES: the old layout
// pushed every interior polyline vertex into two consecutive segments, so
// the rasterizer produced two fragments at every joint pixel. With the
// alpha-blended ink in draw_edges (depth_mask off, no depth filtering),
// the joint pixel ended up blended twice — visibly darker than the rest of
// the line — and a row of evenly-spaced "dots" appeared along every curved
// edge. Strip + restart makes each joint a single fragment.
bool sample_edge_into_lod(scene::Mesh::EdgeLod& lod,
                          const TopoDS_Edge& edge,
                          double angular_deflection,
                          double linear_deflection) {
  BRepAdaptor_Curve curve(edge);
  std::vector<scene::vec3> pts;
  try {
    GCPnts_TangentialDeflection sampler(
      curve,
      static_cast<Standard_Real>(angular_deflection),
      static_cast<Standard_Real>(linear_deflection));
    const Standard_Integer n = sampler.NbPoints();
    if (n < 2) return false;
    pts.reserve(static_cast<std::size_t>(n));
    for (Standard_Integer i = 1; i <= n; ++i) {
      const gp_Pnt& p = sampler.Value(i);
      pts.emplace_back(static_cast<float>(p.X()),
                       static_cast<float>(p.Y()),
                       static_cast<float>(p.Z()));
    }
  } catch (...) {
    return false;
  }

  const auto base = static_cast<std::uint32_t>(lod.vertices.size());
  lod.vertices.insert(lod.vertices.end(), pts.begin(), pts.end());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    lod.indices.push_back(base + static_cast<std::uint32_t>(i));
  }
  // Restart sentinel separates this polyline from the next so the renderer
  // can issue a single GL_LINE_STRIP draw across the whole LOD buffer
  // without the strips bleeding into each other. The renderer binds 0xFFFFFFFFu
  // via glPrimitiveRestartIndex; keep both sides in sync if it ever changes.
  lod.indices.push_back(0xFFFFFFFFu);
  return true;
}

// Bake a level-of-detail ladder of BRep edge polylines for the shape. Each
// tier is a full re-discretization at a finer chord-to-curve tolerance: tier
// 0 matches the surface mesh deflection (so edges and faces align at default
// zoom), tier 1 is 4x finer, tier 2 is 16x finer. The renderer picks one
// tier per frame from the camera's world-per-pixel scale and binds that
// tier's prebuilt GPU buffers — no runtime curve evaluation, no stutter.
//
// Straight lines collapse to 2 samples regardless of deflection, so the
// extra LODs are essentially free for boxy parts; curves get progressively
// smoother polylines without ever forcing the renderer to recompute them.
//
// Deduplication is by the underlying TShape pointer so an edge shared by
// two adjacent faces is only sampled once per tier. Degenerate edges (e.g.
// the pole-collapse "edge" of a sphere) carry no 3D curve and are skipped.
void extract_brep_edges(scene::Mesh& mesh,
                        const TopoDS_Shape& shape,
                        const ImportOptions& opts,
                        ConversionStats& stats) {
  using clock = std::chrono::steady_clock;
  // Tier deflection multipliers. The factor-of-4 spacing on linear
  // deflection gives a comfortable hysteresis band when the renderer picks
  // a tier from world-per-pixel: a flip requires zooming ~4x further, so
  // tier oscillation during slow zooms is impossible.
  struct Tier { double linear_mul; double angular_mul; };
  static constexpr Tier kTiers[] = {
    {1.0,    1.0   },  // coarse — surface-aligned
    {0.25,   0.5   },  // fine
    {0.0625, 0.25  },  // ultra
  };
  constexpr std::size_t kLodCount = sizeof(kTiers) / sizeof(kTiers[0]);

  mesh.edge_lods.resize(kLodCount);
  for (std::size_t t = 0; t < kLodCount; ++t) {
    mesh.edge_lods[t].linear_deflection =
      static_cast<float>(opts.linear_deflection * kTiers[t].linear_mul);
  }

  std::unordered_set<const void*> seen;
  // Seam edges are per-face closing edges, so detecting them needs the face
  // context this shape-level edge walk doesn't have: collect their TShapes
  // up front. Skipped for the same reason append_face skips them — they are
  // parametrization artifacts no commercial viewer draws, and the
  // silhouette pass carries the curved-face contour in wireframe now.
  std::unordered_set<const void*> seam_edges;
  for (TopExp_Explorer fx(shape, TopAbs_FACE); fx.More(); fx.Next()) {
    const TopoDS_Face& face = TopoDS::Face(fx.Current());
    for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
      const TopoDS_Edge& edge = TopoDS::Edge(ex.Current());
      if (BRep_Tool::IsClosed(edge, face)) {
        seam_edges.insert(edge.TShape().get());
      }
    }
  }
  for (TopExp_Explorer ex(shape, TopAbs_EDGE); ex.More(); ex.Next()) {
    const TopoDS_Edge& edge = TopoDS::Edge(ex.Current());
    if (BRep_Tool::Degenerated(edge)) continue;
    const void* key = edge.TShape().get();
    if (seam_edges.count(key)) continue;
    if (!seen.insert(key).second) continue;

    for (std::size_t t = 0; t < kLodCount; ++t) {
      const auto phase_start = clock::now();
      sample_edge_into_lod(
        mesh.edge_lods[t], edge,
        opts.angular_deflection * kTiers[t].angular_mul,
        opts.linear_deflection  * kTiers[t].linear_mul);
      if (opts.profile_timings) {
        add_timing(stats, "analytical edge LOD sampling",
                   clock::now() - phase_start);
      }
    }
  }

  // Drop trailing empty tiers (e.g. all edges failed to sample at the
  // finest tolerance) so the renderer's selection always lands on a tier
  // with geometry.
  while (!mesh.edge_lods.empty() && mesh.edge_lods.back().indices.empty()) {
    mesh.edge_lods.pop_back();
  }
}

// A shape is safe to backface-cull only when its triangles carry a consistent
// outward winding — that is, when it bounds a volume. STEP imports are
// typically TopoDS_Solids and qualify. IGES imports are usually a quilt of
// independent trimmed faces with no shell/solid topology, so each patch's
// orientation comes straight from its (arbitrary) surface parametrization;
// culling would silently drop every patch that happens to face away from the
// camera. Treat a shape as cull-safe if it contains a solid or a closed shell;
// the importer flags everything else double-sided.
bool is_cull_safe(const TopoDS_Shape& shape) {
  if (TopExp_Explorer(shape, TopAbs_SOLID).More()) return true;
  for (TopExp_Explorer ex(shape, TopAbs_SHELL); ex.More(); ex.Next()) {
    if (BRep_Tool::IsClosed(ex.Current())) return true;
  }
  return false;
}

} // namespace

std::shared_ptr<scene::Mesh>
shape_to_mesh(const TopoDS_Shape& shape,
              const ImportOptions& opts,
              ConversionStats& stats,
              IProgressSink* progress) {
  using clock = std::chrono::steady_clock;
  if (shape.IsNull()) return nullptr;
  if (progress && progress->cancelled()) return nullptr;
  // Safety net for faces the whole-document batch pass missed (or the
  // fallback path, which has no batch pass at all). Skipped entirely when
  // every face is already triangulated — see fully_triangulated().
  //
  // This mesher must be bridged into OCCT exactly like the batch pass is.
  // Without a progress range BRepMesh never asks whether to stop, and on
  // the no-XDE fallback path this call *is* the whole tessellation: a
  // budgeted import (the Quick Look preview gives 20 s) was observed
  // pinning five cores for over twenty minutes here, long after its sink
  // had reported cancelled, because nothing in this function ever looked.
  if (!fully_triangulated(shape)) {
    auto phase_start = clock::now();
    if (progress) {
      Handle(OcctProgressBridge) bridge =
        new OcctProgressBridge(*progress, 0.45f, 0.70f,
                               "Tessellating geometry...");
      tessellate(shape, opts, bridge->Start());
      if (progress->cancelled()) return nullptr;
    } else {
      tessellate(shape, opts);
    }
    if (opts.profile_timings) {
      add_timing(stats, "shape safety tessellation",
                 clock::now() - phase_start);
    }
  }

  auto mesh = std::make_shared<scene::Mesh>();
  std::uint32_t face_id = 0;
  auto phase_start = clock::now();
  std::unordered_set<const void*> seen_strip_edges;
  for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
    const TopoDS_Face& face = TopoDS::Face(ex.Current());
    append_face(*mesh, face, opts, stats, face_id++, seen_strip_edges);
  }
  if (opts.profile_timings) {
    add_timing(stats, "face topology walk", clock::now() - phase_start);
  }
  if (mesh->indices.empty()) return nullptr;
  // Non-solid shapes (typically IGES surface quilts / open shells) have no
  // consistent outward winding; the renderer must draw them double-sided or
  // half of every part gets backface-culled away.
  phase_start = clock::now();
  mesh->double_sided = !is_cull_safe(shape);
  if (opts.profile_timings) {
    add_timing(stats, "cull-safety topology check", clock::now() - phase_start);
  }
  phase_start = clock::now();
  extract_brep_edges(*mesh, shape, opts, stats);
  if (opts.profile_timings) {
    add_timing(stats, "analytical edge LOD total", clock::now() - phase_start);
  }
  return mesh;
}

std::shared_ptr<scene::Scene>
document_to_scene(const opencascade::handle<TDocStd_Document>& doc,
                  const TopoDS_Shape& fallback_shape,
                  const ImportOptions& opts,
                  float unit_to_meters,
                  ConversionStats& stats,
                  IProgressSink& progress) {
  auto scn = std::make_shared<scene::Scene>();
  scn->unit_to_meters = unit_to_meters;

  // No-XDE fallback path — treat the whole shape as a single node.
  if (doc.IsNull()) {
    if (fallback_shape.IsNull()) return scn;
    std::size_t unbounded_faces = 0;
    const auto resolved = resolve_tessellation_policy(
      opts, shape_bounds(fallback_shape, &unbounded_faces));
    apply_resolved_tessellation(stats, resolved);
    note_unbounded_faces(stats, unbounded_faces);
    auto phase_start = std::chrono::steady_clock::now();
    auto mesh = shape_to_mesh(fallback_shape, resolved.options, stats,
                              &progress);
    if (opts.profile_timings) {
      add_timing(stats, "fallback shape conversion",
                 std::chrono::steady_clock::now() - phase_start);
    }
    if (!mesh) return scn;
    mesh->name = "root";
    const auto mesh_idx = scn->add_mesh(mesh);
    scene::Node root;
    root.name = "root";
    root.mesh_index = mesh_idx;
    root.source_label = "/root";
    scn->add_node(std::move(root));
    scn->add_material(scene::Material::neutral_clay());
    phase_start = std::chrono::steady_clock::now();
    scn->update_transforms();
    if (opts.profile_timings) {
      add_timing(stats, "scene transform update",
                 std::chrono::steady_clock::now() - phase_start);
    }
    return scn;
  }

  Handle(XCAFDoc_ShapeTool) shape_tool =
    XCAFDoc_DocumentTool::ShapeTool(doc->Main());
  Handle(XCAFDoc_ColorTool) color_tool =
    XCAFDoc_DocumentTool::ColorTool(doc->Main());

  // Allocate the default material slot so face submeshes can index it.
  scn->add_material(scene::Material::neutral_clay());

  TDF_LabelSequence labels;
  shape_tool->GetFreeShapes(labels);
  if (labels.IsEmpty()) {
    stats.diagnostics.push_back({DiagnosticSeverity::Warning,
                                 "XDE document had no free shapes."});
    return scn;
  }

  // Triangulate the whole document in a single pass. BRepMesh_IncrementalMesh
  // parallelizes across faces, so meshing every free shape at once keeps all
  // cores busy. Meshing per-part during the walk instead (as shape_to_mesh
  // still does below) serializes many small jobs, each with too few faces to
  // fill the thread pool — the slow path on large assemblies. Located instances
  // of a part share the same face TShapes, and a triangulation is stored in the
  // face's local frame, so one mesher covers every occurrence; afterwards
  // shape_to_mesh sees every face already triangulated and skips its safety
  // mesher (which only runs for shapes this batch somehow missed).
  ImportOptions resolved_opts = opts;
  {
    const auto phase_start = std::chrono::steady_clock::now();
    TopoDS_Compound all;
    BRep_Builder builder;
    builder.MakeCompound(all);
    for (Standard_Integer i = 1; i <= labels.Length(); ++i) {
      TopoDS_Shape s;
      if (shape_tool->GetShape(labels.Value(i), s) && !s.IsNull()) {
        builder.Add(all, s);
      }
    }
    std::size_t unbounded_faces = 0;
    const auto resolved =
      resolve_tessellation_policy(opts, shape_bounds(all, &unbounded_faces));
    apply_resolved_tessellation(stats, resolved);
    note_unbounded_faces(stats, unbounded_faces);
    resolved_opts = resolved.options;
    progress.update(0.45f, "Tessellating geometry...");
    // Bridge the sink into the mesher so the (potentially tens of seconds)
    // batch pass honours cancellation and advances the progress bar.
    Handle(OcctProgressBridge) mesh_bridge =
      new OcctProgressBridge(progress, 0.45f, 0.70f,
                             "Tessellating geometry...");
    tessellate(all, resolved_opts, mesh_bridge->Start());
    if (opts.profile_timings) {
      add_timing(stats, "batch document tessellation",
                 std::chrono::steady_clock::now() - phase_start);
    }
  }
  if (progress.cancelled()) return scn;

  // Each "free shape" is treated as a separate root. We then expand the
  // assembly under it. Mesh sharing is keyed on the OCCT shape's TShape*
  // pointer so instanced parts upload once.
  std::unordered_map<const void*, std::uint32_t> mesh_cache;
  std::unordered_map<std::uint32_t, std::uint32_t> material_cache;

  auto material_for_color = [&](const std::optional<Quantity_Color>& c) -> std::uint32_t {
    if (!c) return 0;
    const std::uint32_t key = pack_color(*c);
    auto it = material_cache.find(key);
    if (it != material_cache.end()) return it->second;
    scene::Material m;
    m.name = "color_" + std::to_string(material_cache.size());
    m.base_color = scene::vec4(
      static_cast<float>(c->Red()),
      static_cast<float>(c->Green()),
      static_cast<float>(c->Blue()),
      1.0f);
    // Fully dielectric + matte. A faintly metallic / mid-roughness default
    // turns residual facet error into visible specular banding from the IBL
    // probe — see Material::plastic() for the same reasoning.
    m.metallic   = 0.0f;
    m.roughness  = 0.75f;
    const std::uint32_t idx = scn->add_material(std::move(m));
    material_cache.emplace(key, idx);
    return idx;
  };

  // Recursive descent. Each call creates one Scene::Node for `label` and
  // attaches/recurses based on the label's role in the XDE document:
  //
  //   - Assembly       : node has children, one per component
  //   - Reference      : node carries the instance location; geometry comes
  //                      from the prototype's shape (recursed into for nested
  //                      assemblies so nothing is lost)
  //   - SimpleShape    : node owns a Mesh directly
  //
  // The location returned by GetLocation(label) is the *instance* transform
  // for component labels and identity for prototype labels — exactly what we
  // need to place each occurrence in world space without manually
  // accumulating transforms during traversal.
  auto build_mesh_for = [&](const TopoDS_Shape& shape,
                            const TDF_Label& proto_label) -> std::uint32_t {
    const void* key = shape.TShape().get();
    auto cache_it = mesh_cache.find(key);
    if (cache_it != mesh_cache.end()) return cache_it->second;

    // Prototype-level colour only. The mesh — including its per-face
    // materials — is shared by every occurrence of the part, so occurrence
    // styling must not leak into it; the walk applies instance colours as
    // Node::material_override instead. (The first caller's instance colour
    // used to be baked into the cached mesh, painting every later
    // occurrence with it.)
    const auto default_color =
      resolve_shape_color(color_tool, proto_label, shape);
    const auto phase_start = std::chrono::steady_clock::now();
    auto mesh = shape_to_mesh(shape, resolved_opts, stats, &progress);
    if (opts.profile_timings) {
      add_timing(stats, "shape conversion total",
                 std::chrono::steady_clock::now() - phase_start);
    }
    if (!mesh) return scene::Scene::kInvalid;
    mesh->name = read_label_name(proto_label);
    if (mesh->name.empty()) mesh->name = "mesh";

    // Pair faces with submeshes through the recorded source_face_id:
    // append_face emits no submesh for faces without triangulation or below
    // the tiny-face threshold, so the submesh list is a subsequence of the
    // explored faces. The old positional pairing shifted every material
    // after the first skipped face onto the wrong submesh.
    std::size_t sub_i = 0;
    std::uint32_t face_id = 0;
    for (TopExp_Explorer ex(shape, TopAbs_FACE);
         ex.More() && sub_i < mesh->submeshes.size();
         ex.Next(), ++face_id) {
      if (mesh->submeshes[sub_i].source_face_id != face_id) continue;
      auto fc = resolve_face_color(color_tool, ex.Current());
      mesh->submeshes[sub_i].material_index = fc
        ? material_for_color(fc)
        : material_for_color(default_color);
      ++sub_i;
    }
    const auto idx = scn->add_mesh(mesh);
    mesh_cache.emplace(key, idx);
    return idx;
  };

  std::function<std::uint32_t(const TDF_Label&, std::uint32_t, const std::string&)> walk;
  walk = [&](const TDF_Label& label,
             std::uint32_t parent,
             const std::string& path) -> std::uint32_t {
    if (progress.cancelled()) return scene::Scene::kInvalid;
    if (!shape_tool->IsShape(label)) return scene::Scene::kInvalid;

    scene::Node node;
    node.parent = parent;
    node.name = read_label_name(label);
    if (node.name.empty()) node.name = "node";
    node.source_label = path + "/" + node.name;

    // Instance/local transform — pulled from the label, which carries the
    // per-instance placement for components and identity for prototypes.
    TopLoc_Location loc = shape_tool->GetLocation(label);
    if (!loc.IsIdentity()) {
      gp_Trsf trsf = loc.Transformation();
      scene::mat4 m(1.0f);
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c)
          m[c][r] = static_cast<float>(trsf.Value(r + 1, c + 1));
      node.local = scene::Transform::from_matrix(m);
    }

    const std::uint32_t node_idx = scn->add_node(std::move(node));
    if (parent != scene::Scene::kInvalid) {
      scn->nodes[parent].children.push_back(node_idx);
    }

    auto recurse_children = [&](const TDF_Label& assembly_label) {
      TDF_LabelSequence components;
      shape_tool->GetComponents(assembly_label, components);
      for (Standard_Integer i = 1; i <= components.Length(); ++i) {
        walk(components.Value(i), node_idx,
             scn->nodes[node_idx].source_label);
      }
    };

    if (shape_tool->IsAssembly(label)) {
      recurse_children(label);
    } else if (shape_tool->IsReference(label)) {
      // Component: resolve the prototype and continue.
      TDF_Label proto;
      if (!shape_tool->GetReferredShape(label, proto)) return node_idx;
      if (shape_tool->IsAssembly(proto)) {
        // The prototype is itself an assembly — its components become our
        // node's children, sharing our instance transform.
        recurse_children(proto);
      } else if (shape_tool->IsSimpleShape(proto)) {
        TopoDS_Shape shape;
        shape_tool->GetShape(proto, shape);
        const auto mesh_idx = build_mesh_for(shape, proto);
        if (mesh_idx != scene::Scene::kInvalid) {
          scn->nodes[node_idx].mesh_index = mesh_idx;
          // Occurrence colour styled on the reference label applies to this
          // instance only; the shared prototype mesh must not absorb it.
          if (auto inst_color = resolve_label_color(color_tool, label)) {
            scn->nodes[node_idx].material_override =
              material_for_color(inst_color);
          }
        }
      }
    } else if (shape_tool->IsSimpleShape(label)) {
      TopoDS_Shape shape;
      shape_tool->GetShape(label, shape);
      const auto mesh_idx = build_mesh_for(shape, label);
      if (mesh_idx != scene::Scene::kInvalid) {
        scn->nodes[node_idx].mesh_index = mesh_idx;
      }
    }
    return node_idx;
  };

  const auto walk_start = std::chrono::steady_clock::now();
  for (Standard_Integer i = 1; i <= labels.Length(); ++i) {
    if (progress.cancelled()) break;
    progress.update(static_cast<float>(i) / labels.Length(),
                    "Walking assembly...");
    walk(labels.Value(i), scene::Scene::kInvalid, "");
  }
  if (opts.profile_timings) {
    add_timing(stats, "assembly walk total",
               std::chrono::steady_clock::now() - walk_start);
  }

  const auto transform_start = std::chrono::steady_clock::now();
  scn->update_transforms();
  if (opts.profile_timings) {
    add_timing(stats, "scene transform update",
               std::chrono::steady_clock::now() - transform_start);
  }
  return scn;
}

} // namespace cadly::cad::occt
