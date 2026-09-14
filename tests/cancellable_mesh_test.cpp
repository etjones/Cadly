// The per-shape safety-net mesher in shape_to_mesh() must honour the
// progress sink exactly like the whole-document batch pass does: refuse to
// start when already cancelled, and abort mid-mesh when cancellation
// arrives while OCCT is triangulating. Before the fix it ran
// BRepMesh_IncrementalMesh with no progress range at all, and on the no-XDE
// fallback path that call is the entire tessellation -- a budgeted preview
// import was seen pinning five cores for twenty minutes with its sink
// reporting cancelled the whole time.
#include "TestChecks.h"

#include "OcctShapeToMesh.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>

#include <string>

namespace {

using cadly::cad::ImportOptions;
using cadly::cad::IProgressSink;
using cadly::cad::occt::ConversionStats;
using cadly::cad::occt::shape_to_mesh;

class AlreadyCancelled final : public IProgressSink {
public:
  void update(float, const std::string&) override {}
  bool cancelled() const override { return true; }
};

// Answers "no" exactly once -- to the entry check -- then "yes" to every
// query OCCT makes through the progress bridge while meshing. Only a mesher
// that actually carries the bridge ever asks a second time.
class CancelWhileMeshing final : public IProgressSink {
public:
  void update(float, const std::string&) override {}
  bool cancelled() const override { return ++asks_ > 1; }
  int asks() const { return asks_; }
private:
  mutable int asks_{0};
};

class Never final : public IProgressSink {
public:
  void update(float, const std::string&) override {}
  bool cancelled() const override { return false; }
};

int triangulated_faces(const TopoDS_Shape& shape, int* total = nullptr) {
  int done = 0, all = 0;
  for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
    ++all;
    TopLoc_Location loc;
    if (!BRep_Tool::Triangulation(TopoDS::Face(ex.Current()), loc).IsNull()) {
      ++done;
    }
  }
  if (total) *total = all;
  return done;
}

// A fresh, untriangulated shape each time (meshing mutates the BRep): a
// grid of boxes, so there are hundreds of faces for the mesher to be
// interrupted between.
TopoDS_Shape fresh_shape() {
  TopoDS_Compound all;
  BRep_Builder builder;
  builder.MakeCompound(all);
  for (int i = 0; i < 12; ++i)
    for (int j = 0; j < 12; ++j)
      builder.Add(all, BRepPrimAPI_MakeBox(gp_Pnt(i * 3.0, j * 3.0, 0.0),
                                           2.0, 2.0, 2.0).Shape());
  return all;
}

} // namespace

int main() {
  ImportOptions opts;
  opts.linear_deflection = 0.01;   // fine enough that meshing is real work
  // Sequential, so OCCT consults the progress range between faces and an
  // abort is observable as a partial triangulation.
  opts.parallel_meshing = false;

  {
    ConversionStats stats;
    AlreadyCancelled sink;
    auto shape = fresh_shape();
    auto mesh = shape_to_mesh(shape, opts, stats, &sink);
    CHECK(!mesh);                              // refused up front
    CHECK(triangulated_faces(shape) == 0);     // and never touched OCCT
  }
  {
    ConversionStats stats;
    CancelWhileMeshing sink;
    auto shape = fresh_shape();
    auto mesh = shape_to_mesh(shape, opts, stats, &sink);
    CHECK(!mesh);                              // aborted once cancelled
    CHECK(sink.asks() > 1);                    // i.e. OCCT was asking
    int total = 0;
    const int done = triangulated_faces(shape, &total);
    CHECK(total > 100);
    CHECK(done < total);                       // stopped part-way, not
                                               // "meshed all, then discarded"
  }
  {
    ConversionStats stats;
    Never sink;
    auto shape = fresh_shape();
    auto mesh = shape_to_mesh(shape, opts, stats, &sink);
    if (CHECK(mesh != nullptr)) CHECK(!mesh->indices.empty());
  }
  {
    ConversionStats stats;
    auto shape = fresh_shape();
    auto mesh = shape_to_mesh(shape, opts, stats);   // no sink: unchanged
    if (CHECK(mesh != nullptr)) CHECK(!mesh->indices.empty());
  }
  return cadly::tests::report();
}
