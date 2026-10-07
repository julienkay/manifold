// Copyright 2021 The Manifold Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest/gtest.h"
#include "manifold/manifold.h"
#include "samples.h"
#include "test.h"

using namespace manifold;

// Check if the mesh remains convex after adding new faces
bool isMeshConvex(Manifold hullManifold, double epsilon = 0.0000001) {
  // Get the mesh from the manifold
  MeshGL64 mesh = hullManifold.GetMeshGL64();

  const auto numTri = mesh.NumTri();
  const auto numVert = mesh.NumVert();

  // Iterate over each triangle
  for (size_t t = 0; t < numTri; ++t) {
    // Get the vertices of the triangle
    auto tri = mesh.GetTriVerts(t);
    vec3 v0 = mesh.GetVertPos(tri[0]);
    vec3 v1 = mesh.GetVertPos(tri[1]);
    vec3 v2 = mesh.GetVertPos(tri[2]);

    // Compute the normal of the triangle
    vec3 normal = la::normalize(la::cross(v1 - v0, v2 - v0));

    // Check all other vertices
    for (size_t i = 0; i < numVert; ++i) {
      if (i == tri[0] || i == tri[1] || i == tri[2])
        continue;  // Skip vertices of the current triangle

      // Get the vertex
      vec3 v = mesh.GetVertPos(i);

      // Compute the signed distance from the plane
      double distance = la::dot(normal, v - v0);

      // If any vertex lies on the opposite side of the normal direction
      if (distance > epsilon) {
        std::cout << distance << std::endl;
        // The manifold is not convex
        return false;
      }
    }
  }
  // If we didn't find any vertex on the opposite side for any triangle, it's
  // convex
  return true;
}

TEST(Hull, Tictac) {
  const double tictacRad = 100;
  const double tictacHeight = 500;
  const int tictacSeg = 500;
  const double tictacMid = tictacHeight - 2 * tictacRad;
  const auto sphere = Manifold::Sphere(tictacRad, tictacSeg);
  const std::vector<Manifold> spheres{sphere,
                                      sphere.Translate({0, 0, tictacMid})};
  const auto tictac = Manifold::Hull(spheres);

  if (options.exportModels) WriteTestOBJ("tictac_hull.obj", tictac);

  EXPECT_NEAR(sphere.NumVert() + tictacSeg, tictac.NumVert(), 1);
}

TEST(Hull, Hollow) {
  auto sphere = Manifold::Sphere(100, 360);
  auto hollow = sphere - sphere.Scale({0.8, 0.8, 0.8});
  const double sphere_vol = sphere.Volume();
  EXPECT_FLOAT_EQ(hollow.Hull().Volume(), sphere_vol);
}

TEST(Hull, Cube) {
  std::vector<vec3> cubePts = {
      {0, 0, 0},       {1, 0, 0},   {0, 1, 0},      {0, 0, 1},  // corners
      {1, 1, 0},       {0, 1, 1},   {1, 0, 1},      {1, 1, 1},  // corners
      {0.5, 0.5, 0.5}, {0.5, 0, 0}, {0.5, 0.7, 0.2}  // internal points
  };
  auto cube = Manifold::Hull(cubePts);
  EXPECT_FLOAT_EQ(cube.Volume(), 1);
}

TEST(Hull, Empty) {
  const std::vector<vec3> tooFew{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  EXPECT_TRUE(Manifold::Hull(tooFew).Simplify().IsEmpty());

  const std::vector<vec3> coplanar{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}};
  EXPECT_TRUE(Manifold::Hull(coplanar).Simplify().IsEmpty());
}

TEST(Hull, MengerSponge) {
  Manifold sponge = MengerSponge(4);
  sponge = sponge.Rotate(10, 20, 30);
  Manifold spongeHull = sponge.Hull();
  EXPECT_EQ(spongeHull.NumTri(), 12);
  EXPECT_FLOAT_EQ(spongeHull.SurfaceArea(), 6);
  EXPECT_FLOAT_EQ(spongeHull.Volume(), 1);
}

TEST(Hull, Sphere) {
  Manifold sphere = Manifold::Sphere(1, 1500);
  sphere = sphere.Translate(vec3(0.5));
  Manifold sphereHull = sphere.Hull();
  EXPECT_EQ(sphereHull.NumTri(), sphere.NumTri());
  EXPECT_FLOAT_EQ(sphereHull.Volume(), sphere.Volume());
}

TEST(Hull, FailingTest1) {
  // 39202.stl
  const std::vector<vec3> hullPts = {
      {-24.983196259f, -43.272167206f, 52.710712433f},
      {-25.0f, -12.7726717f, 49.907142639f},
      {-23.016393661f, 39.865562439f, 79.083930969f},
      {-24.983196259f, -40.272167206f, 52.710712433f},
      {-4.5177311897f, -28.633184433f, 50.405872345f},
      {11.176083565f, -22.357545853f, 45.275596619f},
      {-25.0f, 21.885698318f, 49.907142639f},
      {-17.633232117f, -17.341972351f, 89.96282196f},
      {26.922552109f, 10.344738007f, 57.146999359f},
      {-24.949174881f, 1.5f, 54.598075867f},
      {9.2058267593f, -23.47851944f, 55.334011078f},
      {13.26748085f, -19.979951859f, 28.117856979f},
      {-18.286884308f, 31.673814774f, 2.1749999523f},
      {18.419618607f, -18.215343475f, 52.450099945f},
      {-24.983196259f, 43.272167206f, 52.710712433f},
      {-1.6232370138f, -29.794223785f, 48.394889832f},
      {49.865573883f, -0.0f, 55.507141113f},
      {-18.627283096f, -39.544368744f, 55.507141113f},
      {-20.442623138f, -35.407661438f, 8.2749996185f},
      {10.229375839f, -14.717799187f, 10.508025169f}};
  auto hull = Manifold::Hull(hullPts);
  if (options.exportModels) WriteTestOBJ("failing_test1.obj", hull);
  EXPECT_TRUE(isMeshConvex(hull, 1.09375e-05));
}

TEST(Hull, FailingTest2) {
  // 1750623.stl
  const std::vector<vec3> hullPts = {
      {174.17001343f, -12.022000313f, 29.562002182f},
      {174.51400757f, -10.858000755f, -3.3340001106f},
      {187.50801086f, 22.826000214f, 23.486001968f},
      {172.42800903f, 12.018000603f, 28.120000839f},
      {180.98001099f, -26.866001129f, 6.9100003242f},
      {172.42800903f, -12.022000313f, 28.120000839f},
      {174.17001343f, 19.498001099f, 29.562002182f},
      {213.96600342f, 2.9400000572f, -11.100000381f},
      {182.53001404f, -22.49200058f, 23.644001007f},
      {175.89401245f, 19.900001526f, 16.118000031f},
      {211.38601685f, 3.0200002193f, -14.250000954f},
      {183.7440033f, 12.018000603f, 18.090000153f},
      {210.51000977f, 2.5040001869f, -11.100000381f},
      {204.13601685f, 34.724002838f, -11.250000954f},
      {193.23400879f, -24.704000473f, 17.768001556f},
      {171.62800598f, -19.502000809f, 27.320001602f},
      {189.67401123f, 8.486000061f, -5.4080004692f},
      {193.23800659f, 24.704000473f, 17.758001328f},
      {165.36801147f, -6.5600004196f, -14.250000954f},
      {174.17001343f, -19.502000809f, 29.562002182f},
      {190.06401062f, -0.81000006199f, -14.250000954f}};
  auto hull = Manifold::Hull(hullPts);
  if (options.exportModels) WriteTestOBJ("failing_test2.obj", hull);
  EXPECT_TRUE(isMeshConvex(hull, 2.13966e-05));
}

TEST(Hull, DisabledFaceTest) {
  // 101213.stl
  const std::vector<vec3> hullPts = {
      {65.398902893, 58.303115845, 58.765388489},
      {42.147319794, 44.512584686, 75.703102112},
      {89.208251953, 97.092460632, 41.632453918},
      {69.860748291, 69.860748291, 56.492958069},
      {45.375354767, 39.067985535, 64.844772339},
      {26.555616379, 18.671405792, 81.067504883},
      {88.179382324, 81.083595276, 43.981628418},
      {51.823883057, 50.247039795, 70.359062195},
      {58.489616394, 72.681190491, 51.274829865},
      {110, 10, 65},
      {29.590316772, 20.917686462, 73.143547058},
      {101.61526489, 98.461585999, 30.909877777}};
  auto hull = Manifold::Hull(hullPts);
  if (options.exportModels) WriteTestOBJ("disabledFaceTest.obj", hull);
  EXPECT_TRUE(!hull.IsEmpty());
  EXPECT_TRUE(isMeshConvex(hull));
}

TEST(Hull, Degenerate2D) {
  // issue 1491
  // note that we need 5 points to trigger this bug
  Manifold hull = Manifold::Hull({
      {0.0, 0.0, 0.0},
      {0.0, 0.0, 1.0},
      {0.5, 0.0, 0.0},
      {0.5, 0.0, 0.0},
      {0.5, 0.0, 1.0},
  });
  EXPECT_TRUE(hull.IsEmpty());
}

TEST(Hull, Degenerate1D) {
  Manifold hull = Manifold::Hull({
      {0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0},
      {0.5, 0.0, 0.0},
      {0.5, 0.0, 0.0},
      {0.5, 0.0, 0.0},
  });
  EXPECT_TRUE(hull.IsEmpty());
}

TEST(Hull, NotEnoughPoints) {
  Manifold hull = Manifold::Hull({
      {0.0, 0.0, 0.0},
      {0.5, 0.0, 0.0},
  });
  EXPECT_TRUE(hull.IsEmpty());
}

TEST(Hull, EmptyHull) {
  Manifold hull = Manifold::Hull(std::vector<vec3>());
  EXPECT_TRUE(hull.IsEmpty());
}

namespace {

// Check the input cloud as well as the retained vertices: successful status and
// closed topology alone do not exclude a self-overlapping hull.
void ExpectSupportingPlanes(const Manifold& hull,
                            const std::vector<vec3>& points) {
  const auto mesh = hull.GetMeshGL64();
  ASSERT_GT(mesh.NumTri(), 0);
  for (size_t f = 0; f < mesh.NumTri(); ++f) {
    const auto tri = mesh.GetTriVerts(f);
    const auto a = mesh.GetVertPos(tri[0]);
    const auto cross =
        la::cross(mesh.GetVertPos(tri[1]) - a, mesh.GetVertPos(tri[2]) - a);
    ASSERT_GT(la::length(cross), 0);
    const auto normal = la::normalize(cross);
    for (const auto& point : points) {
      EXPECT_LE(la::dot(normal, point - a), 1e-10) << "Face " << f;
    }
  }
}

double HullWinding(const Manifold& hull, const vec3& point) {
  const auto mesh = hull.GetMeshGL64();
  double angle = 0;
  for (size_t f = 0; f < mesh.NumTri(); ++f) {
    const auto tri = mesh.GetTriVerts(f);
    const auto u = mesh.GetVertPos(tri[0]) - point;
    const auto v = mesh.GetVertPos(tri[1]) - point;
    const auto w = mesh.GetVertPos(tri[2]) - point;
    const auto lu = la::length(u), lv = la::length(v), lw = la::length(w);
    angle += 2 * std::atan2(la::dot(u, la::cross(v, w)),
                            lu * lv * lw + la::dot(u, v) * lw +
                                la::dot(v, w) * lu + la::dot(w, u) * lv);
  }
  return angle / (4 * std::acos(-1.0));
}

}  // namespace

TEST(Hull, TranslatedMinkowskiSupportingPlanes) {
  // A translated triangle swept by a low-resolution sphere. Hex literals
  // preserve the captured double coordinates, including source roundoff.
  const std::vector<vec3> captured = {
      {0x1.87de2a6aea95fp-2, 0x1.40a13ca48c6f8p+6, -0x1.56e2651d8d437p+0},
      {0x1.1517a7bdb3891p-2, 0x1.40a13ca48c6f8p+6, -0x1.119c7b2e20612p+0},
      {-0x1.00a6cad4d5761p-52, 0x1.40a13ca48c6f8p+6, -0x1.e9d5b505a53bcp-1},
      {-0x1.1517a7bdb3898p-2, 0x1.40a13ca48c6f8p+6, -0x1.119c7b2e20612p+0},
      {-0x1.87de2a6aea967p-2, 0x1.40a13ca48c6f8p+6, -0x1.56e2651d8d437p+0},
      {-0x1.1517a7bdb389ap-2, 0x1.40a13ca48c6f8p+6, -0x1.9c284f0cfa25cp+0},
      {-0x1.6cb7203bebf8ap-52, 0x1.40a13ca48c6f8p+6, -0x1.b8d9efb847e90p+0},
      {0x1.1517a7bdb3890p-2, 0x1.40a13ca48c6f8p+6, -0x1.9c284f0cfa25cp+0},
      {0x1.d906bcf328d44p-1, 0x1.3e770d5511086p+6, -0x1.56e2651d8d437p+0},
      {0x1.4e7ae9144f0fap-1, 0x1.3e770d5511086p+6, -0x1.5f49e126cb773p-1},
      {-0x1.b4e3f25d0099ep-53, 0x1.3e770d5511086p+6, -0x1.a97c1a8fe3650p-2},
      {-0x1.4e7ae9144f0fdp-1, 0x1.3e770d5511086p+6, -0x1.5f49e126cb772p-1},
      {-0x1.d906bcf328d48p-1, 0x1.3e770d5511086p+6, -0x1.56e2651d8d436p+0},
      {-0x1.4e7ae9144f0ffp-1, 0x1.3e770d5511086p+6, -0x1.fe1fd9a7b4cb4p+0},
      {-0x1.df55952eeb740p-52, 0x1.3e770d5511086p+6, -0x1.21b2e1cb90d6dp+1},
      {0x1.4e7ae9144f0f8p-1, 0x1.3e770d5511086p+6, -0x1.fe1fd9a7b4cb6p+0},
      {0x1.d906bcf328d44p-1, 0x1.3b6751003b334p+6, -0x1.56e2651d8d437p+0},
      {0x1.4e7ae9144f0fap-1, 0x1.3b6751003b334p+6, -0x1.5f49e126cb773p-1},
      {-0x1.b4e3f25d0099ep-53, 0x1.3b6751003b334p+6, -0x1.a97c1a8fe3650p-2},
      {-0x1.4e7ae9144f0fdp-1, 0x1.3b6751003b334p+6, -0x1.5f49e126cb772p-1},
      {-0x1.d906bcf328d48p-1, 0x1.3b6751003b334p+6, -0x1.56e2651d8d436p+0},
      {-0x1.4e7ae9144f0ffp-1, 0x1.3b6751003b334p+6, -0x1.fe1fd9a7b4cb4p+0},
      {-0x1.df55952eeb740p-52, 0x1.3b6751003b334p+6, -0x1.21b2e1cb90d6dp+1},
      {0x1.4e7ae9144f0f8p-1, 0x1.3b6751003b334p+6, -0x1.fe1fd9a7b4cb6p+0},
      {0x1.87de2a6aea961p-2, 0x1.393d21b0bfcc2p+6, -0x1.56e2651d8d437p+0},
      {0x1.1517a7bdb3893p-2, 0x1.393d21b0bfcc2p+6, -0x1.119c7b2e20612p+0},
      {-0x1.00a6cad4d5761p-52, 0x1.393d21b0bfcc2p+6, -0x1.e9d5b505a53bcp-1},
      {-0x1.1517a7bdb389ap-2, 0x1.393d21b0bfcc2p+6, -0x1.119c7b2e20611p+0},
      {-0x1.87de2a6aea969p-2, 0x1.393d21b0bfcc2p+6, -0x1.56e2651d8d437p+0},
      {-0x1.1517a7bdb389bp-2, 0x1.393d21b0bfcc2p+6, -0x1.9c284f0cfa25cp+0},
      {-0x1.6cb7203bebf8ap-52, 0x1.393d21b0bfcc2p+6, -0x1.b8d9efb847e90p+0},
      {0x1.1517a7bdb3891p-2, 0x1.393d21b0bfcc2p+6, -0x1.9c284f0cfa25dp+0},
      {0x1.55a7513f22807p+1, 0x1.390d970e5c86cp+6, -0x1.24ab8bf1c52ddp+1},
      {0x1.474e80e97b9eep+1, 0x1.390d970e5c86cp+6, -0x1.020896fa0ebcap+1},
      {0x1.24ab8bf1c52dbp+1, 0x1.390d970e5c86cp+6, -0x1.e75f8d48cfb61p+0},
      {0x1.020896fa0ebc8p+1, 0x1.390d970e5c86cp+6, -0x1.020896fa0ebcap+1},
      {0x1.e75f8d48cfb5dp+0, 0x1.390d970e5c86cp+6, -0x1.24ab8bf1c52ddp+1},
      {0x1.020896fa0ebc8p+1, 0x1.390d970e5c86cp+6, -0x1.474e80e97b9f0p+1},
      {0x1.24ab8bf1c52dbp+1, 0x1.390d970e5c86cp+6, -0x1.55a7513f22809p+1},
      {0x1.474e80e97b9eep+1, 0x1.390d970e5c86cp+6, -0x1.474e80e97b9f0p+1},
      {0x1.9aed3b2e8f62cp+1, 0x1.36e367bee11fap+6, -0x1.24ab8bf1c52ddp+1},
      {0x1.784a4636d8f1ap+1, 0x1.36e367bee11fap+6, -0x1.a219a35962d3cp+0},
      {0x1.24ab8bf1c52dbp+1, 0x1.36e367bee11fap+6, -0x1.5cd3b969f5f17p+0},
      {0x1.a219a35962d38p+0, 0x1.36e367bee11fap+6, -0x1.a219a35962d3cp+0},
      {0x1.5cd3b969f5f13p+0, 0x1.36e367bee11fap+6, -0x1.24ab8bf1c52ddp+1},
      {0x1.a219a35962d38p+0, 0x1.36e367bee11fap+6, -0x1.784a4636d8f1cp+1},
      {0x1.24ab8bf1c52dbp+1, 0x1.36e367bee11fap+6, -0x1.9aed3b2e8f62ep+1},
      {0x1.784a4636d8f1ap+1, 0x1.36e367bee11fap+6, -0x1.784a4636d8f1cp+1},
      {0x1.9aed3b2e8f62cp+1, 0x1.33d3ab6a0b4a8p+6, -0x1.24ab8bf1c52ddp+1},
      {0x1.784a4636d8f1ap+1, 0x1.33d3ab6a0b4a8p+6, -0x1.a219a35962d3cp+0},
      {0x1.24ab8bf1c52dbp+1, 0x1.33d3ab6a0b4a8p+6, -0x1.5cd3b969f5f17p+0},
      {0x1.a219a35962d38p+0, 0x1.33d3ab6a0b4a8p+6, -0x1.a219a35962d3cp+0},
      {0x1.5cd3b969f5f13p+0, 0x1.33d3ab6a0b4a8p+6, -0x1.24ab8bf1c52ddp+1},
      {0x1.a219a35962d38p+0, 0x1.33d3ab6a0b4a8p+6, -0x1.784a4636d8f1cp+1},
      {0x1.24ab8bf1c52dbp+1, 0x1.33d3ab6a0b4a8p+6, -0x1.9aed3b2e8f62ep+1},
      {0x1.784a4636d8f1ap+1, 0x1.33d3ab6a0b4a8p+6, -0x1.784a4636d8f1cp+1},
      {0x1.55a7513f22808p+1, 0x1.31a97c1a8fe36p+6, -0x1.24ab8bf1c52ddp+1},
      {0x1.474e80e97b9eep+1, 0x1.31a97c1a8fe36p+6, -0x1.020896fa0ebcap+1},
      {0x1.24ab8bf1c52dbp+1, 0x1.31a97c1a8fe36p+6, -0x1.e75f8d48cfb61p+0},
      {0x1.020896fa0ebc8p+1, 0x1.31a97c1a8fe36p+6, -0x1.020896fa0ebcap+1},
      {0x1.e75f8d48cfb5dp+0, 0x1.31a97c1a8fe36p+6, -0x1.24ab8bf1c52ddp+1},
      {0x1.020896fa0ebc8p+1, 0x1.31a97c1a8fe36p+6, -0x1.474e80e97b9f0p+1},
      {0x1.24ab8bf1c52dbp+1, 0x1.31a97c1a8fe36p+6, -0x1.55a7513f2280ap+1},
      {0x1.474e80e97b9eep+1, 0x1.31a97c1a8fe36p+6, -0x1.474e80e97b9f0p+1},
      {0x1.87de2a6aea958p-2, 0x1.390d970e5c86cp+6, -0x1.9de5e554c3b9dp+1},
      {0x1.1517a7bdb388ap-2, 0x1.390d970e5c86cp+6, -0x1.7b42f05d0d48ap+1},
      {-0x1.48e8b213a9d2fp-51, 0x1.390d970e5c86cp+6, -0x1.6cea200766671p+1},
      {-0x1.1517a7bdb389fp-2, 0x1.390d970e5c86cp+6, -0x1.7b42f05d0d48ap+1},
      {-0x1.87de2a6aea96ep-2, 0x1.390d970e5c86cp+6, -0x1.9de5e554c3b9dp+1},
      {-0x1.1517a7bdb38a1p-2, 0x1.390d970e5c86cp+6, -0x1.c088da4c7a2b0p+1},
      {-0x1.7ef0dcc735143p-51, 0x1.390d970e5c86cp+6, -0x1.cee1aaa2210c9p+1},
      {0x1.1517a7bdb3889p-2, 0x1.390d970e5c86cp+6, -0x1.c088da4c7a2b0p+1},
      {0x1.d906bcf328d41p-1, 0x1.36e367bee11fap+6, -0x1.9de5e554c3b9dp+1},
      {0x1.4e7ae9144f0f7p-1, 0x1.36e367bee11fap+6, -0x1.4a472b0faff5ep+1},
      {-0x1.35ce49407f3e6p-51, 0x1.36e367bee11fap+6, -0x1.27a43617f984cp+1},
      {-0x1.4e7ae9144f100p-1, 0x1.36e367bee11fap+6, -0x1.4a472b0faff5ep+1},
      {-0x1.d906bcf328d4bp-1, 0x1.36e367bee11fap+6, -0x1.9de5e554c3b9dp+1},
      {-0x1.4e7ae9144f102p-1, 0x1.36e367bee11fap+6, -0x1.f1849f99d77dcp+1},
      {-0x1.b8401740b4d1ep-51, 0x1.36e367bee11fap+6, -0x1.0a13ca48c6f77p+2},
      {0x1.4e7ae9144f0f5p-1, 0x1.36e367bee11fap+6, -0x1.f1849f99d77dcp+1},
      {0x1.d906bcf328d41p-1, 0x1.33d3ab6a0b4a8p+6, -0x1.9de5e554c3b9dp+1},
      {0x1.4e7ae9144f0f7p-1, 0x1.33d3ab6a0b4a8p+6, -0x1.4a472b0faff5ep+1},
      {-0x1.35ce49407f3e6p-51, 0x1.33d3ab6a0b4a8p+6, -0x1.27a43617f984cp+1},
      {-0x1.4e7ae9144f100p-1, 0x1.33d3ab6a0b4a8p+6, -0x1.4a472b0faff5ep+1},
      {-0x1.d906bcf328d4bp-1, 0x1.33d3ab6a0b4a8p+6, -0x1.9de5e554c3b9dp+1},
      {-0x1.4e7ae9144f102p-1, 0x1.33d3ab6a0b4a8p+6, -0x1.f1849f99d77dcp+1},
      {-0x1.b8401740b4d1ep-51, 0x1.33d3ab6a0b4a8p+6, -0x1.0a13ca48c6f77p+2},
      {0x1.4e7ae9144f0f5p-1, 0x1.33d3ab6a0b4a8p+6, -0x1.f1849f99d77dcp+1},
      {0x1.87de2a6aea95ap-2, 0x1.31a97c1a8fe36p+6, -0x1.9de5e554c3b9dp+1},
      {0x1.1517a7bdb388cp-2, 0x1.31a97c1a8fe36p+6, -0x1.7b42f05d0d48ap+1},
      {-0x1.48e8b213a9d2fp-51, 0x1.31a97c1a8fe36p+6, -0x1.6cea200766670p+1},
      {-0x1.1517a7bdb38a1p-2, 0x1.31a97c1a8fe36p+6, -0x1.7b42f05d0d48ap+1},
      {-0x1.87de2a6aea970p-2, 0x1.31a97c1a8fe36p+6, -0x1.9de5e554c3b9dp+1},
      {-0x1.1517a7bdb38a2p-2, 0x1.31a97c1a8fe36p+6, -0x1.c088da4c7a2b0p+1},
      {-0x1.7ef0dcc735143p-51, 0x1.31a97c1a8fe36p+6, -0x1.cee1aaa2210cap+1},
      {0x1.1517a7bdb388ap-2, 0x1.31a97c1a8fe36p+6, -0x1.c088da4c7a2b0p+1},
  };
  for (const auto origin : {vec3(0.0), captured[0]}) {
    auto points = captured;
    for (auto& point : points) point -= origin;
    const auto hull = Manifold::Hull(points);
    ASSERT_EQ(hull.Status(), Manifold::Error::NoError);
    ASSERT_FALSE(hull.IsEmpty());
    EXPECT_EQ(hull.Genus(), 0);
    ExpectSupportingPlanes(hull, points);
    EXPECT_NEAR(hull.Volume(), 19.84668710976564, 1e-10);
    EXPECT_NEAR(hull.SurfaceArea(), 41.99514594556986, 1e-10);
    EXPECT_NEAR(HullWinding(hull, vec3(.95, 78.38, -2.84) - origin), 1, 1e-10);
  }
}

TEST(Hull, TranslatedCapsule) {
  const auto sphere = Manifold::Sphere(1, 8);
  for (const auto offset : {vec3(0.0), vec3(0, 10, 0)}) {
    const std::vector<Manifold> ends = {
        sphere.Translate(offset), sphere.Translate(offset + vec3(2, -2, 1))};
    std::vector<vec3> points;
    for (const auto& end : ends) {
      const auto mesh = end.GetMeshGL64();
      for (size_t i = 0; i < mesh.NumVert(); ++i)
        points.push_back(mesh.GetVertPos(i));
    }
    const auto hull = Manifold::Hull(ends);
    ASSERT_EQ(hull.Status(), Manifold::Error::NoError);
    ASSERT_FALSE(hull.IsEmpty());
    EXPECT_EQ(hull.Genus(), 0);
    ExpectSupportingPlanes(hull, points);
    EXPECT_NEAR(hull.Volume(), 10.771236166328256, 1e-10);
  }
}

TEST(Hull, ParametricTranslatedTriangleSphere) {
  constexpr int segments = 8;
  const double pi = std::acos(-1.0);
  std::vector<vec3> points;
  for (const vec3 center : {vec3(0, 60, 0), vec3(2, 58, 0), vec3(0, 58, -1)}) {
    for (int i = 0; i < segments / 2; ++i) {
      const double phi = pi * (i + 0.5) / (segments / 2);
      for (int j = 0; j < segments; ++j) {
        const double theta = 2 * pi * j / segments;
        points.push_back(center + vec3(std::sin(phi) * std::cos(theta),
                                       std::cos(phi),
                                       std::sin(phi) * std::sin(theta)));
      }
    }
  }
  for (const vec3 origin : {vec3(0.0), vec3(0, 60, 0)}) {
    auto cloud = points;
    for (auto& point : cloud) point -= origin;
    const auto hull = Manifold::Hull(cloud);
    ASSERT_EQ(hull.Status(), Manifold::Error::NoError);
    ASSERT_FALSE(hull.IsEmpty());
    EXPECT_EQ(hull.Genus(), 0);
    ExpectSupportingPlanes(hull, cloud);
    EXPECT_NEAR(hull.Volume(), 17.863848845758767, 1e-10);
  }
}

TEST(Hull, TranslatedNearlyCoplanar) {
  for (const auto shift : {vec3(0.0), vec3(10, -20, 30)}) {
    std::vector<vec3> points = {{0, 0, 0},       {1, 0, 0},
                                {0, 1, 0},       {0.25, 0.25, 1e-4},
                                {0.25, 0.25, 0}, {0.5, 0.25, 0}};
    for (auto& point : points) point += shift;
    for (const size_t count : {size_t(4), points.size()}) {
      const std::vector<vec3> cloud(points.begin(), points.begin() + count);
      const auto hull = Manifold::Hull(cloud);
      ASSERT_EQ(hull.Status(), Manifold::Error::NoError);
      ASSERT_FALSE(hull.IsEmpty());
      ExpectSupportingPlanes(hull, cloud);
      EXPECT_NEAR(hull.Volume(), 1e-4 / 6, 1e-12);
    }
  }
}

TEST(Hull, TranslatedDegenerateInputs) {
  const vec3 shift(10, -20, 30);
  for (auto points :
       {std::vector<vec3>(5, vec3(0.0)),
        std::vector<vec3>{
            {0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {4, 0, 0}},
        std::vector<vec3>{
            {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {.5, .5, 0}}}) {
    for (auto& point : points) point += shift;
    const auto hull = Manifold::Hull(points);
    EXPECT_EQ(hull.Status(), Manifold::Error::NoError);
    EXPECT_TRUE(hull.IsEmpty());
  }
}
