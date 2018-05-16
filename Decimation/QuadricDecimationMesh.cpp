/*************************************************************************************************
 *
 * Modeling and animation (TNM079) 2007
 * Code base for lab assignments. Copyright:
 *   Gunnar Johansson (gunnar.johansson@itn.liu.se)
 *   Ken Museth (ken.museth@itn.liu.se)
 *   Michael Bang Nielsen (bang@daimi.au.dk)
 *   Ola Nilsson (ola.nilsson@itn.liu.se)
 *   Andreas Sderstrm (andreas.soderstrom@itn.liu.se)
 *
 *************************************************************************************************/
#include "QuadricDecimationMesh.h"

const QuadricDecimationMesh::VisualizationMode
    QuadricDecimationMesh::QuadricIsoSurfaces =
        NewVisualizationMode("Quadric Iso Surfaces");

void QuadricDecimationMesh::Initialize() {
  // Allocate memory for the quadric array
  size_t numVerts = mVerts.size();
  mQuadrics.reserve(numVerts);
  std::streamsize width = std::cerr.precision(); // store stream precision
  for (size_t i = 0; i < numVerts; i++) {

    // Compute quadric for vertex i here
    mQuadrics.push_back(createQuadricForVert(i));

    // Calculate initial error, should be numerically close to 0

    Vector3<float> v0 = mVerts[i].pos;
    Vector4<float> v(v0[0], v0[1], v0[2], 1);
    Matrix4x4<float> m = mQuadrics.back();

    float error = v * (m * v);
    std::cerr << std::scientific << std::setprecision(2) << error << " ";
  }
  std::cerr << std::setprecision(width) << std::fixed; // reset stream precision

  // Run the initialize for the parent class to initialize the edge collapses
  DecimationMesh::Initialize();
}

/*! \lab2 Implement the computeCollapse here */
/*!
 * \param[in,out] collapse The edge collapse object to (re-)compute,
 * DecimationMesh::EdgeCollapse
 */
void QuadricDecimationMesh::computeCollapse(EdgeCollapse *collapse) {
  // Compute collapse->position and collapse->cost here
  // based on the quadrics at the edge endpoints
  const size_t vInd1 = mEdges[collapse->halfEdge].vert;
  const size_t vInd2 = mEdges[mEdges[collapse->halfEdge].pair].vert;

  Matrix4x4<float> Q = mQuadrics[vInd1] + mQuadrics[vInd2];
  float inversionValues[4][4] = {
    {Q(0,0), Q(0,1), Q(0,2), Q(0,3)},
    {Q(0,1), Q(1,1), Q(1,2), Q(1,3)},
    {Q(0,2), Q(1,2), Q(2,2), Q(2,3)},
    {0, 0, 0, 1}
  };
  Matrix4x4<float> ToInverse(inversionValues);

  if (!ToInverse.IsSingular()) {
    Matrix4x4<float> inverse = ToInverse.Inverse();
    auto v = Vector4<float>(
      inverse(0, 3), inverse(1, 3), inverse(2, 3), 1
    );
    collapse->position = Vector3<float>(v[0], v[1], v[2]);
    collapse->cost = v * (Q * v);
  } else {
    auto p1 = v(vInd1).pos;
    auto p2 = v(vInd2).pos;
    auto v1 = Vector4<float>(p1[0], p1[1], p1[2], 1);
    auto v2 = Vector4<float>(p2[0], p2[1], p2[2], 1);
    auto midpoint = 0.5 * (v1 + v2);
    float v1Cost = v1 * (Q * v1);
    float midCost = midpoint * (Q * midpoint);
    float v2Cost = v2 * (Q * v2);

    if (v1Cost <= midCost && v1Cost <= v2Cost) {
      collapse->position = p1;
      collapse->cost = v1Cost;
    } else if (midCost <= v1Cost && midCost <= v2Cost) {
      collapse->position = Vector3<float>(midpoint[0], midpoint[1], midpoint[2]);
      collapse->cost = midCost;
    } else {
      collapse->position = p2;
      collapse->cost = v2Cost;
    }
  }
}

/*! After each edge collapse the vertex properties need to be updated */
void QuadricDecimationMesh::updateVertexProperties(size_t ind) {
  DecimationMesh::updateVertexProperties(ind);
  mQuadrics[ind] = createQuadricForVert(ind);
}

/*!
 * \param[in] indx vertex index, points into HalfEdgeMesh::mVerts
 */
Matrix4x4<float>
QuadricDecimationMesh::createQuadricForVert(size_t indx) const {
  Vertex vertex = v(indx);
  std::vector<size_t> neighborFaces = FindNeighborFaces(indx);
  float q[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
  Matrix4x4<float> Q(q);

  for (size_t faceIndex : neighborFaces) {
    Q += createQuadricForFace(faceIndex);
  }

  // The quadric for a vertex is the sum of all the quadrics for the adjacent
  // faces Tip: Matrix4x4 has an operator +=
  return Q;
}

/*!
 * \param[in] indx face index, points into HalfEdgeMesh::mFaces
 */
Matrix4x4<float>
QuadricDecimationMesh::createQuadricForFace(size_t indx) const {
  // Calculate the quadric (outer product of plane parameters) for a face
  // here using the formula from Garland and Heckbert
  float q[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};

  Face face = f(indx);
  Vector3<float> n = face.normal;
  Vector3<float> p = v(e(face.edge).vert).pos;
  float planeParameters[] =
    { n[0], n[1], n[2], - n[0]*p[0] - n[1]*p[1] - n[2]*p[2] };

  for (int c = 0; c < 4; c++) {
    for (int r = 0; r < 4; r++) {
      q[r][c] = planeParameters[r]*planeParameters[c];
    }
  }

  return Matrix4x4<float>(q);
}

void QuadricDecimationMesh::Render() {
  DecimationMesh::Render();

  glEnable(GL_LIGHTING);
  glMatrixMode(GL_MODELVIEW);

  if (mVisualizationMode == QuadricIsoSurfaces) {
    // Apply transform
    glPushMatrix(); // Push modelview matrix onto stack

    // Implement the quadric visualization here
    std::cout << "Quadric visualization not implemented" << std::endl;

    // Restore modelview matrix
    glPopMatrix();
  }
}
