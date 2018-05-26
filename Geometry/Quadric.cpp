/*************************************************************************************************
 *
 * Modeling and animation (TNM079) 2007
 * Code base for lab assignments. Copyright:
 *   Gunnar Johansson (gunnar.johansson@itn.liu.se)
 *   Ken Museth (ken.museth@itn.liu.se)
 *   Michael Bang Nielsen (bang@daimi.au.dk)
 *   Ola Nilsson (ola.nilsson@itn.liu.se)
 *   Andreas Sˆderstrˆm (andreas.soderstrom@itn.liu.se)
 *
 *************************************************************************************************/
#include "Quadric.h"

Quadric::Quadric(const Matrix4x4<float> &q) { this->mQuadric = q; }

Quadric::~Quadric() {}

/*!
 * Evaluation of world coordinates are done through either transformation
 * of the world-coordinates by mWorld2Obj, or transformation of the quadric
 * coefficient matrix by GetTransform() ONCE (see Section 2.2 in lab text).
 */
float Quadric::GetValue(float x, float y, float z) const {
  TransformW2O(x, y, z);
  Vector4<float> p = Vector4<float>(x, y, z, 1);
  float res = (mQuadric * p) * p;
  return res;
}

/*!
 * Use the quadric matrix to evaluate the gradient.
 */
Vector3<float> Quadric::GetGradient(float x, float y, float z) const {
  TransformW2O(x, y, z);
  Vector4<float> p = Vector4<float>(x, y, z, 1);

  Vector3<float> res(0, 0, 0);
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 4; col++) {
      res[row] += mQuadric(row, col) * p[row];
    }
  }

  return res * 2;
}
