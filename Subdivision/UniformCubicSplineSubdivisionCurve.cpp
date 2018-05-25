
#include "UniformCubicSplineSubdivisionCurve.h"

UniformCubicSplineSubdivisionCurve::UniformCubicSplineSubdivisionCurve(
    const std::vector<Vector3<float>> &joints, Vector3<float> lineColor,
    float lineWidth)
    : mCoefficients(joints), mControlPolygon(joints) {
  this->mLineColor = lineColor;
  this->mLineWidth = lineWidth;
}

// calc_c_prime and calc_c_prime_half correspond to equation 30 from
// the lab description
Vector3<float> calc_c_prime(int i, const std::vector<Vector3<float>>& coeffs) {
  return 1.0 / 8.0 * (coeffs[i - 1] + 6 * coeffs[i] + coeffs[i + 1]);
}

Vector3<float> calc_c_prime_half(int i, const std::vector<Vector3<float>>& coeffs) {
  return 1.0 / 2.0 * (coeffs[i] + coeffs[i + 1]);
}

void UniformCubicSplineSubdivisionCurve::Subdivide() {
  // Allocate space for new coefficients
  std::vector<Vector3<float>> newc;

  assert(mCoefficients.size() > 4 && "Need at least 5 points to subdivide");
  size_t totalNewcSize = mCoefficients.size() * 2 - 1;

  newc.push_back(mCoefficients[0]);
  newc.push_back(calc_c_prime_half(0, mCoefficients));
  for (int i = 1; i < mCoefficients.size() - 1; i++) {
    newc.push_back(calc_c_prime(i, mCoefficients));
    newc.push_back(calc_c_prime_half(i, mCoefficients));
  }
  newc.push_back(mCoefficients.back());

  assert(newc.size() == totalNewcSize &&
  	 "Incorrect number of new coefficients!");

  mCoefficients = newc;
}

void UniformCubicSplineSubdivisionCurve::Render() {
  // Apply transform
  glPushMatrix(); // Push modelview matrix onto stack

  // Convert transform-matrix to format matching GL matrix format
  // Load transform into modelview matrix
  glMultMatrixf(mTransform.ToGLMatrix().GetArrayPtr());

  mControlPolygon.Render();

  // save line point and color states
  glPushAttrib(GL_POINT_BIT | GL_LINE_BIT | GL_CURRENT_BIT);

  // draw segments
  glLineWidth(mLineWidth);
  glColor3fv(mLineColor.GetArrayPtr());
  glBegin(GL_LINE_STRIP);
  // just draw the spline as a series of connected linear segments
  for (size_t i = 0; i < mCoefficients.size(); i++) {
    glVertex3fv(mCoefficients.at(i).GetArrayPtr());
  }
  glEnd();

  // restore attribs
  glPopAttrib();

  glPopMatrix();

  GLObject::Render();
}
