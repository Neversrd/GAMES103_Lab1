#pragma once
#include "CoreMinimal.h"

static FQuat QuaternionCrossProduct(const FQuat& q1, const FQuat& q2) {
  FQuat result;
  FVector v1(q1.X, q1.Y, q1.Z);
  FVector v2(q2.X, q2.Y, q2.Z);
  /*FVector v1(q1.Z, q1.Y, q1.X);
  FVector v2(q2.Z, q2.Y, q2.X);*/

  result.W = q1.W * q2.W - FVector::DotProduct(v1, v2);

  v1 = q1.W * v2 + q2.W * v1 + v1 ^ v2;
  result.X = v1.X;
  result.Y = v1.Y;
  result.Z = v1.Z;
  return result;
}

inline float ComputePhiPlane(const FVector& x, const FVector& p,
                                    const FVector& n) {
  return FVector::DotProduct(x - p, n);
}

static FMatrix ToSkewSymmetricMatrix(const FVector& v) {
  FMatrix result(EForceInit::ForceInitToZero);

  result.M[0][1] = -v.Z;
  result.M[0][2] = v.Y;

  result.M[1][0] = v.Z;
  result.M[1][2] = -v.X;

  result.M[2][0] = -v.Y;
  result.M[2][1] = v.X;

  result.M[3][3] = 1.f;
  return result;
}

static FMatrix MatrixSubstraction(const FMatrix& m1, const FMatrix& m2) {
  FMatrix result = FMatrix::Identity;
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      result.M[i][j] = m1.M[i][j] - m2.M[i][j];
    }
  }
  return result;
}

static FVector MatrixVectorMultiplication(const FMatrix& m,
                                                 const FVector& v) {
  FVector result;
  for (size_t i = 0; i < 3; i++) {
    result[i] = m.M[i][0] * v[0] + m.M[i][1] * v[1] + m.M[i][2] * v[2];
  }
  return result;
}

static FMatrix VectorOuterProduct(const FVector& v1, const FVector& v2) {
  FMatrix result = FMatrix::Identity;
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      result.M[i][j] = v1[i] * v2[j];
    }
  }
  return result;
}

static FMatrix Get_Rotation(const FMatrix& F) {
  FMatrix C(EForceInit::ForceInitToZero);
  for (int ii = 0; ii < 3; ii++)
    for (int jj = 0; jj < 3; jj++)
      for (int kk = 0; kk < 3; kk++) C.M[ii][jj] += F.M[kk][ii] * F.M[kk][jj];

  FMatrix C2(EForceInit::ForceInitToZero);
  for (int ii = 0; ii < 3; ii++)
    for (int jj = 0; jj < 3; jj++)
      for (int kk = 0; kk < 3; kk++) C2.M[ii][jj] += C.M[ii][kk] * C.M[jj][kk];

  float det = F.M[0][0] * F.M[1][1] * F.M[2][2] + F.M[0][1] * F.M[1][2] * F.M[2][0] +
              F.M[1][0] * F.M[2][1] * F.M[0][2] - F.M[0][2] * F.M[1][1] * F.M[2][0] -
              F.M[0][1] * F.M[1][0] * F.M[2][2] - F.M[0][0] * F.M[1][2] * F.M[2][1];

  float I_c = C.M[0][0] + C.M[1][1] + C.M[2][2];
  float I_c2 = I_c * I_c;
  float II_c = 0.5f * (I_c2 - C2.M[0][0] - C2.M[1][1] - C2.M[2][2]);
  float III_c = det * det;
  float k = I_c2 - 3 * II_c;

  FMatrix inv_U(EForceInit::ForceInitToZero);
  if (k < 1e-10f) {
    float inv_lambda = 1 / FMath::Sqrt(I_c / 3);
    inv_U.M[0][0] = inv_lambda;
    inv_U.M[1][1] = inv_lambda;
    inv_U.M[2][2] = inv_lambda;
  } else {
    float l = I_c * (I_c * I_c - 4.5f * II_c) + 13.5f * III_c;
    float k_root = FMath::Sqrt(k);
    float value = l / (k * k_root);
    if (value < -1.0f) value = -1.0f;
    if (value > 1.0f) value = 1.0f;
    float phi = FMath::Acos(value);
    float lambda2 = (I_c + 2 * k_root * FMath::Cos(phi / 3)) / 3.0f;
    float lambda = FMath::Sqrt(lambda2);

    float III_u = FMath::Sqrt(III_c);
    if (det < 0) III_u = -III_u;
    float I_u = lambda + FMath::Sqrt(-lambda2 + I_c + 2 * III_u / lambda);
    float II_u = (I_u * I_u - I_c) * 0.5f;

    float inv_rate, factor;
    inv_rate = 1 / (I_u * II_u - III_u);
    factor = I_u * III_u * inv_rate;

    FMatrix U(EForceInit::ForceInitToZero);
    U.M[0][0] = factor;
    U.M[1][1] = factor;
    U.M[2][2] = factor;

    factor = (I_u * I_u - II_u) * inv_rate;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        U.M[i][j] += factor * C.M[i][j] - inv_rate * C2.M[i][j];

    inv_rate = 1 / III_u;
    factor = II_u * inv_rate;
    inv_U.M[0][0] = factor;
    inv_U.M[1][1] = factor;
    inv_U.M[2][2] = factor;

    factor = -I_u * inv_rate;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        inv_U.M[i][j] += factor * U.M[i][j] + inv_rate * C.M[i][j];
  }

  FMatrix R(EForceInit::ForceInitToZero);
  for (int ii = 0; ii < 3; ii++)
    for (int jj = 0; jj < 3; jj++)
      for (int kk = 0; kk < 3; kk++)
        R.M[ii][jj] += F.M[ii][kk] * inv_U.M[kk][jj];
  R.M[3][3] = 1;
  return R;
}