/*
MIT License

Copyright (c) 2020 Pathfinder for Autonomous Navigation (PAN)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/**
 * \file kalman_utl.h
 * \author Nathan Zimmerberg
 * \date 28 APR 2020
 * \brief Utility functions for Kalman filters.
 */

#pragma once
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <array>
#include <lin/core.hpp>
#include <lin/references.hpp>
#include <lin/factorizations/qr.hpp>

namespace orb
{

/** 
 * Solve C'xC = A'xA + B'xB for C, where A, B, and C are square matrices
 * @param[in] A
 * @param[in] B
 * @param[out] C
 * REFERENCES:
 * D. F. Crouse, "Basic tracking using nonlinear 3D monostatic and
 *    bistatic measurements," IEEE Aerospace and Electronic Systems
 *    Magazine, vol. 29, no. 8, Part II, pp. 4-53, Aug. 2014.
 */
template <typename T, lin::size_t N>
void matrix_hypot(const lin::Matrix<T,N,N>& A,const lin::Matrix<T,N,N>& B,lin::Matrix<T,N,N>& C){
    lin::Matrix<T,N*2,N> M;
    lin::Matrix<T,N*2,N> junk;
    lin::ref<N, N> (M, 0, 0) = A;
    lin::ref<N, N> (M, N, 0) = B;
    lin::qr(M, junk, C);
}

/**
 * Potter Algorithm as described in
 * "Factorization Methods for Discrete Sequential Estimation", by Gerald J. Bierman
 * Appendix II.E
 * z= A*x + noise with variance 1.0/(invstddiv*invstddiv)
 * The measurement noise is assumed to be independent zero mean gaussian
 * @param[in,out] x: State estimate
 * @param[in,out] S: square root covariance P= S*S^T
 * @param[in] z: Measurement
 * @param[in] A: Measurement coefficents
 * @param[in] invstddiv(finite): Inverse of measurement standard diviation
 */
template <typename T, lin::size_t N>
void potter_measurement_update(lin::Vector<T,N>& x, lin::Matrix<T,N,N>& S, const T& z, const lin::RowVector<T,N>& A, const T& invstddiv){
    T one= 1;
    lin::Vector<T,N> v= lin::transpose(A*S);//n^2
    v= v*invstddiv;
    T sigma= one/(lin::fro(v)+one);
    T delta= (z-lin::dot(A,x))*invstddiv;// predicted residuals 
    lin::Vector<T,N> K= S*v;//n^2
    x= x+K*(delta*sigma);
    T gamma= sigma/(one+std::sqrt(sigma));
    S= S-(gamma*K)*lin::transpose(v);//n^2
}




} //namespace orb