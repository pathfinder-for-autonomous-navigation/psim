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
#include <lin.hpp>

namespace orb
{

/** 
 * Solve C'xC = A'xA + B'xB for C, where A, B, and C are square matrices
 * @param[in] A
 * @param[in] B
 * @param[out] C
 */
template <typename T, lin::size_t N>
void matrix_hypot(const lin::Matrix<T,N,N>& A,const lin::Matrix<T,N,N>& B,lin::Matrix<T,N,N>& C){
    lin::Matrix<T,N*2,N> M;
    lin::Matrix<T,N*2,N*2> junk;
    lin::Matrix<T,N*2,N> R;
    lin::ref<N, N> (M, 0, 0) = A;
    lin::ref<N, N> (M, N, 0) = B;
    lin::qr(M, junk, R);
    C= lin::ref<N, N> (R, 0, 0);
}

}