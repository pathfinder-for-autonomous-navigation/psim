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
 * \file OrbitEstimate.hpp
 * \author Kyle Krol
 * \date 02 FEB 2021
 * \brief A class to provide an estimate of a sat's orbit given a GPS position
 *        and velocity readout.
 */

#pragma once
#include <lin/core.hpp>
#include <lin/factorizations.hpp>
#include <lin/generators.hpp>
#include <lin/references.hpp>
#include <lin/substitutions.hpp>
#include "Orbit.h"

namespace orb
{

/** This wrapper around the orb::Orbit class intended to facilitate orbital
 *  state estimation of a single satellite given GPS data.
 */
class OrbitEstimate {
  public:
    /// \private
    /** Current orbital state estimate.
     */
    Orbit _orbit;

    /// \private
    /** Square root of the state covariance matrix (also denoted as S).
     */
    lin::Matrixd<6, 6> _sqrtP = lin::nans<lin::Matrixd<6, 6>>();

    /**
     */
    int64_t nsgpstime() const{
        return _orbit.nsgpstime();
    }

    /** Return position of the sat (m).
     *
     *  If the orbit estimate isn't valid, the returned vector will contain
     *  only nans.
     */
    lin::Vector3d recef() const {
        return _orbit.recef();
    }

    /** Return velocity of the sat (m).
     *
     *  If the orbit estimate isn't valid, the returned vector will contain
     *  only nans.
     */
    lin::Vector3d vecef() const {
        return _orbit.vecef();
    }

    /** Return the cholesky factorization of the state covariance matrix.
     */
    lin::Matrixd<6, 6> S() const {
        return _sqrtP;
    }

    /** Return true if the orbit estimate is valid.
     *
     *  See orb::Orbit::valid. An orbit estimate has the additional condition
     *  that the covariance of the state estimate is finite.
     */
    bool valid() const {
        return _orbit.valid();
    }

    /// \private
    /** Helper function to determine if the orbit estimate is valid.
     */
    void _check_validity() {
        _orbit._check_validity();
        if (!_orbit.valid()) {
            _sqrtP = lin::nans<lin::Matrixd<6, 6>>();
        }
    }

    /** Construct an invalid orbit estimate.
     */
    OrbitEstimate() {}

    /** Construct an orbit estimate from time, position, velocity, and an
     *  initial state covariance.
     * 
     *  @param[in] ns_gps_time time since gps epoch (ns).
     *  @param[in] r_ecef      position of the center of mass of the sat (m).
     *  @param[in] v_ecef      velocity of the sat (m/s).
     *  @param[in] S           cholesky factorization of the state estimate
     *                         covariance.
     */
    OrbitEstimate(int64_t const &ns_gps_time,
                  lin::Vector3d const &r_ecef,
                  lin::Vector3d const &v_ecef,
                  lin::Matrixd<6, 6> const &S
                  ) :
        _orbit(ns_gps_time, r_ecef, v_ecef), _sqrtP(S) {
        _check_validity();
    }

    /** Apply a deltav to the orbit estimate.
     *
     *  @param[in] deltav_ecef the change in velocity in the ecef frame (m/s).
     */
    void applydeltav(lin::Vector3d const &deltav_ecef) {
        _orbit.applydeltav(deltav_ecef);
        // TODO : Update state covariance

        _check_validity();
    }

    /** Return the specific energy of the orbit estimate (J/kg).
     * 
     *  If the orbit estimate is invalid this returns nan.
     *
     *  @param[in] earth_rate_ecef Earth's angular rate in the ecef frame
     *                             (rad/s).
     */
    double specifienergy(lin::Vector3d const &earth_rate_ecef) const {
        return _orbit.specificenergy(earth_rate_ecef);
    }

    /************* Single Cycle Prediction and Estimation ******************/

    /** Perform a predition step only - i.e. no measurement is accounted for.
     *
     *  @param[in]  dt_ns
     *  @param[in]  earth_rate_ecef
     *  @param[in]  sqrtQ
     *  @param[out] specificenergy
     */
    void shortupdate(int32_t            const  dt_ns,
                     lin::Vector3d      const &earth_rate_ecef,
                     lin::Matrixd<6, 6> const &sqrtQ,
                     double                   &specificenergy
                     ) {
        lin::Matrixd<6, 6> F;
        _orbit.shortupdate(dt_ns, earth_rate_ecef, specificenergy, F);

        /* This leverages the square root formulation of the EKF covariance
         * prediction step:
         *
         *   qr([ S_k|k transpose(F_k) ]) = _ S_k+1|k
         *     ([       sqrt(Q_k)      ])
         */
        lin::Matrixd<12, 6> A, _;
        lin::ref<lin::Matrixd<6, 6>>(A, 0, 0) = _sqrtP * lin::transpose(F);
        lin::ref<lin::Matrixd<6, 6>>(A, 6, 0) = sqrtQ;
        lin::qr(A, _, _sqrtP);

        _check_validity();
    }

    /** Perform a predition step only - i.e. no measurement is accounted for.
     *
     *  @param[in]  dt_ns
     *  @param[in]  earth_rate_ecef
     *  @param[in]  sqrtQ
     *  @param[out] specificenergy
     */
    void shortupdate(int32_t            const  dt_ns,
                     lin::Vector3d      const &earth_rate_ecef,
                     lin::Vector3d      const &r_ecef,
                     lin::Vector3d      const &v_ecef,
                     lin::Matrixd<6, 6> const &sqrtQ,
                     lin::Matrixd<6, 6> const &sqrtR,
                     double                   &specificenergy
                     ) {
        // Prediction step
        {
            double _;
            shortupdate(dt_ns, earth_rate_ecef, sqrtQ, _);
        }

        // Update step
        {
            /* This again leverages the square root formulation of the EKF. The
             * update step is given by:
             * 
             *   qr([      sqrt(R)             0    ]) = _ [ transpose(C)      D     ]
             *     ([ S_k+1|k transpose(H)  S_k+1|k ])     [      0        S_k+1|k+1 ]
             */
            lin::Matrixd<12, 12> A, B, _;
            lin::ref<lin::Matrixd<6, 6>>(A, 0, 0) = sqrtR;
            lin::ref<lin::Matrixd<6, 6>>(A, 0, 6) = lin::zeros<lin::Matrixd<6, 6>>();
            lin::ref<lin::Matrixd<6, 6>>(A, 6, 0) = _sqrtP;  // H = I
            lin::ref<lin::Matrixd<6, 6>>(A, 6, 6) = _sqrtP;
            lin::qr(A, _, B);

            lin::Matrixd<6, 6> C, D;
            C = lin::transpose(lin::ref<lin::Matrixd<6, 6>>(B, 0, 0));
            D = lin::ref<lin::Matrixd<6, 6>>(B, 0, 6);

            /* Kalman gain calculation from the square root formulation:
             *
             *   K_k+1 = D inverse(C)
             */
            lin::Matrixd<6, 6> K;
            {
                lin::Matrixd<6, 6> Q, R;
                lin::qr(lin::transpose(C).eval(), Q, R);
                lin::backward_sub(R, K, (lin::transpose(Q) * lin::transpose(D)).eval());

                K = lin::transpose(K).eval();
            }

            // Measurement
            lin::Vectord<6> z;
            lin::ref<lin::Vector3d>(z, 0, 0) = r_ecef;
            lin::ref<lin::Vector3d>(z, 3, 0) = v_ecef;

            lin::Vectord<6> x;
            auto r = lin::ref<lin::Vector3d>(x, 0, 0);
            auto v = lin::ref<lin::Vector3d>(x, 3, 0);

            r = _orbit.recef();
            v = _orbit.vecef();
            x = x + K * (z - x).eval();

            _orbit = Orbit(_orbit.nsgpstime(), r, v);
            _sqrtP = lin::ref<lin::Matrixd<6, 6>>(B, 6, 6);

            _check_validity();
        }
    }
};
} // namespace orb
