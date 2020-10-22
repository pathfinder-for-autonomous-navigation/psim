//
// MIT License
//
// Copyright (c) 2020 Pathfinder for Autonomous Navigation (PAN)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

/** @file psim/truth/orbit.cpp
 *  @author Kyle Krol
 */

#include <psim/truth/attitude_orbit.hpp>

namespace psim {
 
   AttitudeOrbitNoFuelGnc::AttitudeOrbitNoFuelGnc(Configuration const &config,
   std::string const &prefix, std::string const &satellite)
 : Super(config, prefix, satellite, "eci") { }
 
   };
   /*  step function:   
 
       mass = "{prefix}.{satellite}.J"
       I = "{prefix}.{satellite}.J"
       I_w = "{prefix}.{satellite}.wheels.J"
       omega = "{prefix}.{satellite}.attitude.w"
       tau_w = "{prefix}.{satellite}.wheels.t"
       omega_w = "{prefix}.{satellite}.wheels.w"
       m = "{prefix}.{satellite}.magnetorquers.m"
       b = gnc::env::earth_attitude(t, q_ecef_eci)
 
       w_dot = inverse(I) * (m.cross(b) + tau_w - omega.cross(I*omega + I_w*omega_w))

       return w_dot + orbit.step()
   */


/* y = [x; y; z; xdot; ydot; zdot;q1;q2;q3;q4;ratex;ratey;ratez;
     1  2  3  4     5     6    7  8  9  10 11    12    13    
    
     fuel_ang_momentx;fuel_ang_momenty;fuel_ang_momentz;]
     14               15               16

    Don't have any super expensive calculation in here.
    Instead make first or zero order approximations of
    perturbations before, and pass in as a function of time.
*/
  vector<double> AttitudeOrbitNoFuelGnc::state_dot(t,y,g){
    vector<double> dydt(16, 0);

    std::copy(y.begin() + 3, y.begin() + 5, dydt.begin()); // dydt(1:3)=y(4:6)

    // dydt(4:6) = g_eci + F_envdrag_eci./(const.MASS);
    // TODO: Do we include drag? Not in yml
    std::copy(g.begin(), g.end(), dydt.begin() + 3);

    // dydt(7:10) = utl_quat_cross_mult(0.5*quat_rate,quat_body_eci);
    // quat_body_eci=y(7:10);
    vector<double> quat_body_eci(4);
    std::copy(y.begin()+6, y.end()+9, quat_body.begin());
    // quat_rate=[y(11:13);0];
    vector<double> quat_rate(4);
    [y(11:13);0];
    qbqr_mult = utl_quat_cross_mult(0.5*quat_rate,quat_body_eci);
    std::copy(qbqr_mult.begin(), qbqr_mult.end(), dydt.begin() + 6);
    
    // dydt(14:16) = -torque_from_fuel_eci;

    // dydt(11:13) = const.JBINV*( Lb-Lwb-cross(y(11:13),const.JB*y(11:13)+const.JWHEEL*wheelrate(t)));

    return dydt;
  }

  void AttitudeOrbitNoFuelGnc::step(){
      this->Super::step();

       // Initialize variables from .yml
       auto &mass = prefix_satellite_m.get();
       auto &I = prefix_satellite_J.get();
       auto &I_w = prefix_satellite_wheels_J.get();
       auto &omega = prefix_satellite_attitude_w.get();
       auto &tau_w = prefix_satellite_wheels_t.get();
       auto &omega_w = prefix_satellite_wheels_w.get();
       auto &m = prefix_satellite_magnetorquers_m.get();
       auto &b = gnc::env::earth_attitude(t, q_ecef_eci);

       // References to the current time and timestep
       auto const &dt = prefix_dt_s->get();
       auto const &t = prefix_t_s->get();

       // References to position and velocity
       auto &r = prefix_satellite_orbit_r.get();
       auto &v = prefix_satellite_orbit_v.get();

       IntegratorData data {&mass, &I, &I_w, &omega, &tau_w, &omega_w, &m, &b};

       // Simulate our dynamics
        auto const xf = ode4(t, dt, {r(0), r(1), r(2), v(0), v(1), v(2)}, nullptr,
        [](Real t, lin::Vector<Real, 6> const &x, void *ptr) -> lin::Vector<Real, 6> { // include/gnc/ode4.hpp
            // References to our position and velocity in ECI
            auto const r = lin::ref<3, 1>(x, 0, 0);
            auto const v = lin::ref<3, 1>(x, 3, 0);
            auto *data = (IntegratorData *) ptr;

            // Angular acceleration
            w_dot = inverse(data->I) * ((data->&m)).cross(data->&b) + (data->&tau_w) - (data->&omega).cross((data->&I))*(data->&omega)) + (data->&I_w)*(data->&omega_w))));

            // Calculate the Earth's current attitude (Position in ECI -> gravitational acceleration vector in ECI)
            Vector4 q; 
            gnc::env::earth_attitude(t, q);  // q = q_ecef_eci

            // Determine our gravitation acceleration in ECI
            Vector3 g;
            {
                Vector3 r_ecef;
                gnc::utl::rotate_frame(q, r.eval(), r_ecef);

                Real _;
                gnc::env::gravity(r_ecef, g, _);  // g = g_ecef
                gnc::utl::quat_conj(q);           // q = q_eci_ecef
                gnc::utl::rotate_frame(q, g);     // g = g_eci
            }

            return {v(0), v(1), v(2), g(0), g(1), g(2)}; //F=ma
        }); 
  }
 
}  // namespace psim
