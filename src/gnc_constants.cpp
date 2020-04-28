/** @file gnc_constants.cpp
 *  Autocoded constants source file. See tools/constants_generator.py for more
 *  information. */

#include <gnc/config.hpp>
#include <gnc/constants.hpp>

namespace gnc {
namespace constant {

GNC_TRACKED_CONSTANT(unsigned short, init_gps_week_number, 2045);

GNC_TRACKED_CONSTANT(unsigned int, init_gps_time_of_week, 0);

GNC_TRACKED_CONSTANT(unsigned long, init_gps_nanoseconds, 0);

GNC_TRACKED_CONSTANT(double, init_dec_year, 2.019205478881278e3);

GNC_TRACKED_CONSTANT(double, earth_rate_ecef_z, 1.0e-4 * 0.729211585530000);

GNC_TRACKED_CONSTANT(lin::Vector3d, earth_precession_rate, 1.0e-11 * 0.069896936522494, 1.0e-11 * -0.315732660584366, 0.0);

GNC_TRACKED_CONSTANT(lin::Vector4d, q_ecef0_eci, 0.000917219458782, 0.000038843277811, 0.998612074831165, 0.052660053181324);

GNC_TRACKED_CONSTANT(lin::Matrix3x3f, J_sat, 0.03798, 0.0, 0.0, 0.0, 0.03957, 0.0, 0.0, 0.0, 0.00688);

GNC_TRACKED_CONSTANT(float, pointer_Kp, 20.0e-4f);

GNC_TRACKED_CONSTANT(float, pointer_Kd, 22.5e-4f);

}  // namespace constant
}  // namespace gnc

