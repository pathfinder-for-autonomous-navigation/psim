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

/** @file psim/sensors/gps_no_attitude.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_SENSORS_GPS_NO_ATTITUDE_HPP_
#define PSIM_SENSORS_GPS_NO_ATTITUDE_HPP_

#include <psim/sensors/gps_no_attitude.yml.hpp>

namespace psim {

class GpsNoAttitude : public GpsNoAttitudeInterface<GpsNoAttitude> {
 private:
  typedef GpsNoAttitudeInterface<GpsNoAttitude> Super;

 public:
  using Super::GpsNoAttitudeInterface;

  GpsNoAttitude() = delete;
  virtual ~GpsNoAttitude() = default;

  Boolean sensors_satellite_gps_valid() const;
  Vector3 sensors_satellite_gps_r() const;
  Vector3 sensors_satellite_gps_r_error() const;
  Vector3 sensors_satellite_gps_v() const;
  Vector3 sensors_satellite_gps_v_error() const;
};
} // namespace psim

#endif
