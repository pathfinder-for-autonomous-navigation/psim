
# Configuration Files

## Constants

Values are assigned globally (i.e. values are specified directly in the top level dictionary). The following fields can be specified:

## Sensors

Values are asigned to for satellite individually (i.e. each `truth.json` should have `shared`, `leader`, and `follower` entries in the top level dictionary). The following fields can then be specified:

* `cdgps_model` String specifying the model used to simulate the CDGPS sensor.

* `cdgps_time_till_lock` Time until CDGPS lock will be acquired in seconds assuming conditions for a lock remain.
    * `value` : scalar

* `cdgps_position_bias` Initial position bias of the CDGPS sensor in units of meters.
    * `frame` : supported frames is/are `ecef`
    * `value` : three dimensional column vector

* `gps_model` String specifying the model used to simulate the GPS sensor.

* `cdgps_time_till_lock` Time until GPS lock will be acquired in seconds assuming conditions for a lock remain.
    * `value` : scalar

* `gps_position_bias` Initial position bias of the GPS sensor in units of meters.
    * `frame` : supported frames is/are `ecef`
    * `value` : three dimensional column vector

* `gps_velocity_bias` Initial velocity bias of the GPS sensor in units of meters per second.
    * `frame` : supported frames is/are `ecef`
    * `value` : three dimensional column vector

* `gyroscope_model` String specifying the model used to simulate the gyroscope.

* `gyroscope_bias` Initial gyroscope bias in the body frame of the spacecraft in units of radians per second.
    * `value` : three dimensional column vector

* `magnetometer_model` String specifying the model used to simulate the magnetometer.

* `magnetometer_bias` Initial magnetometer bias in the body frame of the spacecraft in units of Tesla.
    * `value` : three dimensional column vector

* `sun_sensor_model` String specifying the model used to simulate the sun sensors.

* `sun_sensor_truth_normals` Truth normals of the individual sun sensors in the body frame of the spacecraft. These normals are used for voltage measurement generation.
    * `value` : twenty by three matrix

* `sun_sensor_measured_normals` Measured normals of the individual sun sensors in the body frame of the spacecraft. These normals are used for the sun vector determination calculation.
    * `value` : twenty by three matrix

* `sun_sensor_truth_voltage_maximums` Truth voltage maximums attainable by each sun sensor. These scalars are used for voltage measurement generation.
    * `value` : twenty dimensional column vector

* `sun_sensor_measured_voltage_maximums` Measured voltage maximums attainable by each sun sensor. These scalars are used for the sun vector determination calculation.
    * `value` : twenty dimensional column vector

* `sun_sensor_alpha` Scale factor used in the sun vector determination algorithm to fight uncertainty in the sun's intensity (generally just set this to `1.0`).
    * `value` : scalar

## Truth

Values are asigned to for satellite individually (i.e. each `truth.json`) should have `shared`, `leader`, and `follower` top level dictionaries. The following fields can then be specified:

* `time` Initial time relative to the PAN epoch.
    * `units` : supported units is/are `nanoseconds`
    * `value` : integer scalar

* `position` Initial position in units of meters.
    * `frame` : supported frames is/are `eci`
    * `value` : three dimensional column vector

* `velocity` Initial velocity in units of meters per second.
    * `frame` : supported frames is/are `eci`
    * `value` : three dimensional column vector

* `angular_rate` Initial angular rate in units of radians per second.
    * `frame` : supported frames is/are `body`
    * `value` : three dimensional column vector

* `quaternion_body` Initial attitude of the spacecraft (i.e. orientation of the body frame).
    * `frame` : supported frames is/are `eci`
    * `value` : four dimensional column vector representing a quaternion

* `wheel_rate` Initial angular rate of the reaction wheels in units of radians per second.
    * `frame` : supported frames is/are `body`
    * `value` : three dimensional column vector

* `fuel_angular_momentum` Initial, net angular momentum for the fuel in the spacecraft in units of kilogram meters squared per second.
    * `frame` : supported frames is/are `eci`
    * `value` : three dimensional column vector

* `fuel_mass` Initial fuel mass stored on the spacecraft in units of kilograms.
    * `value` : scalar
