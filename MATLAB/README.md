
# Overview

This directory will contain all MATLAB code housed in this repository.

The MATLAB portion of the simulation should house a sequence of scripts that
operate on variables stored in the global namespace. For example, we could have
`truth.r` store the truth position of the satellite in ECI and `truth.v` store
it's truth velocity in ECI. `truth.rwa_axl` can store reaction wheel angular
acceleration, `truth.rwa_w` can store the reaction wheels angular speeds, et
cetera.

If we have a sequence of scripts that can sim a single satellite, we can then
run a second inpedendant sequence of scripts via python (see the `/python`
folder) in parallel to sim a full mission. Yes, we would need to pass a little
bit of information between the MATLAB instances to perform rendezvous
calculations, but this can easily be done through python and given only a small
amount of data will be passed, the overhead will be tiny.

Also, once a full simulation is complete, we will need to add scripts that run
conditionally depending on whether or not the satellite is the leader, follower,
et cetera. We will be able to slowly replace MATLAB code with CXX code to test
the GNC code we will be flying with beforehand. This includes feedback control,
rendezvous algorith, state filter, et ceter. See the `/src` and `/include`
repositories.


# Directory Structure

 * `./*` - scripts to be called for/during the full simulation.
 * `./utl/*` - utility functions shared across all other MATLAB scripts in
   this repository.
 * `./environmental_models/*` - environmental functions shared across 
   all other MATLAB scripts in this repository.
 * `./environmental_models/helper_functions/*` - helper functions for environmental functions.
 * `./test/*` - standalone scripts that only depend on the utility functions
   mentioned above that demonstrate small bits of the simulation.


# Global Variables

 * truth
   * time, time since inital GPS week.
   * position_eci, Position of the center of mass of the satellite (m)
   * velocity_eci, Velocity of the center of mass of the satellite (m/s)
   * angular_rate_body, Angular rate of the spacecraft in the body frame (rad/s)
   * quat_body_eci, Quaternion that rotates from eci to body frame.
   * wheel_rate_body, x,y, and z, wheel angular rates (rad/s)
   * fuel_net_angular_momentum_eci, Net angular momentum of the fuel (N*m*s)
   * fuel_mass, The mass of the fuel (kg)
   * mission_time(positive int64), Mission time (ns)
 * actuators
   * firing_start_times, Times since inital GPS week to start firing (s)
   * real_thrust_vectors_body, Real thruster forces (N)
   * centers_of_thrust_body, Center of thrust for each firing (m)
   * firing_on_times, How long firings last (s)
   * wheel_commanded_rate, Commanded x,y,z wheel rate (rad/s)
   * wheel_commanded_ramp, Commanded x,y,z wheel ramp (rad/s/s)
   * magrod_real_moment_body, Real magnetorquer moment (A*m^2)
 * const
   * mu(positive scalar), Earth's gravitational constant (m^3/s^2)
   * dt(positive int64), Simulation timestep (ns)
   * INITGPS_WN(positive int), Initial gps week number, epoch for time (weeks)
   * R_EARTH(positive scalar), Equatorial Radius of Earth (m)
   * e_earth(positive scalar), Earth's eccentricity
   * tp_earth(scalar), Time when earth was at perihelion (s)
   * period_earth(positive scalar), Earth orbital period (s)
   * quat_eci_perifocal(quaternion), Quat between earth's perifocal and eci frame.
   * PRECESSION_RATE(3 vector), earth's axis precession rate (rad/s)
   * quat_ecef0_eci(quaternion), ecef0 is ecef frame at time 0.
   * earth_rate_ecef(3 vector), earth's inertial rotation rate in ecef frame (rad/s)
   * MAXWHEELRATE(positive scalar),  Max wheel rate (rad/s)
   * MAXWHEELRAMP(positive scalar), Max wheel ramp (rad/s/s)
   * MAXMOMENT(positive scalar), Max magrod moment on one axis (A*m^2)
   * MASS(positive scalar), Dry mass of satellite (kg)
   * JB(3x3 symmetric matrix), Dry moment of inertia of satellite in body frame (kg*m^2)
   * JBINV(3x3 symmetric matrix), Inverse of dry moment of inertia of satellite in body frame (1/(kg*m^2))
   * JWHEEL(positive scalar),  Wheel Inertia (kg*m^2)
   * JFUEL_NORM(positive scalar), Moment of inertia of the fuel/mass of the fuel (m^2)
   * SLOSH_DAMPING(positive scalar), Torque on fuel/difference in angular rates in eci (Nm/(rad/s))
   * ATTITUDE_PD_KP(scalar), Attitude PD controller K_p (N*m)
   * ATTITUDE_PD_KD(scalar), Attitude PD controller K_d (N*m/(rad/s))
   * SUNSENSOR_DEADZONE(positive scalar), Angle from +z axis where the sun sensors don't work (rad)




# Add-Ons

There are a few required add-ons

 * Mapping Toolbox
 * Ephemeris Data for Aerospace Toolbox
 * Aerospace Toolbox
 * Robotics System Toolbox
