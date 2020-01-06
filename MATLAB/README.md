# Pathfinder for Autonomous Navigation MATLAB Simulation README
Started by Kyle Krol on Sep 4, 2019

**Authors** Nathan Zimmerberg, Kyle Krol

Latest Revision: Dec 30, 2019

Pathfinder for Autonomous Navigation

Space Systems Design Studio

Cornell University


## Overview

The main goal is to develop and test all Guidance Navigation and Control(GNC) algorithms needed to rendezvous and dock two CubeSats in low earth orbit. One satellite is the `leader` and just passively orbits. The other satellite is the `follower` and uses cold gas thrusters to rendezvous with the `leader`. Each satellite tries to point its docking face at the other satellite and point its antennas away from earth.

This doc is stored in the MATLAB directory of the psim repository which contains all MATLAB code in psim.

The MATLAB portion of psim has two major components.

1. A simulator to model the physics of Pathfinder for Autonomous Navigation(PAN).
1. A matlab prototype version of GNC related flight software.


The simulator has four main functions that preform computations with the `main_state`.
 * `initialize_main_state`: Constructs the main state given a seed, and situation.
 * `sensor_reading`: Outputs the sensor readings of one satellite.
 * `main_state_update`: Updates the main state over one delta time step (`const.dt` ns).
 * `actuator_command`: Modifies the state of one satellite, given its actuator commands.

The matlab prototype of gnc flight software has two main functions.
 * `initialize_computer_states`: Constructs both leader and follower computer states.
 * `update_FC_state`: Outputs actuator commands and updates one satellite's computer state given sensor readings.
These functions, and functions they call will have to be translated into c++.

There are two main scripts, `main.m` and `run_tests.m`. `run_tests.m` runs all the test scripts.
`main.m` runs a simulation and saves the `main_state_trajectory` as well as computer state trajectories.

## Python Integration

Python uses the [Matlab engine](https://www.mathworks.com/help/matlab/matlab_external/call-matlab-functions-from-python.html)
to call simulation functions from MATLAB, primarily `config`, `generate_mex_code`, `initialize_main_state`, `sensor_reading`, `main_state_update`, and `actuator_command`.
Global variables in the workspace don't have to be used to transfer data.

Most Matlab types are compatible with python types: [Python to Matlab function inputs](https://www.mathworks.com/help/matlab/matlab_external/pass-data-to-matlab-from-python.html)
[Matlab function outputs to Python](https://www.mathworks.com/help/matlab/matlab_external/handle-data-returned-from-matlab-to-python.html)

## Directory Structure

 * `./*` - scripts to be called for/during the full simulation.
 * `./utl/*` - utility functions shared across all other MATLAB scripts in
   this repository.
 * `./environmental_models/*` - environmental functions shared across
   all other MATLAB scripts in this repository.
 * `./environmental_models/helper_functions/*` - helper functions for environmental functions.
 * `./test/*` - standalone scripts that test almost every function in the simulation.
 * `./plot/*` - functions to create plots of the `main_state_trajectory`
 * `./adcs/*` - attitude determination and control functions.
 * `./estimator_prototypes/*` - prototypes for estimators.

## Main State Data Structure

`main_state` contains the state of both `leader` and `follower` satellites as members.
Each satellites state has the following members and submembers:
* `dynamics`
   * `time`, Time since initial GPS week (s)
   * `time_ns`(positive int64), Time since initial GPS week (ns)
   * `position_eci`, Position of the center of mass of the satellite (m)
   * `velocity_eci`, Velocity of the center of mass of the satellite (m/s)
   * `angular_rate_body`, Angular rate of the spacecraft in the body frame (rad/s)
   * `quat_body_eci`, Quaternion that rotates from eci to body frame.
   * `wheel_rate_body`, x,y, and z, wheel angular rates (rad/s)
   * `fuel_net_angular_momentum_eci`, Net angular momentum of the fuel (Nms)
   * `fuel_mass`, The mass of the fuel (kg)
 * `actuators`
   * `firing_start_times`(4x1 matrix): Times since initial GPS week to start firing each thruster (s)
   * `thrust_vectors_body`(3x4 matrix): each thruster's force vector (N)
   * `centers_of_thrust_body`(3x4 matrix): Center of thrust of each truster (m)
   * `firing_on_times`(4x1 matrix): How long each thruster firing lasts (s)
   * `wheel_commanded_rate`(3x1 matrix): Commanded x,y,z wheel rate (rad/s)
   * `wheel_commanded_ramp`(3x1 matrix): Commanded x,y,z wheel ramp (rad/s/s)
   * `magrod_real_moment_body`(3x1 matrix): Real magnetorquer moment (Am^2)
   * `magrod_hysteresis_body`(3x1 matrix): Real magnetorquer hysteresis moment (Am^2)
 * `sensors`
   * `gyro_bias`, (rad/s)
   * `magnetometer_bias`, (T)
   * `sunsensor_real_normals`, (unitless)
   * `sunsensor_real_voltage_maximums`, normalization constants for voltage measurements (V)
   * `sunsensor_measured_normals`, (unitless)
   * `sunsensor_measured_voltage_maximums`, (V)
   * `gps_bias`,  (m and m/s)
   * `gps_time_till_lock`,
       time till the GPS gets a lock, starts at `const.GPS_LOCK_TIME`, then counts down and stays at 0
       when the antenna is pointing towards the GPS constellation (s)
   * `cdgps_bias`, (m and m/s)
   * `cdgps_time_till_lock`,
        time till the carrier-phase differential GPS(CDGPS) gets a lock, starts at `const.CDGPS_LOCK_TIME`, then counts down and stays at 0
        when the gps antenna is pointing towards the GPS constellation the piksi antenna is pointing at the other sattelite (s)

## get_truth
In addition to this core state, other info about the satellite is retrieved using
the function get_truth which takes a string and the dynamics of a satellite as input.

For example,
`get_truth('magnetic field body',main_state.follower.dynamics)`
returns the real magnetic field in the body frame.

`get_truth('magnetic field eci',main_state.follower.dynamics)`
returns the real magnetic field in the eci frame.

`get_truth('quat body eci',main_state.follower.dynamics)`
returns the quaternion that rotates vectors from eci to body

`get_truth('dcm body eci',main_state.follower.dynamics)`
returns the Direction Cosine Matrix(DCM) that rotates vectors from eci to body

`get_truth('rate body eci body',main_state.follower.dynamics)`
returns the angular rate of the transform from eci to body in the body frame.

Call `get_truth('help')` for complete documentation about supported values

## Sensor Readings and Actuator Commands

sensor readings is a struct with elements:
 * `gyro_body`, gyro reading (rad/s)
 * `magnetometer_body`, magnetometer reading (T)
 * `sat2sun_body`, unit vector from satellite to sun (unitless)
 * `sun_sensor_true`, true if sun vector reading is good, else false.
 * `wheel_momentum_body`, wheel angular momentum reading (Nms)
 * `time`, time since initial GPS week (s)
 * `position_ecef`, position of the gps reciever of the satellite (m)
 * `velocity_ecef`, velocity of the gps reciever of the satellite (m/s)
 * `target_position_ecef`, position of the target gps reciever of the satellite, from ground (m)
 * `target_velocity_ecef`, velocity of the target gps reciever of the satellite, from ground (m/s)
 * `relative_position_ecef`, position vector from self to target, from cdgps (m)

actuator commands is a struct with elements:
 * `firing_start_times`(4x1 matrix): commanded times since initial GPS week to start firing (s)
 * `firing_on_times`(4x1 matrix): commanded thruster on times (s)
 * `wheel_torque`, commanded x,y,z wheel torque, (signed ramp)x(rotor inertia) (Nm)
 * `wheel_enable`, commanded x,y,z wheel enables, whether each wheel
           should be on, if false, the wheel rate is commanded to zero.
 * `magrod_moment`, commanded x,y,z magnetorquer moments (Am^2)

## Functions

`config()`: sets up path and constants
`generate_mex_code()`: generates mex code from C++ wrapper function
`config()`: sets up path and constants
`get_truth(name,dynamics)`: returns named value from dynamics

The simulator has four main functions that preform computations with `main_state`.
 * `initialize_main_state`: Constructs the main state given a seed, and situation.
 * `sensor_reading`: Outputs the sensor readings of one satellite.
 * `main_state_update`: Updates the main state over one delta time step.
 * `actuator_command`: Modifies the state of one satellite, given its actuator commands.

The matlab prototype of gnc flight software has two main functions.
 * `initialize_computer_states`: Constructs both leader and follower computer states.
 * `update_FC_state`: Outputs actuator commands and updates one satellite's computer state given sensor readings.


main_state_update has a few main helper funtions.
 * `dynamics=dynamics_update(dynamics,actuators)`
        Update the orbital and attitude dynamics, and time using numerical integration.
 * `sensors=sensors_update(sensors,dynamics)`
        Update the sensor state given `dynamics`.

update_FC_state is also broken in to a few main helper functions.
 * `[orbit_controller_state, delta_v, delta_time]=get_next_maneuver(orbit_controller_state, current_orbit,target_orbit,current_time)`
        Calculate the next maneuver to rendevous with target_orbit.
 * `[orbit,jacobian] = orbit_propagator(orbit,current_time,delta_time)`
        Propagate orbit forward delta_time and calculate the jacobian of the translation.
 * `[orbit_estimator_state, both_orbits, time]=estimate_orbits(orbit_estimator_state,my_gps_readings,other_gps_readings,cdgps_readings)`
        Update estimates of my orbit and other orbit.
 * `[adcs_state,magrod_moment,wheel_torque,wheel_enable]=adcs_update(adcs_state,time,orbit,target_orbit,gyro,magnetometer,sat2sun)`
        Attitude Determination and Control System (ADCS) update.

Environmental functions.
 * `density= env_atmosphere_density(time,x)`
 * `[quat_ecef_eci,rate_ecef]= env_earth_attitude(time)`
 * `eclipse = env_eclipse(earth2sat,sat2sun)`
 * `[acceleration,potential,hessian]= env_gravity(time,x)`
 * `B= env_magnetic_field(time,x)`
 * `S= env_sun_vector(time)`

 Quaternion functions.
 * `u = utl_rotateframe(q,v)`
 * `qout = utl_quat_conj(q)`
 * `q = utl_dcm2quat(a)`
 * `qout = utl_quat_cross_mult(q1,q2)`
 * `q_triad = utl_triad(N_sun, N_mag, B_sun, B_mag)`

Testing utility functions:
 * `equal= utl_compare_main_states(main_state1,main_state2)`
 * `equal= utl_compare_dynamics(dynamics1,dynamics2)`
 * `equal= utl_compare_actuators(actuators1,actuators2)`
 * `equal= utl_compare_sensors(sensors1,sensors2)`
        returns true if sensors1 and sensors2 are close enough.
 * `orbit = true_orbit_propagator(orbit,current_time,delta_time)`
         returns the updated orbit, using drag, j8 and maybe other perturbations.

Plotting and visualization functions:
 * `plot_get_truth(main_state_trajectory,name,sat_name)`
        Create a plot over time of `name` truth value from `sat_name`.
        `name` must be an acceptable input to `get_truth()`,
        `sat_name` should be either `'follower'` or `'leader'`
        For example:

        `plot_get_truth(main_state_trajectory,'wheel rate leader (rad/s) body','leader')`

        plots the `'wheel rate'` of the `leader` and its norm in the `body` frame
 * `plot_almost_conserved_values(main_state_trajectory)`
        creates plots of angular momentums in eci and total energies,
        if these have a huge jump, there is a problem with physics.
 * `plot_pointing_errors(main_state_trajectory)`
        plots magnitude of error of where the sats are pointing vs where they should be pointing.
 * `plot_power(main_state_trajectory)`
        plots power stuff.
 * `plot_orbit_error(main_state_trajectory)`
        plots differences in leader and follower orbits.
 * `fancy_animation(main_state_trajectory)`
        creates a fancy animation from the simulation data.

## Controllers
 <img src="https://docs.google.com/drawings/d/e/2PACX-1vQ36cMMJu3pSCEW4oTc9ZblkLZlGmEKQNGi2ywjk4QizGxEGnlWA3RTp1Hhh_5vhKp9Q6UxJgSJFVQZ/pub?w=846&amp;h=547">

 <img src="https://docs.google.com/drawings/d/e/2PACX-1vQjQfsUQSmmeXKQT5tWik40ip17f55RgKxIxE-MhlL6FJqcM33Bdasc_leOyrpsiqIZiHovv2fvI1kh/pub?w=900&amp;h=694">

## Constants
Constants are stored in the `const` global struct.
`const` is initialized by `config()`.
 * const
   * `mu`(positive scalar), Earth's gravitational constant (m^3/s^2)
   * `dt`(positive int64), Simulation timestep (ns)
   * `INITGPS_WN`(positive int), Initial gps week number, epoch for time (weeks)
   * `R_EARTH`(positive scalar), Equatorial Radius of Earth (m)
   * `e_earth`(positive scalar), Earth's eccentricity
   * `tp_earth`(scalar), Time when earth was at perihelion (s)
   * `period_earth`(positive scalar), Earth orbital period (s)
   * `quat_eci_perifocal`(quaternion), Quat between earth's perifocal and eci frame.
   * `PRECESSION_RATE`(3 vector), earth's axis precession rate (rad/s)
   * `quat_ecef0_eci`(quaternion), ecef0 is ecef frame at time 0.
   * `earth_rate_ecef`(3 vector), earth's inertial rotation rate in ecef frame (rad/s)
   * `MAXWHEELRATE`(positive scalar),  Max wheel rate (rad/s)
   * `MAXWHEELRAMP`(positive scalar), Max wheel ramp (rad/s/s)
   * `MAXMOMENT`(positive scalar), Max magrod moment on one axis (Am^2)
   * `MASS`(positive scalar), Dry mass of satellite (kg)
   * `JB`(3x3 symmetric matrix), Dry moment of inertia of satellite in body frame (kgm^2)
   * `JBINV`(3x3 symmetric matrix), Inverse of dry moment of inertia of satellite in body frame (1/(kgm^2))
   * `JWHEEL`(positive scalar),  Wheel Inertia (kgm^2)
   * `JFUEL_NORM`(positive scalar), Moment of inertia of the fuel/mass of the fuel (m^2)
   * `SLOSH_DAMPING`(positive scalar), Torque on fuel/difference in angular rates in eci (Nm/(rad/s))
   * `ATTITUDE_PD_KP`(scalar), Attitude PD controller K_p (Nm)
   * `ATTITUDE_PD_KD`(scalar), Attitude PD controller K_d (Nm/(rad/s))
   * `detumble_safety_factor`(scalar range (0,1)):
The fraction of max wheel momentum detumbling ends at.
   * `GPS_LOCK_TIME`(positive scalar), Time it takes the GPS to get a lock (s)
   * `CDGPS_LOCK_TIME`(positive scalar), Time it takes the CDGPS to get a lock (s)
   * `magnetometer_bias_readings_min`(positive int): number of readings per axis to get a 
good magnetometer bias estimate.
   * `magnetometer_noise_sdiv`(positive scalar): 
standard diviation of the magnetometer noise (T)
   * `magnetometer_bias_sdiv`(positive scalar): 
standard diviation of the magnetometer bias (T)
   * `gyro_noise_sdiv`(positive scalar):
standard diviation of the gyro noise (rad/s)
   * `gyro_bias_sdiv`(positive scalar):
standard diviation of the gyro bias (rad/s)


## Functions to be implemented in C++
update_FC_state and any function it uses including:
    `get_next_maneuver`, `orbit_propagator`, `estimate_orbits`, `adcs_update`,
    `env_earth_attitude`, `env_eclipse`, `env_magnetic_field`, `env_sun_vector`, `utl_rotateframe`, `utl_quat_conj`,
    `utl_dcm2quat`, `utl_quat_cross_mult`, `utl_triad`.

The test scripts for these functions should also be translated into C++.
See the `/src` and `/include`
repositories.


## Test Harness
Each function will have its own test script in `./test/` called `test_functionName.m` containing:
    1. Setup of path, constants, and seed.
    2. Calls to the function.
    3. Asserts that the outputs are within required accuracy.
`run_tests.m` is a script that will run all of the test scripts and give a failure summary for each test that failed.

Tests should be completely deterministic, fast, and not have any prints or plots unless they fail.


## Team Member Responsibilities

Each team member is the principle programmer
for some functions in the system. If you are going to start work on a function,
create an issue with the same name as the function and assign yourself to it.
In addition to the function also write a test script that at a minimum just calls the function and add that script to `run_tests.m`
If you need new constants just add them to `config.m` and add the name, description, and units to the readme.


## Current Status

One the Matlab simulation side, next getting a complete version of `sensor_reading`, `actuator_command`, `initialize_main_state`, `sensors_update`, and `adcs_update` is the most important task. On the output and visualization side, `get_truth` is the most important task. On the C++ side, a working version of Kyle's vector library, and quaternion functions are next. On the rendezvous side, `true_orbit_propagator`, `orbit_propagator`, and `get_next_maneuver` are the most important.

| Function                     | Person  | Priority | Basic Matlab Version | Test Script | C++ Version |
|------------------------------|---------|----------|----------------------|-------------|-------------|
| get_truth                    | Nathan  |          | not started          | not started | NA          |
| initialize_main_state        | Nathan  |          | wip                  | not started | NA          |
| sensor_reading               | Kyle    |          | wip                  | not started | NA          |
| main_state_update            | Nathan  |          | wip                  | not started | NA          |
| actuator_command             | Josh    |          | wip                  | not started | NA          |
| initialize_computer_states   |         |          | wip                  | not started | NA          |
| update_FC_state              |         |          | wip                  | not started | NA          |
| dynamics_update              | Nathan  |          | wip                  | not started | NA          |
| sensors_update               |         |          | not started          | not started | NA          |
| get_next_maneuver            |         |          | not started          | not started | not started |
| orbit_propagator             | Kyle    |          | wip                  | wip         | wip         |
| estimate_orbits              |         |          | not started          | not started | not started |
| adcs_update                  | Nathan  |          | wip                  | not started | not started |
| env_atmosphere_density       | Sruti   |          | not started          | not started | NA          |
| env_earth_attitude           | Nathan  |          | done                 | done        | wip         |
| env_eclipse                  | Nathan  |          | done                 | not started | not started |
| env_gravity                  |         |          | wip                  | not started | NA          |
| env_magnetic_field           | Nathan  |          | wip                  | done        | wip         |
| env_sun_vector               | Nathan  |          | done                 | done        | wip         |
| utl_rotateframe              | Kyle    |          | done                 | done        | done        |
| utl_quat_conj                | Kyle    |          | done                 | done        | done        |
| utl_dcm2quat                 | Kyle    |          | done                 | done        | done        |
| utl_quat_cross_mult          | Kyle    |          | done                 | done        | done        |
| utl_triad                    | Kyle    |          | done                 | done        | done        |
| utl_vect_rot2quat            | Kyle    |          | done                 | done        | done        |
| utl_compare_main_states      |         |          | not started          | not started | NA          |
| utl_compare_dynamics         |         |          | not started          | not started | NA          |
| utl_compare_actuators        |         |          | not started          | not started | NA          |
| utl_compare_sensors          |         |          | not started          | not started | NA          |
| true_orbit_propagator        | Sruti   |          | not started          | not started | NA          |
| plot_almost_conserved_values |         |          | not started          | NA          | NA          |
| plot_pointing_errors         |         |          | not started          | NA          | NA          |
| plot_wheel_rates             |         |          | not started          | NA          | NA          |
| plot_orbit_error             |         |          | not started          | NA          | NA          |
| fancy_animation              |         |          | not started          | NA          | NA          |





## Matlab Add-Ons

There are a few required Add-Ons for Matlab

 * Mapping Toolbox
 * Ephemeris Data for Aerospace Toolbox
 * Aerospace Toolbox
 * Robotics System Toolbox
