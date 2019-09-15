
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

# Add-Ons

There are a few required add-ons

 * Mapping Toolbox
 * Ephemeris Data for Aerospace Toolbox
 * Aerospace Toolbox
 * Robotics System Toolbox
