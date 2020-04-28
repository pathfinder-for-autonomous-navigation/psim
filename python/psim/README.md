
# PSim Python Module

This module is intended to consolodate how PAN team members interface with the MATLAB simulation. Given both PTest and GNC developers will be using this interface we have the following requirements:

 * Flexible, data driven, simulation configuration with support for monte carlo simulations.
 * Simple command line interface to start/attach to a running MATLAB instance, configure a simulation, and run it.
 * Python interface with similiar functionality that easily exposes simulation variables.
 * Modularity such that the flight computer update step can easily be stubbed out of the simulation in the future.
 * Ability to easily migrate GNC code between MATLAB, Python, and C++ without effecting code downstream
