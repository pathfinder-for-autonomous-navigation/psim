# USB Console Configuration

Here you can find some template configurations. I recommend copy-pasting a template to somewhere else on your disk and updating it with real values. We don't want to upload actual configs to the repository since they contain sensitive data.

## Items
- `seed` : Randomization seed for simulation.
- `sim_duration` : Number of seconds to run the simulation. If this is set to zero, the simulation is never started.
- `devices`: A list of device configuration objects, each with the following data:
  - `name` of device.
  - `run_mode`: Either set to `teensy` or `native`, depending on whether the device is connected via a Teensy or if the device should be emulated with a desktop binary.
  - `binary_filepath`: If the `run_mode` was specified as `native`, this filepath describes where to find the desktop binary. The filepath must be absolute.
- `radios`: A list of radio configuration objects, each with the following data:
  - `name` of device that the radio is connected to. It should correspond to one of the devices above.
  - `imei` IMEI # of the radio. This is used to tell Iridium which radio we're trying to collect/send data from/to.
  - `connect` If false, no attempt is made to establish the radio connection. This option exists so that the simulation can be run separately from radio testing.

All of the above fields are required, but the `devices` and `radios` list may potentially be empty.