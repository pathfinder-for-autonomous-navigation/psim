// Stub file to compile with PlatformIO.

// Add includes to ensure they don't break compilation
#include <gnc_attitude_estimation.hpp>
#include <gnc_constants.hpp>
#include <gnc_containers.hpp>
#include <gnc_environment.hpp>
#include <gnc_ode.hpp>
#include <gnc_utilities.hpp>
#include <lin.hpp>

#if !defined(UNIT_TEST) && !defined(FUNCTIONAL_TEST) && !defined(FLIGHT)
#if defined(TEST_DESKTOP) || defined(DESKTOP)
    int main() {
        return 0;
    }
#else
    #include <Arduino.h>

    void setup() {}
    void loop()  {}
#endif
#endif
