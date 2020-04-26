#include <stdio.h>
#include <cstdint>
#include <limits>
#include <lin.hpp>
#include <gnc/constants.hpp>
#include <gnc/config.hpp>

//UTILITY MACROS
#include <unity.h>
#include "../custom_assertions.hpp"
#include <orb/Orbit.h>
#include <orb/GroundPropagator.h>

//0 or 1 that can't be calculated at compile time.
#ifdef DESKTOP
#define notcompiletime (rand()%2)
#else
#include <Arduino.h>
#define notcompiletime (millis()%2)
#endif

/** Check some of the internal invariants are true.*/
#define CHECKINVARIANT(est) do {\
            if (est.catching_up.valid()){ \
                TEST_ASSERT(est.current.valid())\
                TEST_ASSERT(est.current.numgravcallsleft()<est.catching_up.numgravcallsleft());\
            }\
            if (est.to_catch_up.valid()){ \
                TEST_ASSERT(est.catching_up.valid())\
                TEST_ASSERT(est.catching_up.numgravcallsleft()<est.to_catch_up.numgravcallsleft());\
            }\
           } while(0) 

//grace orbit initial
const orb::Orbit gracestart(uint64_t(gnc::constant::init_gps_week_number)*gnc::constant::NANOSECONDS_IN_WEEK,{-6522019.833240811L, 2067829.846415895L, 776905.9724453629L},{941.0211143841228L, 85.66662333729801L, 7552.870253470936L});

//grace orbit 100 seconds later
const orb::Orbit grace100s(uint64_t(gnc::constant::init_gps_week_number)*gnc::constant::NANOSECONDS_IN_WEEK+100'000'000'000ULL,{-6388456.55330517L, 2062929.296577276L, 1525892.564091281L},{1726.923087560988L, -185.5049475128178L, 7411.544615026139L});

//grace orbit 13700 seconds later
const orb::Orbit grace13700s(uint64_t(gnc::constant::init_gps_week_number)*gnc::constant::NANOSECONDS_IN_WEEK+13700'000'000'000ULL,{1579190.147083268L, -6066459.613667888L, 2785708.976728437L},{480.8261476296949L, -3083.977113497177L, -6969.470748202005L});

//earth rate in ecef (rad/s)
lin::Vector3d earth_rate_ecef= {0.000000707063506E-4,-0.000001060595259E-4,0.729211585530000E-4};

void test_basic_constructors() {
    orb::GroundPropagator est;
    orb::Orbit x= est.best_estimate();
    TEST_ASSERT_FALSE(x.valid());
    TEST_ASSERT_EQUAL_INT(est.total_num_grav_calls_left(),0);
}

/** Test propagating invalid orbits doesn't have any effect.*/
void test_propagating_invalid_orbits() {
    orb::GroundPropagator est;
    orb::Orbit x= est.best_estimate();
    TEST_ASSERT_FALSE(x.valid());
    TEST_ASSERT_EQUAL_INT(est.total_num_grav_calls_left(),0);
    auto est_copy= est;
    est.one_grav_call();
    CHECKINVARIANT(est);
    x= est.best_estimate();
    TEST_ASSERT_FALSE(x.valid());
    TEST_ASSERT_EQUAL_INT(est.total_num_grav_calls_left(),0);
    TEST_ASSERT_EQUAL_MEMORY (&est, &est_copy, sizeof(orb::GroundPropagator));
}

/** Test inputting invalid Orbits doesn't have an effect.*/
void test_input_invalid() {
    orb::GroundPropagator est;
    orb::GroundPropagator est_copy= est;
    est.input(orb::Orbit(),gracestart.nsgpstime(),earth_rate_ecef);
    TEST_ASSERT_EQUAL_MEMORY (&est, &est_copy, sizeof(orb::GroundPropagator));
    est.input(gracestart,gracestart.nsgpstime(),earth_rate_ecef);
    est_copy= est;
    est.input(orb::Orbit(),gracestart.nsgpstime(),earth_rate_ecef);
    TEST_ASSERT_EQUAL_MEMORY (&est, &est_copy, sizeof(orb::GroundPropagator));
}

/** Test inputting the first valid Orbit.*/
void test_1st_input_valid() {
    orb::GroundPropagator est;
    est.input(gracestart,gracestart.nsgpstime(),earth_rate_ecef);
    CHECKINVARIANT(est);
    TEST_ASSERT(est.current.valid());
    TEST_ASSERT(!est.catching_up.valid());
    TEST_ASSERT(!est.to_catch_up.valid());
    orb::Orbit x= est.best_estimate();
    TEST_ASSERT_TRUE(x.valid());
    TEST_ASSERT_FALSE(x.numgravcallsleft());
    TEST_ASSERT_FALSE(est.total_num_grav_calls_left());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x.vecef(), gracestart.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x.recef(), gracestart.recef());
}

/** Test inputting the 2 valid Orbits that are the same times.*/
void test_two_inputs_a() {
    orb::GroundPropagator est;
    est.input(gracestart,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(gracestart,gracestart.nsgpstime(),earth_rate_ecef);
    CHECKINVARIANT(est);
    TEST_ASSERT(est.current.valid());
    TEST_ASSERT(!est.catching_up.valid());
    TEST_ASSERT(!est.to_catch_up.valid());
    orb::Orbit x= est.best_estimate();
    TEST_ASSERT_TRUE(x.valid());
    TEST_ASSERT_FALSE(x.numgravcallsleft());
    TEST_ASSERT_FALSE(est.total_num_grav_calls_left());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x.vecef(), gracestart.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x.recef(), gracestart.recef());
}

/** Test inputting the 2 valid Orbits that are different times.*/
void test_two_inputs_b() {
    orb::GroundPropagator est;
    orb::Orbit x0= gracestart;
    orb::Orbit x1= gracestart;
    x1.applydeltav({1.0,0.0,0.0});
    est.input(x0,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x1,gracestart.nsgpstime()+100LL,earth_rate_ecef);
    CHECKINVARIANT(est);
    TEST_ASSERT(est.current.valid());
    TEST_ASSERT(!est.catching_up.valid());
    TEST_ASSERT(!est.to_catch_up.valid());
    orb::Orbit best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_TRUE(best.numgravcallsleft());
    best.onegravcall();
    x1.startpropagating(gracestart.nsgpstime()+100LL,earth_rate_ecef);
    x1.onegravcall();
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x1.vecef(), best.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x1.recef(), best.recef());
}

/** Test inputting the 2 valid Orbits, the second one
 * needs to be propagated in background.*/
void test_two_inputs_c() {
    orb::GroundPropagator est;
    orb::Orbit x0= gracestart;
    orb::Orbit x1(gracestart.nsgpstime()-1000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x1.applydeltav({1.0,0.0,0.0});
    est.input(x0,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x1,gracestart.nsgpstime(),earth_rate_ecef);
    CHECKINVARIANT(est);
    TEST_ASSERT(est.current.valid());
    TEST_ASSERT(est.catching_up.valid());
    TEST_ASSERT(!est.to_catch_up.valid());
    orb::Orbit best= est.best_estimate();
    TEST_ASSERT_TRUE(est.total_num_grav_calls_left());
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x0.vecef(), best.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x0.recef(), best.recef());
    x1.startpropagating(gracestart.nsgpstime(),earth_rate_ecef);
    x1.finishpropagating();
    while(est.total_num_grav_calls_left()) est.one_grav_call();
    CHECKINVARIANT(est);
    TEST_ASSERT(!est.catching_up.valid());
    TEST_ASSERT(!est.to_catch_up.valid());
    best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x1.vecef(), best.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x1.recef(), best.recef());
}

/** Test inputting the 2 valid Orbits, the first one
 * needs to be propagated in background, 
 * but should be over written by the second input.*/
void test_two_inputs_d() {
    orb::GroundPropagator est;
    orb::Orbit x0(gracestart.nsgpstime()-1000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x0.applydeltav({1.0,0.0,0.0});
    orb::Orbit x1= gracestart;
    est.input(x0,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x1,gracestart.nsgpstime(),earth_rate_ecef);
    CHECKINVARIANT(est);
    TEST_ASSERT(est.current.valid());
    TEST_ASSERT(!est.catching_up.valid());
    TEST_ASSERT(!est.to_catch_up.valid());
    orb::Orbit best= est.best_estimate();
    TEST_ASSERT_FALSE(est.total_num_grav_calls_left());
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x1.vecef(), best.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x1.recef(), best.recef());
}

/** Test inputting the 2 valid Orbits, the second one
 * needs to be propagated in background, 
 * but should be over write the first input because the
 * input time changes by the second input.*/
void test_two_inputs_e() {
    orb::GroundPropagator est;
    orb::Orbit x0= gracestart;
    orb::Orbit x1(gracestart.nsgpstime()-1000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x1.applydeltav({1.0,0.0,0.0});
    est.input(x0,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x1,gracestart.nsgpstime()-1000'000'000'000LL,earth_rate_ecef);
    CHECKINVARIANT(est);
    TEST_ASSERT(est.current.valid());
    TEST_ASSERT(!est.catching_up.valid());
    TEST_ASSERT(!est.to_catch_up.valid());
    orb::Orbit best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x1.vecef(), best.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x1.recef(), best.recef());
}

/** Test inputting the 2 valid Orbits, the second one
 * needs to be propagated in background, 
 * but should be over write the first input because needs 
 * less grav calls.*/
void test_two_inputs_f() {
    orb::GroundPropagator est;
    orb::Orbit x0(gracestart.nsgpstime()-2000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    orb::Orbit x1(gracestart.nsgpstime()-1000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x1.applydeltav({1.0,0.0,0.0});
    est.input(x0,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x1,gracestart.nsgpstime(),earth_rate_ecef);
    CHECKINVARIANT(est);
    TEST_ASSERT(est.current.valid());
    TEST_ASSERT(!est.catching_up.valid());
    TEST_ASSERT(!est.to_catch_up.valid());
    orb::Orbit best= est.best_estimate();
    best.finishpropagating();
    x1.startpropagating(gracestart.nsgpstime(),earth_rate_ecef);
    x1.finishpropagating();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x1.vecef(), best.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x1.recef(), best.recef());
}

/** Test inputting the 3 valid Orbits, the second one
 * needs to be propagated in background, 
 * and shouldn't be over written by the third input, because it
 * less grav calls.*/
void test_three_inputs_a() {
    orb::GroundPropagator est;
    orb::Orbit x0(gracestart.nsgpstime()-1000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    orb::Orbit x1(gracestart.nsgpstime()-2000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x1.applydeltav({1.0,0.0,0.0});
    orb::Orbit x2(gracestart.nsgpstime()-3000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x2.applydeltav({2.0,0.0,0.0});
    est.input(x0,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x1,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x2,gracestart.nsgpstime(),earth_rate_ecef);
    CHECKINVARIANT(est);
    TEST_ASSERT(est.current.valid());
    TEST_ASSERT(est.catching_up.valid());
    TEST_ASSERT(est.to_catch_up.valid());
    //propagate orbit x0 up
    x0.startpropagating(gracestart.nsgpstime(),earth_rate_ecef);
    while(x0.numgravcallsleft()){
        est.one_grav_call();
        x0.onegravcall();
    }
    CHECKINVARIANT(est);
    TEST_ASSERT(est.current.valid());
    TEST_ASSERT(est.catching_up.valid());
    TEST_ASSERT(est.to_catch_up.valid());
    orb::Orbit best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x0.vecef(), best.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x0.recef(), best.recef());
    //propagate orbit x1 up
    x1.startpropagating(gracestart.nsgpstime(),earth_rate_ecef);
    while(x1.numgravcallsleft()){
        CHECKINVARIANT(est);
        TEST_ASSERT_TRUE(est.best_estimate().valid());
        TEST_ASSERT_FALSE(est.best_estimate().numgravcallsleft());
        TEST_ASSERT(est.current.valid());
        TEST_ASSERT(est.catching_up.valid());
        TEST_ASSERT(est.to_catch_up.valid());
        est.one_grav_call();
        x1.onegravcall();
    }
    CHECKINVARIANT(est);
    TEST_ASSERT(est.current.valid());
    TEST_ASSERT(est.catching_up.valid());
    TEST_ASSERT(!est.to_catch_up.valid());
    best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x1.vecef(), best.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x1.recef(), best.recef());
    //propagate orbit x2 up
    x2.startpropagating(gracestart.nsgpstime(),earth_rate_ecef);
    while(x2.numgravcallsleft()){
        CHECKINVARIANT(est);
        TEST_ASSERT(est.current.valid());
        TEST_ASSERT_FALSE(best.numgravcallsleft());
        TEST_ASSERT(est.catching_up.valid());
        TEST_ASSERT(!est.to_catch_up.valid());
        est.one_grav_call();
        x2.onegravcall();
    }
    CHECKINVARIANT(est);
    TEST_ASSERT(est.current.valid());
    TEST_ASSERT(!est.catching_up.valid());
    TEST_ASSERT(!est.to_catch_up.valid());
    best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x2.vecef(), best.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x2.recef(), best.recef());
}

/** Test inputting the 3 valid Orbits, the second one
 * should be over written by the third input, because it
 * has less grav calls.*/
void test_three_inputs_b() {
    orb::GroundPropagator est;
    orb::Orbit x0(gracestart.nsgpstime()-1000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    orb::Orbit x1(gracestart.nsgpstime()-2000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x1.applydeltav({1.0,0.0,0.0});
    orb::Orbit x2(gracestart.nsgpstime()-1500'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x2.applydeltav({2.0,0.0,0.0});
    est.input(x0,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x1,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x2,gracestart.nsgpstime(),earth_rate_ecef);
    CHECKINVARIANT(est);
    TEST_ASSERT(est.current.valid());
    TEST_ASSERT(est.catching_up.valid());
    TEST_ASSERT(!est.to_catch_up.valid());
    //propagate orbit x0 up
    x0.startpropagating(gracestart.nsgpstime(),earth_rate_ecef);
    while(x0.numgravcallsleft()){
        est.one_grav_call();
        x0.onegravcall();
    }
    CHECKINVARIANT(est);
    TEST_ASSERT(est.current.valid());
    TEST_ASSERT(est.catching_up.valid());
    TEST_ASSERT(!est.to_catch_up.valid());
    orb::Orbit best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x0.vecef(), best.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x0.recef(), best.recef());
    //propagate orbit x2 up
    x2.startpropagating(gracestart.nsgpstime(),earth_rate_ecef);
    while(x2.numgravcallsleft()){
        CHECKINVARIANT(est);
        TEST_ASSERT_TRUE(est.best_estimate().valid());
        TEST_ASSERT_FALSE(est.best_estimate().numgravcallsleft());
        TEST_ASSERT(est.current.valid());
        TEST_ASSERT(est.catching_up.valid());
        TEST_ASSERT(!est.to_catch_up.valid());
        est.one_grav_call();
        x2.onegravcall();
    }
    CHECKINVARIANT(est);
    TEST_ASSERT(est.current.valid());
    TEST_ASSERT(!est.catching_up.valid());
    TEST_ASSERT(!est.to_catch_up.valid());
    best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x2.vecef(), best.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x2.recef(), best.recef());
}

/** Test inputting the 3 valid Orbits, the first and second one
 * should be over written by the third input, because it has
 * less grav calls.*/
void test_three_inputs_c() {
    orb::GroundPropagator est;
    orb::Orbit x0(gracestart.nsgpstime()-1000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    orb::Orbit x1(gracestart.nsgpstime()-2000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x1.applydeltav({1.0,0.0,0.0});
    orb::Orbit x2(gracestart.nsgpstime()-500'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x2.applydeltav({2.0,0.0,0.0});
    est.input(x0,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x1,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x2,gracestart.nsgpstime(),earth_rate_ecef);
    CHECKINVARIANT(est);
    TEST_ASSERT(est.current.valid());
    TEST_ASSERT(!est.catching_up.valid());
    TEST_ASSERT(!est.to_catch_up.valid());
    //propagate orbit x2 up
    x2.startpropagating(gracestart.nsgpstime(),earth_rate_ecef);
    while(x2.numgravcallsleft()){
        est.one_grav_call();
        x2.onegravcall();
    }
    CHECKINVARIANT(est);
    TEST_ASSERT(est.current.valid());
    TEST_ASSERT(!est.catching_up.valid());
    TEST_ASSERT(!est.to_catch_up.valid());
    orb::Orbit best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x2.vecef(), best.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x2.recef(), best.recef());
}

/** Test inputting the 2 valid Orbits and a invalid orbit.
 * The third input shouldn't do anything.*/
void test_three_inputs_d() {
    orb::GroundPropagator est;
    orb::Orbit x0(gracestart.nsgpstime()-1000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    orb::Orbit x1(gracestart.nsgpstime()-2000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x1.applydeltav({1.0,0.0,0.0});
    est.input(x0,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x1,gracestart.nsgpstime(),earth_rate_ecef);
    auto est_copy= est;
    est.input(orb::Orbit(),gracestart.nsgpstime(),earth_rate_ecef);
    TEST_ASSERT_EQUAL_MEMORY (&est, &est_copy, sizeof(orb::GroundPropagator));   
}

/** Test inputting the 4 valid Orbits, the second one
 * needs to be propagated in background.
 *  The fourth input should replace the third*/
void test_four_inputs_a() {
    orb::GroundPropagator est;
    orb::Orbit x0(gracestart.nsgpstime()-1000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    orb::Orbit x1(gracestart.nsgpstime()-2000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x1.applydeltav({1.0,0.0,0.0});
    orb::Orbit x2(gracestart.nsgpstime()-3000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x2.applydeltav({2.0,0.0,0.0});
    orb::Orbit x3(gracestart.nsgpstime()-4000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x3.applydeltav({3.0,0.0,0.0});
    est.input(x0,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x1,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x2,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x3,gracestart.nsgpstime(),earth_rate_ecef);
    CHECKINVARIANT(est);
    TEST_ASSERT(est.current.valid());
    TEST_ASSERT(est.catching_up.valid());
    TEST_ASSERT(est.to_catch_up.valid());
    //propagate orbit x0 up
    x0.startpropagating(gracestart.nsgpstime(),earth_rate_ecef);
    while(x0.numgravcallsleft()){
        est.one_grav_call();
        x0.onegravcall();
    }
    CHECKINVARIANT(est);
    TEST_ASSERT(est.current.valid());
    TEST_ASSERT(est.catching_up.valid());
    TEST_ASSERT(est.to_catch_up.valid());
    orb::Orbit best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x0.vecef(), best.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x0.recef(), best.recef());
    //propagate orbit x1 up
    x1.startpropagating(gracestart.nsgpstime(),earth_rate_ecef);
    while(x1.numgravcallsleft()){
        CHECKINVARIANT(est);
        TEST_ASSERT_TRUE(est.best_estimate().valid());
        TEST_ASSERT_FALSE(est.best_estimate().numgravcallsleft());
        TEST_ASSERT(est.current.valid());
        TEST_ASSERT(est.catching_up.valid());
        TEST_ASSERT(est.to_catch_up.valid());
        est.one_grav_call();
        x1.onegravcall();
    }
    CHECKINVARIANT(est);
    TEST_ASSERT(est.current.valid());
    TEST_ASSERT(est.catching_up.valid());
    TEST_ASSERT(!est.to_catch_up.valid());
    best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x1.vecef(), best.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x1.recef(), best.recef());
    //propagate orbit x3 up
    x3.startpropagating(gracestart.nsgpstime(),earth_rate_ecef);
    while(x3.numgravcallsleft()){
        CHECKINVARIANT(est);
        TEST_ASSERT(est.current.valid());
        TEST_ASSERT_FALSE(best.numgravcallsleft());
        TEST_ASSERT(est.catching_up.valid());
        TEST_ASSERT(!est.to_catch_up.valid());
        est.one_grav_call();
        x3.onegravcall();
    }
    CHECKINVARIANT(est);
    TEST_ASSERT(est.current.valid());
    TEST_ASSERT(!est.catching_up.valid());
    TEST_ASSERT(!est.to_catch_up.valid());
    best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x3.vecef(), best.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-10, x3.recef(), best.recef());
}

/** Test inputting the 3 valid Orbits and a invalid orbit.
 * The forth input shouldn't do anything.*/
void test_four_inputs_b() {
    orb::GroundPropagator est;
    orb::Orbit x0(gracestart.nsgpstime()-1000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    orb::Orbit x1(gracestart.nsgpstime()-2000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x1.applydeltav({1.0,0.0,0.0});
    orb::Orbit x2(gracestart.nsgpstime()-3000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x2.applydeltav({2.0,0.0,0.0});
    est.input(x0,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x1,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x2,gracestart.nsgpstime(),earth_rate_ecef);
    auto est_copy= est;
    est.input(orb::Orbit(),gracestart.nsgpstime(),earth_rate_ecef);
    TEST_ASSERT_EQUAL_MEMORY (&est, &est_copy, sizeof(orb::GroundPropagator));   
}

/** Test inputting 4 valid Orbits.
 * The 4th input should replace the 1st because it is most up to date.*/
void test_four_inputs_c() {
    orb::GroundPropagator est;
    orb::Orbit x0(gracestart.nsgpstime()-1000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    orb::Orbit x1(gracestart.nsgpstime()-2000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x1.applydeltav({1.0,0.0,0.0});
    orb::Orbit x2(gracestart.nsgpstime()-3000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x2.applydeltav({2.0,0.0,0.0});
    orb::Orbit x3(gracestart.nsgpstime()-500'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x3.applydeltav({3.0,0.0,0.0});
    est.input(x0,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x1,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x2,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x3,gracestart.nsgpstime(),earth_rate_ecef);
    x3.startpropagating(gracestart.nsgpstime(),earth_rate_ecef);
    orb::Orbit best= est.best_estimate();
    TEST_ASSERT_EQUAL_MEMORY (&best, &x3, sizeof(orb::Orbit));   
}

/** Test inputting 4 valid Orbits.
 * The 4th input should replace the 1st because it is most up to date.*/
void test_four_inputs_c() {
    orb::GroundPropagator est;
    orb::Orbit x0(gracestart.nsgpstime()-1000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    orb::Orbit x1(gracestart.nsgpstime()-2000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x1.applydeltav({1.0,0.0,0.0});
    orb::Orbit x2(gracestart.nsgpstime()-3000'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x2.applydeltav({2.0,0.0,0.0});
    orb::Orbit x3(gracestart.nsgpstime()-500'000'000'000LL,gracestart.recef(),gracestart.vecef());
    x3.applydeltav({3.0,0.0,0.0});
    est.input(x0,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x1,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x2,gracestart.nsgpstime(),earth_rate_ecef);
    est.input(x3,gracestart.nsgpstime(),earth_rate_ecef);
    x3.startpropagating(gracestart.nsgpstime(),earth_rate_ecef);
    orb::Orbit best= est.best_estimate();
    TEST_ASSERT_EQUAL_MEMORY (&best, &x3, sizeof(orb::Orbit));   
}




/** Returns real fake orbits sent from ground.*/
orb::Orbit real_fake_ground_data(int controlcycle){
    static orb::Orbit r=gracestart;
    switch (controlcycle)
    {
    case 100:
        //return an old orbit
        r=gracestart;
        return r;
        break;
    case 110:
        //return an old orbit
        r.startpropagating(r.nsgpstime()+100'000'000'000ULL,earth_rate_ecef);
        r.finishpropagating();
        return r;
        break;
    case 112:
        //return an old orbit
        r.startpropagating(r.nsgpstime()+1'000'000'000ULL,earth_rate_ecef);
        r.finishpropagating();
        return r;
        break;
    case 113:
        //return an old orbit
        r.startpropagating(r.nsgpstime()+20'000'000'000ULL,earth_rate_ecef);
        r.finishpropagating();
        return r;
        break;
    case 400:
        //return an old orbit
        r= grace100s;
        r.startpropagating(r.nsgpstime()+1000'000'000'000ULL,earth_rate_ecef);
        r.finishpropagating();
        return r;
        break;
    default:
        return orb::Orbit();
        break;
    }
}


void test_normal_use() {
    orb::GroundPropagator est;
    uint64_t gpsns= uint64_t(gnc::constant::init_gps_week_number)*gnc::constant::NANOSECONDS_IN_WEEK;
    gpsns+= 1000'000'000'000ULL;
    uint64_t dtgpsns= 120'000'000ULL;
    //start est with an up to date current estimate.
    est.input(orb::Orbit(gpsns,gracestart.recef(),gracestart.vecef()),gpsns,earth_rate_ecef);
    orb::Orbit best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    for (int controlcycle=0; controlcycle<1000; controlcycle++){
        gpsns+=dtgpsns;
        orb::Orbit ground_data= real_fake_ground_data(controlcycle);
        est.input(ground_data,gpsns,earth_rate_ecef);
        est.one_grav_call();
        est.one_grav_call();
        est.one_grav_call();
        est.one_grav_call();
        orb::Orbit best= est.best_estimate();
        //assert that the orbit never gets invalid or propagating
        TEST_ASSERT_TRUE(best.valid());
        TEST_ASSERT_FALSE(best.numgravcallsleft());
    }
    orb::Orbit lastsentorbit= grace100s;
    lastsentorbit.startpropagating(gpsns,earth_rate_ecef);
    lastsentorbit.finishpropagating();
    best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    TEST_ASSERT_TRUE(best.nsgpstime()==lastsentorbit.nsgpstime());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-8, best.vecef(), lastsentorbit.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-5, best.recef(), lastsentorbit.recef());
}

void test_time_going_backwards() {
    orb::GroundPropagator est;
    uint64_t gpsns= uint64_t(gnc::constant::init_gps_week_number)*gnc::constant::NANOSECONDS_IN_WEEK;
    gpsns+= 1000'000'000'000ULL;
    //Time is going backwards for some reason???
    int64_t dtgpsns= -120'000'000LL;
    //start est with an up to date current estimate.
    est.input(orb::Orbit(gpsns,gracestart.recef(),gracestart.vecef()),gpsns,earth_rate_ecef);
    orb::Orbit best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    for (int controlcycle=0; controlcycle<1000; controlcycle++){
        gpsns+=dtgpsns;
        orb::Orbit ground_data= real_fake_ground_data(controlcycle);
        est.input(ground_data,gpsns,earth_rate_ecef);
        est.one_grav_call();
        est.one_grav_call();
        est.one_grav_call();
        est.one_grav_call();
        orb::Orbit best= est.best_estimate();
        //assert that the orbit never gets invalid or propagating
        TEST_ASSERT_TRUE(best.valid());
        TEST_ASSERT_FALSE(best.numgravcallsleft());
    }
    orb::Orbit lastsentorbit= grace100s;
    lastsentorbit.startpropagating(gpsns,earth_rate_ecef);
    lastsentorbit.finishpropagating();
    best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    TEST_ASSERT_TRUE(best.nsgpstime()==lastsentorbit.nsgpstime());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-6, best.vecef(), lastsentorbit.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-3, best.recef(), lastsentorbit.recef());
}

void test_lots_of_ground_data() {
    orb::GroundPropagator est;
    uint64_t gpsns= uint64_t(gnc::constant::init_gps_week_number)*gnc::constant::NANOSECONDS_IN_WEEK;
    gpsns+= 1000'000'000'000ULL;
    int64_t dtgpsns= 120'000'000LL;
    //start est with an up to date current estimate.
    est.input(orb::Orbit(gpsns,gracestart.recef(),gracestart.vecef()),gpsns,earth_rate_ecef);
    orb::Orbit best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    for (int controlcycle=0; controlcycle<1000; controlcycle++){
        gpsns+=dtgpsns;
        orb::Orbit ground_data= gracestart;//ground data sent every controll cycle
        est.input(ground_data,gpsns,earth_rate_ecef);
        est.one_grav_call();
    }
    //because there is only one grav call per control cycle the propagator can never catch up.
    orb::Orbit lastsentorbit= orb::Orbit(gracestart.nsgpstime()+1000'000'000'000ULL,gracestart.recef(),gracestart.vecef());
    lastsentorbit.startpropagating(gpsns,earth_rate_ecef);
    lastsentorbit.finishpropagating();
    best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    TEST_ASSERT_TRUE(best.nsgpstime()==lastsentorbit.nsgpstime());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-6, best.vecef(), lastsentorbit.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-3, best.recef(), lastsentorbit.recef());
}

void test_lots_of_ground_data_catchingup() {
    orb::GroundPropagator est;
    uint64_t gpsns= uint64_t(gnc::constant::init_gps_week_number)*gnc::constant::NANOSECONDS_IN_WEEK;
    gpsns+= 1000'000'000'000ULL;
    int64_t dtgpsns= 120'000'000LL;
    //start est with an up to date current estimate.
    est.input(orb::Orbit(gpsns,gracestart.recef(),gracestart.vecef()),gpsns,earth_rate_ecef);
    orb::Orbit best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    for (int controlcycle=0; controlcycle<1000; controlcycle++){
        gpsns+=dtgpsns;
        orb::Orbit ground_data= gracestart;//ground data sent every controll cycle
        est.input(ground_data,gpsns,earth_rate_ecef);
        est.one_grav_call();
        est.one_grav_call();
    }
    //because there is two grav calls per control cycle the propagator can catch up.
    orb::Orbit lastsentorbit= gracestart;
    lastsentorbit.startpropagating(gpsns,earth_rate_ecef);
    lastsentorbit.finishpropagating();
    best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    TEST_ASSERT_TRUE(best.nsgpstime()==lastsentorbit.nsgpstime());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-6, best.vecef(), lastsentorbit.vecef());
    PAN_TEST_ASSERT_LIN_3VECT_WITHIN(1E-3, best.recef(), lastsentorbit.recef());
}

void test_edge_cases() {
    orb::GroundPropagator est;
    uint64_t t0= gracestart.nsgpstime();
    lin::Vector3d r0= gracestart.recef();
    lin::Vector3d v0= gracestart.vecef();
    est.input(orb::Orbit(t0+100,r0,v0),t0,earth_rate_ecef);
    orb::Orbit best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_EQUAL_INT(best.numgravcallsleft(),1);

    est.input(orb::Orbit(t0,r0,v0),t0,earth_rate_ecef);
    best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    TEST_ASSERT_FALSE(est.total_num_grav_calls_left());
    TEST_ASSERT_TRUE(best.nsgpstime()==t0);

    est.input(orb::Orbit(t0,r0,v0),t0,earth_rate_ecef);
    best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    TEST_ASSERT_FALSE(est.total_num_grav_calls_left());
    TEST_ASSERT_TRUE(best.nsgpstime()==t0);

    est.input(orb::Orbit(t0-100,r0,v0),t0,earth_rate_ecef);
    best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    TEST_ASSERT_EQUAL_INT(est.total_num_grav_calls_left(),1);
    TEST_ASSERT_TRUE(best.nsgpstime()==t0);

    est.input(orb::Orbit(t0-1'000'000'000LL,r0,v0),t0,earth_rate_ecef);
    best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    TEST_ASSERT_TRUE(est.total_num_grav_calls_left()>1);
    TEST_ASSERT_TRUE(best.nsgpstime()==t0);

    est.input(orb::Orbit(t0+100LL,r0,v0),t0-1'000'000'000LL,earth_rate_ecef);
    best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    TEST_ASSERT_TRUE(est.total_num_grav_calls_left());

    est.input(orb::Orbit(),t0+100LL,earth_rate_ecef);
    best= est.best_estimate();
    TEST_ASSERT_TRUE(best.valid());
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    TEST_ASSERT_TRUE(est.total_num_grav_calls_left()==0);
    TEST_ASSERT_TRUE(best.nsgpstime()==t0+100);

    est.reset_orbits();
    best= est.best_estimate();
    TEST_ASSERT_FALSE(best.valid());

}

int test_orbit() {
    UNITY_BEGIN();
    RUN_TEST(test_basic_constructors);
    RUN_TEST(test_propagating_invalid_orbits);
    RUN_TEST(test_input_invalid);
    RUN_TEST(test_1st_input_valid);
    RUN_TEST(test_two_inputs_a);
    RUN_TEST(test_two_inputs_b);
    RUN_TEST(test_two_inputs_c);
    RUN_TEST(test_two_inputs_d);
    RUN_TEST(test_two_inputs_e);
    RUN_TEST(test_two_inputs_f);
    RUN_TEST(test_three_inputs_a);
    RUN_TEST(test_three_inputs_b);
    RUN_TEST(test_three_inputs_c);
    RUN_TEST(test_three_inputs_d);
    RUN_TEST(test_three_inputs_d);
    RUN_TEST(test_four_inputs_a);
    RUN_TEST(test_four_inputs_b);
    RUN_TEST(test_four_inputs_c);
    RUN_TEST(test_normal_use);
    RUN_TEST(test_time_going_backwards);
    RUN_TEST(test_lots_of_ground_data);
    RUN_TEST(test_lots_of_ground_data_catchingup);
    RUN_TEST(test_edge_cases);
    return UNITY_END();
}


#ifdef DESKTOP
int main(int argc, char *argv[]) {
    return test_orbit();
}
#else
#include <Arduino.h>
void setup() {
    delay(10000);
    Serial.begin(9600);
    test_orbit();
}

void loop() {}
#endif