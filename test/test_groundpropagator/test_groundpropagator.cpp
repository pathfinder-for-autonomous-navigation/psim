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
    TEST_ASSERT_TRUE(best.numgravcallsleft());
    TEST_ASSERT_TRUE(est.total_num_grav_calls_left()>1);
    best.startpropagating(t0-100,earth_rate_ecef);
    TEST_ASSERT_FALSE(best.numgravcallsleft());
    TEST_ASSERT_TRUE(best.nsgpstime()==t0-100);

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