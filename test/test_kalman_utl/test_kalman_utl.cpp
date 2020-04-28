#include <stdio.h>
#include <cstdint>
#include <limits>
#include <lin.hpp>
#include "../custom_assertions.hpp"
#include <orb/kalman_utl.h>
#ifndef DESKTOP
#include <Arduino.h>
#endif
#include <unity.h>

void test_matrix_hypot() {
    lin::internal::RandomsGenerator const rand;
    lin::Matrixf<8, 8> A, B, C;
    for (int i = 0; i < 25; i++) {
        A= lin::rands<decltype(A)>(A.rows(), A.cols(), rand);
        B= lin::rands<decltype(A)>(A.rows(), A.cols(), rand);
        orb::matrix_hypot(A,B,C);
        TEST_ASSERT(0.0==lin::fro(lin::transpose(C)*C-lin::transpose(A)*A-lin::transpose(B)*B));
    }
}


int test_kalman_utl() {
    UNITY_BEGIN();
    RUN_TEST(test_matrix_hypot);
    return UNITY_END();
}

#ifdef DESKTOP
int main(int argc, char *argv[]) {
    return test_kalman_utl();
}
#else
#include <Arduino.h>
void setup() {
    delay(10000);
    Serial.begin(9600);
    test_kalman_utl();
}

void loop() {}
#endif