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
    lin::internal::RandomsGenerator const rand(0);
    {
    lin::Matrixf<8, 8> A, B, C;
    for (int i = 0; i < 25; i++) {
        A= lin::rands<decltype(A)>(A.rows(), A.cols(), rand);
        B= lin::rands<decltype(A)>(A.rows(), A.cols(), rand);
        orb::matrix_hypot(A,B,C);
        TEST_ASSERT_FLOAT_WITHIN(1E-10, 0.0,lin::fro(lin::transpose(C)*C-lin::transpose(A)*A-lin::transpose(B)*B));
    }
    }
    {
    lin::Matrixd<8, 8> A, B, C;
    for (int i = 0; i < 25; i++) {
        A= lin::rands<decltype(A)>(A.rows(), A.cols(), rand);
        B= lin::rands<decltype(A)>(A.rows(), A.cols(), rand);
        orb::matrix_hypot(A,B,C);
        TEST_ASSERT_FLOAT_WITHIN(1E-12, 0.0,lin::fro(lin::transpose(C)*C-lin::transpose(A)*A-lin::transpose(B)*B));
    }
    }
    {
    lin::Matrixf<6, 6> A, B, C;
    for (int i = 0; i < 25; i++) {
        A= lin::rands<decltype(A)>(A.rows(), A.cols(), rand);
        B= lin::rands<decltype(A)>(A.rows(), A.cols(), rand);
        orb::matrix_hypot(A,B,C);
        TEST_ASSERT_FLOAT_WITHIN(1E-10, 0.0,lin::fro(lin::transpose(C)*C-lin::transpose(A)*A-lin::transpose(B)*B));
    }
    }
    {
    lin::Matrixd<6, 6> A, B, C;
    for (int i = 0; i < 25; i++) {
        A= lin::rands<decltype(A)>(A.rows(), A.cols(), rand);
        B= lin::rands<decltype(A)>(A.rows(), A.cols(), rand);
        orb::matrix_hypot(A,B,C);
        TEST_ASSERT_FLOAT_WITHIN(1E-12, 0.0,lin::fro(lin::transpose(C)*C-lin::transpose(A)*A-lin::transpose(B)*B));
    }
    }
}


void test_potter_measurement_update(){
    // function [x,P] = kalman_correction_step(x,y,H,P,R)
    // % correction step of kalman filter, see canx paper eq 4.4 - 4.7
    // %get kalman gain K
    // K= P*H'/(H*P*H'+ R);
    // P= (eye(length(x))-K*H)*P*(eye(length(x))-K*H)'+K*R*K';
    // x= x + K*(y-H*x);
    // end
    lin::internal::RandomsGenerator const rand(0);
    {
        typedef float realtype;
        constexpr static int N=6;
        lin::Matrix<realtype,N, N> S;
        lin::Matrix<realtype,N, N> P;
        lin::Vector<realtype,N> x;
        lin::Vector<realtype,N> x_test;
        lin::RowVector<realtype,N> A;
        realtype invstddiv;
        realtype z;
        for (int i = 0; i < 25; i++) {
            S= lin::rands<decltype(S)>(S.rows(), S.cols(), rand);
            P= S*lin::transpose(S);
            A= lin::rands<decltype(A)>(A.rows(), A.cols(), rand);
            x= lin::rands<decltype(x)>(x.rows(), x.cols(), rand);
            x_test=x;
            lin::Vectorf<2> B= lin::rands<lin::Vectorf<2>>(2, 1, rand);
            invstddiv= std::abs(B(0));
            z= B(1);
            orb::potter_measurement_update(x_test,S,z,A,invstddiv);
            // now use regular kalman update
            realtype R= (1/invstddiv)*(1/invstddiv);
            lin::Vector<realtype,N> K= P*lin::transpose(A)/(lin::dot(A*P,lin::transpose(A))+R);
            P= ((lin::identity<realtype,N>()-K*A)*P*lin::transpose(lin::identity<realtype,N>()-K*A)+K*R*lin::transpose(K)).eval();
            x= (x + K*(z-lin::dot(A,x))).eval();
            TEST_ASSERT_FLOAT_WITHIN(1E-11, 0.0,lin::fro(x_test-x));
            TEST_ASSERT_FLOAT_WITHIN(1E-11, 0.0,lin::fro(P-S*lin::transpose(S)));
        }
    }
    {
        typedef double realtype;
        constexpr static int N=12;
        lin::Matrix<realtype,N, N> S;
        lin::Matrix<realtype,N, N> P;
        lin::Vector<realtype,N> x;
        lin::Vector<realtype,N> x_test;
        lin::RowVector<realtype,N> A;
        realtype invstddiv;
        realtype z;
        for (int i = 0; i < 25; i++) {
            S= lin::rands<decltype(S)>(S.rows(), S.cols(), rand);
            P= S*lin::transpose(S);
            A= lin::rands<decltype(A)>(A.rows(), A.cols(), rand);
            x= lin::rands<decltype(x)>(x.rows(), x.cols(), rand);
            x_test=x;
            lin::Vectorf<2> B= lin::rands<lin::Vectorf<2>>(2, 1, rand);
            invstddiv= std::abs(B(0));
            z= B(1);
            orb::potter_measurement_update(x_test,S,z,A,invstddiv);
            // now use regular kalman update
            realtype R= (1/invstddiv)*(1/invstddiv);
            lin::Vector<realtype,N> K= P*lin::transpose(A)/(lin::dot(A*P,lin::transpose(A))+R);
            P= ((lin::identity<realtype,N>()-K*A)*P*lin::transpose(lin::identity<realtype,N>()-K*A)+K*R*lin::transpose(K)).eval();
            x= (x + K*(z-lin::dot(A,x))).eval();
            TEST_ASSERT_FLOAT_WITHIN(1E-11, 0.0,lin::fro(x_test-x));
            TEST_ASSERT_FLOAT_WITHIN(1E-11, 0.0,lin::fro(P-S*lin::transpose(S)));
        }
    }

}


int test_kalman_utl() {
    UNITY_BEGIN();
    RUN_TEST(test_matrix_hypot);
    RUN_TEST(test_potter_measurement_update);
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