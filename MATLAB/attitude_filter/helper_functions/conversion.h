#include "mex.hpp"
#include "mexAdapter.hpp"

#include <gnc/attitude_estimator.hpp>
#include <lin.hpp>

using namespace matlab::data;

template<lin::size_t N, typename T>
TypedArray<T> create_from_lin_vec(ArrayFactory& f, lin::Vector<T, N> lin_vec){
    TypedArray<T> ret = f.createArray<T>({N, 1});
    for(int r = 0; r < N; r++){
        ret[r][0] = lin_vec(r);
    }
    return ret;
}

template<lin::size_t N, typename T>
TypedArray<T> create_from_lin_vec_arr(ArrayFactory& f, lin::Vector<T, N>* lin_vec, size_t L){
    TypedArray<T> ret = f.createArray<T>({N, 1, L});
    for(int i = 0; i < L; i++){
        for(int r = 0; r < N; r++){
            ret[r][0][i] = (lin_vec[i])(r);
        }
    }
    return ret;    
}

template<lin::size_t R, lin::size_t C>
TypedArray<double> create_from_lin_mat(ArrayFactory& f, lin::Matrixd<R, C> lin_mat){
    TypedArray<double> ret = f.createArray<double>({R, C});
    for(int r = 0; r < R; r++){
        for(int c = 0; c < C; c++)
            ret[r][c] = lin_mat(r, c);
    }
    return ret;  
}