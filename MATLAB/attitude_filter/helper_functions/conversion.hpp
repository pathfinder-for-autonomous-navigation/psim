#include "mex.hpp"
#include "mexAdapter.hpp"

#include <gnc/attitude_estimator.hpp>
#include <lin.hpp>

using namespace matlab::data;

/**
 * @brief Create a matlab array from lin vec object
 * 
 * @tparam N the dimension of the lin::vector
 * @tparam T the type of lin::vector 
 * @param f a matlab array factory
 * @param lin_vec input lin::vector to be copied from
 * @return TypedArray<T> a matlab array
 */
template<lin::size_t N, typename T>
TypedArray<T> create_from_lin_vec(ArrayFactory& f, const lin::Vector<T, N>& lin_vec){
    TypedArray<T> ret = f.createArray<T>({N, 1});
    for(int r = 0; r < N; r++){
        ret[r][0] = lin_vec(r);
    }
    return ret;
}

/**
 * @brief Create an array of matlab arrays from a list of lin::vec arrays
 * 
 * @tparam N length of each sub array
 * @tparam T type of input element
 * @param f matlab array factory
 * @param lin_vec input list of lin::vecs
 * @param L number of lin::vecs
 * @return TypedArray<T> returned array of arrays
 */
template<lin::size_t N, typename T>
TypedArray<T> create_from_lin_vec_arr(ArrayFactory& f, const lin::Vector<T, N>* lin_vec, size_t L){
    TypedArray<T> ret = f.createArray<T>({N, 1, L});
    for(int i = 0; i < L; i++){
        for(int r = 0; r < N; r++){
            ret[r][0][i] = (lin_vec[i])(r);
        }
    }
    return ret;    
}

/**
 * @brief Create a from lin mat object
 * 
 * @tparam R rows
 * @tparam C columns
 * @tparam T input element type
 * @param f matlab array factory
 * @param lin_mat input lin matrix
 * @return TypedArray<T> matlab output matrix
 */
template<lin::size_t R, lin::size_t C, typename T>
TypedArray<T> create_from_lin_mat(ArrayFactory& f, const lin::Matrix<T, R, C>& lin_mat){
    TypedArray<T> ret = f.createArray<T>({R, C});
    for(int r = 0; r < R; r++){
        for(int c = 0; c < C; c++)
            ret[r][c] = lin_mat(r, c);
    }
    return ret;  
}

/**
 * @brief copy a matlab array into a lin::Vec
 * 
 * @tparam N array length
 * @tparam T element type
 * @param lin_vec lin::vec output reference
 * @param arr matlab array input
 */
template<lin::size_t N, typename T>
void typed_array_to_lin_vec(lin::Vector<T, N>& lin_vec, matlab::data::Array& arr){
    for (int i = 0; i<N; i++) {
        lin_vec(i) = arr[i];
    }
}

/**
 * @brief Copy a matlab matrix into a lin::matrix
 * 
 * @tparam T element type
 * @tparam R rows
 * @tparam C columns
 * @param lin_mat lin::matrix output reference
 * @param arr matlab array input
 */
template<typename T, lin::size_t R, lin::size_t C>
void typed_array_to_lin_mat(lin::Matrix<T, R, C>& lin_mat, matlab::data::Array& arr){
    for( int r = 0; r<R; r++){
        for( int c = 0; c<C; c++){
            lin_mat(r, c) = arr[r][c];
        }
    }
}