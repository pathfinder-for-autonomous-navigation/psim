/** Contains python wrappers of used lin types. 
 * 
 */ 

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <string>
#include <stdio.h>
#include <orb/Orbit.h>
#include <lin/core.hpp>
#include <lin/generators.hpp>

namespace py = pybind11;

/**
 * Wrap lin Matrix.
 * */
#define WRAPLINMATRIX(pythontypename,module,T,R,C,MR,MC) py::class_<lin::Matrix<T, R, C, MR, MC>>(module,pythontypename,py::buffer_protocol())\
        .def("__init__", [](lin::Matrix<T, R, C, MR, MC> &x, py::array_t<T> b) { \
            /* Request a buffer descriptor from Python */ \
            py::buffer_info info = b.request(); \
            /* Some sanity checks ... */ \
            if (info.format != py::format_descriptor<double>::format()) \
                throw std::runtime_error("Incompatible format: expected a " #T " array!"); \
            if (info.ndim != 2) \
                throw std::runtime_error("Incompatible buffer dimension!"); \
            if (info.shape[0] != (long)(x.rows())) \
                throw std::runtime_error("Incompatible buffer shape!"); \
            if (info.shape[1] != (long)(x.cols())) \
                throw std::runtime_error("Incompatible buffer shape!"); \
            lin::size_t n= x.rows(); \
            lin::size_t m= x.cols(); \
            for (lin::size_t i=0; i<n; i++){ \
                for (lin::size_t j=0; j<m; j++){ \
                    x(i,j)= *(reinterpret_cast<T*>(reinterpret_cast<char*>(info.ptr)+i*info.strides[0]+j*info.strides[1])); \
                } \
            } \
        }) \
        .def_buffer([](lin::Matrix<T, R, C, MR, MC>& x)->py::buffer_info{ \
            return py::buffer_info( \
                &(x(0)),                            /* Pointer to buffer */ \
                sizeof(T),                          /* Size of one scalar */ \
                py::format_descriptor<T>::format(), /* Python struct-style format descriptor */ \
                2,                                  /* Number of dimensions */ \
                {x.rows(),x.cols()},                /* Buffer dimensions */ \
                { MC*sizeof(T),sizeof(T)}           /* Strides (in bytes) for each index */ \
            ); \
        }) \
        .def("__repr__",[](const lin::Matrix<T, R, C, MR, MC>& x){ \
            std::string out=pythontypename; \
            lin::size_t n= x.rows(); \
            lin::size_t m= x.cols(); \
            for (lin::size_t i=0; i<n; i++){ \
                out += "\n"; \
                for (lin::size_t j=0; j<m; j++){ \
                    out += std::to_string(x(i,j))+","; \
                } \
            } \
            return out; \
        }) \
        .def("__call__",[](const lin::Matrix<T, R, C, MR, MC>& x, lin::size_t i, lin::size_t j){return x(i,j);}) \
        .def("__call__",[](const lin::Matrix<T, R, C, MR, MC>& x, lin::size_t i){return x(i);}) \

/**
 * Wrap lin Vector.
 * */
#define WRAPLINVECTOR(pythontypename,module,T,N,MN) py::class_<lin::Vector<T, N, MN>>(module,pythontypename,py::buffer_protocol())\
        .def("__init__", [](lin::Vector<T, N, MN> &x, py::array_t<T> b) { \
            /* Request a buffer descriptor from Python */ \
            py::buffer_info info = b.request(); \
            /* Some sanity checks ... */ \
            if (info.format != py::format_descriptor<double>::format()) \
                throw std::runtime_error("Incompatible format: expected a double array!"); \
            if (info.ndim != 1) \
                throw std::runtime_error("Incompatible buffer dimension!"); \
            if (info.shape[0] != (long)(x.size())) \
                throw std::runtime_error("Incompatible buffer shape!"); \
            lin::size_t n= x.size(); \
            for (lin::size_t i=0; i<n; i++){ \
                x(i)= *(reinterpret_cast<T*>(reinterpret_cast<char*>(info.ptr)+i*info.strides[0])); \
            } \
        }) \
        .def_buffer([](lin::Vector<T, N, MN>& x)->py::buffer_info{ \
            return py::buffer_info( \
                &(x(0)),                            /* Pointer to buffer */ \
                sizeof(T),                          /* Size of one scalar */ \
                py::format_descriptor<T>::format(), /* Python struct-style format descriptor */ \
                1,                                  /* Number of dimensions */ \
                {x.size()},                         /* Buffer dimensions */ \
                {sizeof(T)}                         /* Strides (in bytes) for each index */ \
            ); \
        }) \
        .def("__repr__",[](const lin::Vector<T, N, MN>& x){ \
            std::string out=pythontypename; \
            lin::size_t n= x.rows(); \
            lin::size_t m= x.cols(); \
            for (lin::size_t i=0; i<n; i++){ \
                out += "\n"; \
                for (lin::size_t j=0; j<m; j++){ \
                    out += std::to_string(x(i,j))+","; \
                } \
            } \
            return out; \
        }) \
        .def("__call__",[](const lin::Vector<T, N, MN>& x, lin::size_t i, lin::size_t j){return x(i,j);}) \
        .def("__call__",[](const lin::Vector<T, N, MN>& x, lin::size_t i){return x(i);}) \


/** 
 * Contains python wrappers of used lin types. 
 */ 
void init_lin_ext(py::module &m){
    WRAPLINMATRIX("lin_Matrix6x6d",m,double,6,6,6,6);
    WRAPLINVECTOR("lin_Vector3d",m,double,3,3);
}