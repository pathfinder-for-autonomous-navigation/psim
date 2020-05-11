/** Contains python wrappers of used lin types. 
 * 
 */ 

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <string>
#include <stdio.h>
#include <orb/Orbit.h>
#include <lin/core.hpp>
#include <lin/generators.hpp>
#include <sstream>


namespace py = pybind11;

/**
 * Wrap lin Matrix.
 * */
#define WRAPLINMATRIX(pythontypename,module,T,R,C,MR,MC) py::class_<lin::Matrix<T, R, C, MR, MC>>(module,pythontypename,py::buffer_protocol())\
        .def("__init__", [](lin::Matrix<T, R, C, MR, MC> &x, py::array_t<T> b) { \
            /* Request a buffer descriptor from Python */ \
            py::buffer_info info = b.request(); \
            /* Some sanity checks ... */ \
            if (info.format != py::format_descriptor<T>::format()) \
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
                    std::ostringstream ss; \
                    ss << x(i,j); \
                    out += ss.str()+","; \
                } \
            } \
            return out; \
        }) \
        .def("__call__",[](const lin::Matrix<T, R, C, MR, MC>& x, lin::size_t i, lin::size_t j){return x(i,j);}) \
        .def("__call__",[](const lin::Matrix<T, R, C, MR, MC>& x, lin::size_t i){return x(i);}) \
        .def(py::pickle(\
            [](const lin::Matrix<T, R, C, MR, MC>& p) { /* __getstate__ */ \
                /* Return a tuple that fully encodes the state of the object */   \
                std::array<T,MR*MC> a; \
                lin::Matrix<T, R, C, MR, MC> c= p; \
                memcpy(&a,&(c(0)),MR*MC*sizeof(T)); \
                return a; \
                }, \
            [](std::array<T,MR*MC> t) { /* __setstate__ */ \
                /* Create a new C++ instance */ \
                lin::Matrix<T, R, C, MR, MC> p; \
                memcpy(&(p(0)),&t,MR*MC*sizeof(T)); \
                return p; \
            } \
        ))\

/**
 * Wrap lin Vector.
 * */
#define WRAPLINVECTOR(pythontypename,module,T,N,MN) py::class_<lin::Vector<T, N, MN>>(module,pythontypename,py::buffer_protocol())\
        .def("__init__", [](lin::Vector<T, N, MN> &x, py::array_t<T> b) { \
            /* Request a buffer descriptor from Python */ \
            py::buffer_info info = b.request(); \
            /* Some sanity checks ... */ \
            if (info.format != py::format_descriptor<T>::format()) \
                throw std::runtime_error("Incompatible format: expected a " #T " array!"); \
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
                    std::ostringstream ss; \
                    ss << x(i,j); \
                    out += ss.str()+","; \
                } \
            } \
            return out; \
        }) \
        .def("__call__",[](const lin::Vector<T, N, MN>& x, lin::size_t i, lin::size_t j){return x(i,j);}) \
        .def("__call__",[](const lin::Vector<T, N, MN>& x, lin::size_t i){return x(i);}) \
        .def(py::pickle(\
            [](const lin::Vector<T, N, MN> &p) { /* __getstate__ */ \
                /* Return a tuple that fully encodes the state of the object */   \
                std::array<T,MN> a; \
                lin::Vector<T, N, MN> c= p; \
                memcpy(&a,&(c(0)),MN*sizeof(T)); \
                return a; \
                }, \
            [](std::array<T,MN> t) { /* __setstate__ */ \
                /* Create a new C++ instance */ \
                lin::Vector<T, N, MN> p; \
                memcpy(&(p(0)),&t,MN*sizeof(T)); \
                return p; \
            } \
        ))\


/** 
 * Contains python wrappers of used lin types. 
 */ 
void init_lin_ext(py::module &m){
    WRAPLINVECTOR("lin_Vector2f",m,float,2,2);
    WRAPLINVECTOR("lin_Vector3f",m,float,3,3);
    WRAPLINVECTOR("lin_Vector4f",m,float,4,4);
    WRAPLINVECTOR("lin_Vector5f",m,float,5,5);
    WRAPLINVECTOR("lin_Vector6f",m,float,6,6);
    WRAPLINVECTOR("lin_Vector7f",m,float,7,7);
    WRAPLINVECTOR("lin_Vector8f",m,float,8,8);
    WRAPLINVECTOR("lin_Vector9f",m,float,9,9);
    WRAPLINVECTOR("lin_Vector10f",m,float,10,10);
    WRAPLINVECTOR("lin_Vector11f",m,float,11,11);
    WRAPLINVECTOR("lin_Vector12f",m,float,12,12);

    WRAPLINVECTOR("lin_Vector2d",m,double,2,2);
    WRAPLINVECTOR("lin_Vector3d",m,double,3,3);
    WRAPLINVECTOR("lin_Vector4d",m,double,4,4);
    WRAPLINVECTOR("lin_Vector5d",m,double,5,5);
    WRAPLINVECTOR("lin_Vector6d",m,double,6,6);
    WRAPLINVECTOR("lin_Vector7d",m,double,7,7);
    WRAPLINVECTOR("lin_Vector8d",m,double,8,8);
    WRAPLINVECTOR("lin_Vector9d",m,double,9,9);
    WRAPLINVECTOR("lin_Vector10d",m,double,10,10);
    WRAPLINVECTOR("lin_Vector11d",m,double,11,11);
    WRAPLINVECTOR("lin_Vector12d",m,double,12,12);

    WRAPLINMATRIX("lin_Matrix2x2f",m,float,2,2,2,2);
    WRAPLINMATRIX("lin_Matrix3x3f",m,float,3,3,3,3);
    WRAPLINMATRIX("lin_Matrix4x4f",m,float,4,4,4,4);
    WRAPLINMATRIX("lin_Matrix5x5f",m,float,5,5,5,5);
    WRAPLINMATRIX("lin_Matrix6x6f",m,float,6,6,6,6);
    WRAPLINMATRIX("lin_Matrix7x7f",m,float,7,7,7,7);
    WRAPLINMATRIX("lin_Matrix8x8f",m,float,8,8,8,8);
    WRAPLINMATRIX("lin_Matrix9x9f",m,float,9,9,9,9);
    WRAPLINMATRIX("lin_Matrix10x10f",m,float,10,10,10,10);
    WRAPLINMATRIX("lin_Matrix11x11f",m,float,11,11,11,11);
    WRAPLINMATRIX("lin_Matrix12x12f",m,float,12,12,12,12);

    WRAPLINMATRIX("lin_Matrix2x2d",m,double,2,2,2,2);
    WRAPLINMATRIX("lin_Matrix3x3d",m,double,3,3,3,3);
    WRAPLINMATRIX("lin_Matrix4x4d",m,double,4,4,4,4);
    WRAPLINMATRIX("lin_Matrix5x5d",m,double,5,5,5,5);
    WRAPLINMATRIX("lin_Matrix6x6d",m,double,6,6,6,6);
    WRAPLINMATRIX("lin_Matrix7x7d",m,double,7,7,7,7);
    WRAPLINMATRIX("lin_Matrix8x8d",m,double,8,8,8,8);
    WRAPLINMATRIX("lin_Matrix9x9d",m,double,9,9,9,9);
    WRAPLINMATRIX("lin_Matrix10x10d",m,double,10,10,10,10);
    WRAPLINMATRIX("lin_Matrix11x11d",m,double,11,11,11,11);
    WRAPLINMATRIX("lin_Matrix12x12d",m,double,12,12,12,12);

}