/** Contains python wrappers of orb. 
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
 * Contains python wrappers of orb. 
 */ 
void init_orb_ext(py::module &m){
        py::class_<orb::Orbit>(m,"orb_Orbit")
        .def(py::init<const int64_t&,const lin::Vector3d&,const lin::Vector3d&>())
        .def(py::init<>())
        .def("nsgpstime", &orb::Orbit::nsgpstime)
        .def("recef", &orb::Orbit::recef)
        .def("vecef", &orb::Orbit::vecef)
        .def("valid", &orb::Orbit::valid)
        .def("__repr__",[](const orb::Orbit& x){
            return 
                "Orbit\n"
                "gps time: "+std::to_string(x.nsgpstime())+" ns\n"+
                "position ECEF: "+std::to_string(x.recef()(0))+", "+std::to_string(x.recef()(1))+", "+std::to_string(x.recef()(2))+" m\n"+
                "velocity ECEF: "+std::to_string(x.vecef()(0))+", "+std::to_string(x.vecef()(1))+", "+std::to_string(x.vecef()(2))+" m/s";
        })
        .def("applydeltav", &orb::Orbit::applydeltav)
        .def("specificenergy", &orb::Orbit::specificenergy)
        .def("shortupdate", [](orb::Orbit& x, int32_t dt_ns, const lin::Vector3d &earth_rate_ecef){
            double specificenergy; 
            lin::Matrix< double, 6, 6 > jac;
            x.shortupdate(dt_ns,earth_rate_ecef,specificenergy,jac);
            return std::make_tuple(specificenergy, jac);
        })
        .def("update", [](orb::Orbit& x, const int64_t &end_gps_time_ns, const lin::Vector3d &earth_rate_ecef){
            x.startpropagating(end_gps_time_ns,earth_rate_ecef);
            x.finishpropagating();
        })
        .def_static("calc_geograv", [](const lin::Vector3d &r_ecef){
            double pot;
            lin::Vector3d g_ecef;
            orb::Orbit::calc_geograv (r_ecef, g_ecef, pot);
            return std::make_tuple(g_ecef, pot);
        })

        ;
}