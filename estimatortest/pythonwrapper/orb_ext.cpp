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
        .def(py::pickle(
            [](const orb::Orbit &p) { // __getstate__
                /* Return a tuple that fully encodes the state of the object */
                return py::make_tuple(p.nsgpstime(),p.recef()(0),p.recef()(1),p.recef()(2),p.vecef()(0),p.vecef()(1),p.vecef()(2));
                },
            [](py::tuple t) { // __setstate__
                if (t.size() != 7)
                    throw std::runtime_error("Invalid state!");
                /* Create a new C++ instance */
                int64_t time= t[0].cast<int64_t>();
                double r0 = t[1].cast<double>();
                double r1 = t[2].cast<double>();
                double r2 = t[3].cast<double>();
                double v0 = t[4].cast<double>();
                double v1 = t[5].cast<double>();
                double v2 = t[6].cast<double>();
                orb::Orbit p(time,{r0,r1,r2},{v0,v1,v2});
                return p;
            }
        ))

        ;
}