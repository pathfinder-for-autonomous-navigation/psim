/** Contains python wrappers of common. WIP
 * 
 */ 

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <string>
#include <stdio.h>
#include <lin/core.hpp>
#include <lin/generators.hpp>
#include <common/StateFieldRegistry.hpp>
#include <common/Event.hpp>
#include <common/Fault.hpp>
#include <fsw/FCCode/ControlTask.hpp>


namespace py = pybind11;

template <typename T>
class PyControlTask<T> : public ControlTask<T> {
public:
    /* Inherit the constructors */
    using ControlTask<T>::ControlTask<T>;

    /* Trampoline (need one for each virtual function) */
    T execute() {
        PYBIND11_OVERLOAD_PURE(
            T, /* Return type */
            ControlTask<T>,      /* Parent class */
            execute,          /* Name of function */
            T      /* Argument(s) */
        );
    }
};

/** 
 * Contains python wrappers of common. 
 */ 
void init_common_ext(py::module &m){
    py::class_<StateFieldRegistry>(m,"StateFieldRegistry")
        .def(py::init<>())

    py::class_<PyControlTask<void>> controltaskvoid(m,"ControlTaskvoid");
    controltask
        .alias<ControlTask<void>>()
        .def(py::init<StateFieldRegistry&>())
        .def("add_field",[](ControlTask<void>& x, InternalStateFieldBase& field){
            const bool added = _registry.add_internal_field(&field);
            check_field_added(added, field.name());
        })

        ;




    py::class_<ReadableStateFieldBase>(m,"ReadableStateFieldBase")
    py::class_<WritableStateFieldBase>(m,"WritableStateFieldBase")
    py::class_<Event>(m,"Event")
    py::class_<Fault>(m,"Fault")




    

}