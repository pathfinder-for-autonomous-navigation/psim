#include "mex.hpp"
#include "mexAdapter.hpp"

#include <gnc/orbit_controller.hpp>
#include <../../src/gnc_orbit_controller.cpp>
#include <lin.hpp>

#include "conversion.hpp"

using namespace matlab::data;

class MexFunction : public matlab::mex::Function {
public:

    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        checkArguments(outputs, inputs);

        static constexpr double mass = 3.7;
        static constexpr double k_p = 1.0e-6;
        static constexpr double k_d = 5.0e-2;
        static constexpr double k_e = 5.0e-5;
        static constexpr double k_h = 2.0e-3;

        gnc::OrbitControllerState state = gnc::OrbitControllerState();
        gnc::OrbitControllerData data = gnc::OrbitControllerData();
        gnc::OrbitActuation actuation = gnc::OrbitActuation();
        gnc::mex_control_orbit(state, data, actuation, mass, k_p, k_d, k_e, k_h);
        
        ArrayFactory f;

        // will only contain one element
        StructArray S = f.createStructArray({1,1}, 
            {"q", "x","P","t"});

        S[0]["q"] = create_from_lin_vec(f, state.q);
        S[0]["x"] = create_from_lin_vec(f, state.x);
        S[0]["P"] = create_from_lin_mat(f, state.P);

        lin::Matrixd<1,1> dummy = {state.t};
        S[0]["t"] = create_from_lin_mat(f, dummy);

        //outputs[0] is the struct
        outputs[0] = S;
    }

}