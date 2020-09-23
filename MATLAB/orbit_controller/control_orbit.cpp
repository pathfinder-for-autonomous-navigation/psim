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

        // assemble lin data
        gnc::OrbitControllerData data_in = OrbitControllerData::OrbitControllerData();
        
        data_in.t = inputs[11][0];
        typed_array_to_lin_vec(data_in.r_ecef, inputs[12]);
        typed_array_to_lin_vec(data_in.v_ecef, inputs[13]);
        typed_array_to_lin_vec(data_in.dr_ecef, inputs[14]);
        typed_array_to_lin_vec(data_in.dv_ecef, inputs[15]);

        // assemble lin state - serves as a calculation buffer
        gnc::OrbitControllerState state = OrbitControllerState::OrbitControllerState();

        state.t_last_firing = inputs[0][0];
        typed_array_to_lin_vec(state.this_r_ecef0, inputs[1]);
        typed_array_to_lin_vec(state.that_r_ecef0, inputs[2]);
        typed_array_to_lin_vec(state.this_r_hat, inputs[3]);
        typed_array_to_lin_vec(state.this_v_ecef0, inputs[4]);
        typed_array_to_lin_vec(state.that_v_ecef0, inputs[5]);
        typed_array_to_lin_vec(state.this_v_hat, inputs[6]);
        typed_array_to_lin_vec(state.this_h_ecef0, inputs[7]);
        typed_array_to_lin_vec(state.that_h_ecef0, inputs[8]);
        typed_array_to_lin_vec(state.this_h_hat, inputs[9]);
        typed_array_to_lin_mat(state.DCM_hill_ecef0, inputs[10]);

        // empty actuation
        gnc::OrbitActuation actuation = OrbitActuation::OrbitActuation();

        gnc::control_orbit(state, data_in, actuation, mass, p, d, energy_gain, h_gain);
        
        ArrayFactory f;

        // will only contain one element
        StructArray S = f.createStructArray({1,1}, 
            {"t_last_firing",
             "r2", "r1", "r2hat", 
             "v2", "v1", "v2hat",
             "h2", "h1", "h2hat",
             "dcm_hill_ecef0"});

        S[0]["t_last_firing"] = create_from_lin_vec(f, state.t_last_firing);
        S[0]["r2"] = create_from_lin_vec(f, state.this_r_ecef0);
        S[0]["r1"] = create_from_lin_vec(f, state.that_r_ecef0);
        S[0]["r2hat"] = create_from_lin_vec(f, state.this_r_hat);
        S[0]["v2"] = create_from_lin_vec(f, state.this_v_ecef0);
        S[0]["v1"] = create_from_lin_vec(f, state.that_v_ecef0);
        S[0]["v2hat"] = create_from_lin_vec(f, state.this_v_hat);
        S[0]["h2"] = create_from_lin_vec(f, state.this_h_ecef0);
        S[0]["h1"] = create_from_lin_vec(f, state.that_h_ecef0);
        S[0]["h2hat"] = create_from_lin_vec(f, state.this_h_hat);
        S[0]["dcm_hill_ecef0"] = create_from_lin_vec(f, state.DCM_hill_ecef0);

        //outputs[0] is the state struct
        outputs[0] = S;

        outputs[1] = create_from_lin_vec(f, actuation.J_ecef);
        outputs[2] = create_from_lin_vec(f, actuation.phase_till_next_node);
    }

    void checkArguments(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
        matlab::data::ArrayFactory factory;

        // assuming t, then q

        if (inputs.size() != 16) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("16 inputs required") }));
        }
    }


}