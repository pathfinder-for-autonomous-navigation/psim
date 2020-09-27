#include "mex.hpp"
#include "mexAdapter.hpp"

#include <gnc/orbit_controller.hpp>
#include <lin.hpp>

#include "conversion.hpp"

using namespace matlab::data;

class MexFunction : public matlab::mex::Function {
public:

    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        checkArguments(outputs, inputs);

        // assemble lin data
        gnc::OrbitControllerData data_in = gnc::OrbitControllerData();
        
        data_in.t = inputs[1][0];
        typed_array_to_lin_vec(data_in.r_ecef, inputs[2]);
        typed_array_to_lin_vec(data_in.v_ecef, inputs[3]);
        typed_array_to_lin_vec(data_in.dr_ecef, inputs[4]);
        typed_array_to_lin_vec(data_in.dv_ecef, inputs[5]);

        // assemble lin state - serves as a calculation buffer
        gnc::OrbitControllerState state = gnc::OrbitControllerState();

        state.t_last_firing = inputs[0][0];

        // empty actuation
        gnc::OrbitActuation actuation = gnc::OrbitActuation();

        control_orbit(state, data_in, actuation);
        
        ArrayFactory f;

        // will only contain one element
        StructArray S = f.createStructArray({1,1}, {"t_last_firing"});

        lin::Matrixd<1,1> dummy = {state.t_last_firing};
        S[0]["t_last_firing"] = create_from_lin_mat(f, dummy);

        //outputs[0] is the state struct - just holds t_last_fire
        outputs[0] = S;

        outputs[1] = create_from_lin_vec(f, actuation.J_ecef);
        dummy = {actuation.phase_till_next_node};
        outputs[2] = create_from_lin_mat(f, dummy);
    }

    void checkArguments(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
        matlab::data::ArrayFactory factory;

        // assuming t, then q

        if (inputs.size() != 6) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("6 inputs required") }));
        }
    }


};