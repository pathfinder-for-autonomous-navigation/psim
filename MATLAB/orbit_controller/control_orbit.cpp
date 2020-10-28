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
        
        data_in.t = inputs[0][0];
        typed_array_to_lin_vec(data_in.r_ecef, inputs[1]);
        typed_array_to_lin_vec(data_in.v_ecef, inputs[2]);
        typed_array_to_lin_vec(data_in.dr_ecef, inputs[3]);
        typed_array_to_lin_vec(data_in.dv_ecef, inputs[4]);

        // assemble lin state - serves as a calculation buffer
        gnc::OrbitControllerState state = gnc::OrbitControllerState();

        state.t_last_firing = data_in.t;

        // empty actuation
        gnc::OrbitActuation actuation = gnc::OrbitActuation();

        control_orbit(state, data_in, actuation);

        ArrayFactory f;
        outputs[0] = create_from_lin_vec(f, actuation.J_ecef);
    }

    void checkArguments(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
        matlab::data::ArrayFactory factory;

        if (inputs.size() != 5) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("5 inputs required") }));
        }
    }


};