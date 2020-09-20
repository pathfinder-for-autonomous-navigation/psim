#include "mex.hpp"
#include "mexAdapter.hpp"

#include <gnc/attitude_estimator.hpp>
#include <lin.hpp>

#include "conversion.hpp"

class MexFunction : public matlab::mex::Function {
public:

    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        checkArguments(outputs, inputs);
        
        // assemble lin data
        gnc::AttitudeEstimatorData data_in = gnc::AttitudeEstimatorData();
        
        data_in.t = inputs[4][0];
        typed_array_to_lin_vec(data_in.r_ecef, inputs[5]);
        typed_array_to_lin_vec(data_in.b_body, inputs[6]);
        typed_array_to_lin_vec(data_in.s_body, inputs[7]);
        typed_array_to_lin_vec(data_in.w_body, inputs[8]);

        // assemble lin state
        gnc::AttitudeEstimatorState state = gnc::AttitudeEstimatorState();

        typed_array_to_lin_vec(state.q, inputs[0]);
        typed_array_to_lin_vec(state.x, inputs[1]);
        typed_array_to_lin_mat(state.P, inputs[2]);
        state.t = inputs[3][0];
        
        state.is_valid = (lin::all(lin::isfinite(state.q)) &&
            lin::all(lin::isfinite(state.x)) &&
            lin::all(lin::isfinite(state.P)) &&
            lin::isfinite(state.t));

        // empty estimate
        gnc::AttitudeEstimate estimate = gnc::AttitudeEstimate();

        gnc::attitude_estimator_update(state, data_in, estimate);
        
        ArrayFactory f;

        // will only contain one element
        StructArray S = f.createStructArray({1,1}, 
            {"q", "x","P","t"});

        S[0]["q"] = create_from_lin_vec(f, state.q);
        S[0]["x"] = create_from_lin_vec(f, state.x);
        S[0]["P"] = create_from_lin_mat(f, state.P);

        lin::Matrixd<1,1> dummy = {state.t};
        S[0]["t"] = create_from_lin_mat(f, dummy);

        //outputs[0] is the state struct
        outputs[0] = S;

        outputs[1] = create_from_lin_vec(f, estimate.q_body_eci);
        outputs[2] = create_from_lin_vec(f, estimate.gyro_bias);
        outputs[3] = create_from_lin_mat(f, estimate.P);
    }

    void checkArguments(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
        matlab::data::ArrayFactory factory;

        if (inputs.size() != 9) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("9 inputs required") }));
        }

    }
};