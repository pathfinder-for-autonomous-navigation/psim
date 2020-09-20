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
        
        // data_in.t = inputs[4][0];
        // typed_array_to_lin_vec(data_in.r_ecef, inputs[5]);
        // typed_array_to_lin_vec(data_in.b_body, inputs[6]);
        // typed_array_to_lin_vec(data_in.s_body, inputs[7]);
        // typed_array_to_lin_vec(data_in.w_body, inputs[8]);


        // assemble lin state
        gnc::AttitudeEstimatorState state = gnc::AttitudeEstimatorState();

        lin::Vector3f r_ecef;
        lin::Vector3f b_body;
        lin::Vector3f s_body;

        double time = inputs[0][0];
        typed_array_to_lin_vec(r_ecef, inputs[1]);
        typed_array_to_lin_vec(b_body, inputs[2]);
        typed_array_to_lin_vec(s_body, inputs[3]);

        // state.is_valid = (lin::all(lin::isfinite(state.q)) &&
        //     lin::all(lin::isfinite(state.x)) &&
        //     lin::all(lin::isfinite(state.P)) &&
        //     lin::isfinite(state.t));

        // empty estimate
        // gnc::AttitudeEstimate estimate = gnc::AttitudeEstimate();

        // gnc::attitude_estimator_update(state, data_in, estimate);
        gnc::attitude_estimator_reset(state, time, r_ecef, b_body, s_body);

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

    void checkArguments(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
        matlab::data::ArrayFactory factory;

        if (inputs.size() != 4) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("9 inputs required") }));
        }

        // if (inputs[5].getNumberOfElements() != 1) {
        //     matlabPtr->feval(u"error", 
        //         0, std::vector<matlab::data::Array>({ factory.createScalar("Time must be a scalar") }));
        // }
        
        // if (inputs[5].getType() != matlab::data::ArrayType::DOUBLE ||
        //     inputs[5].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
        //     matlabPtr->feval(u"error", 
        //         0, std::vector<matlab::data::Array>({ factory.createScalar("Time multiplier must be a noncomplex scalar double") }));
        // }

        // if (inputs[6].getType() != matlab::data::ArrayType::DOUBLE ||
        //     inputs[6].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
        //     matlabPtr->feval(u"error", 
        //         0, std::vector<matlab::data::Array>({ factory.createScalar("Input matrix must be type double") }));
        // }

        // if (inputs[6].getDimensions().size() != 2) {
        //     matlabPtr->feval(u"error", 
        //         0, std::vector<matlab::data::Array>({ factory.createScalar("Input must be m-by-n dimension") }));
        // }
    }
};