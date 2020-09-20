#include "mex.hpp"
#include "mexAdapter.hpp"

#include <gnc/attitude_estimator.hpp>
#include <lin.hpp>

#include "conversion.hpp"

class MexFunction : public matlab::mex::Function {
public:

    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        checkArguments(outputs, inputs);

        // initial lin state
        gnc::AttitudeEstimatorState state = gnc::AttitudeEstimatorState();

        // make lin input vectors
        lin::Vector3f r_ecef;
        lin::Vector3f b_body;
        lin::Vector3f s_body;

        // dump function inputs to reset inputs
        double time = inputs[0][0];
        typed_array_to_lin_vec(r_ecef, inputs[1]);
        typed_array_to_lin_vec(b_body, inputs[2]);
        typed_array_to_lin_vec(s_body, inputs[3]);

        gnc::attitude_estimator_reset(state, time, r_ecef, b_body, s_body);

        ArrayFactory f;

        // will only contain one element
        StructArray S = f.createStructArray({1,1}, 
            {"q", "x","P","t"});

        // build output
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
                0, std::vector<matlab::data::Array>({ factory.createScalar("4 inputs required") }));
        }

    }
};