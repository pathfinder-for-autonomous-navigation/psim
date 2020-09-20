#include "mex.hpp"
#include "mexAdapter.hpp"

#include <gnc/attitude_estimator.hpp>
#include <lin.hpp>

#include "conversion.hpp"

using namespace matlab::data;

class MexFunction : public matlab::mex::Function {
public:

    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        checkArguments(outputs, inputs);

        lin::Vector4f q_body_eci;
        typed_array_to_lin_vec(q_body_eci, inputs[1]);

        double time = inputs[0][0];

        gnc::AttitudeEstimatorState state = gnc::AttitudeEstimatorState();
        gnc::attitude_estimator_reset(state, time, q_body_eci);
        
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

        if (inputs.size() != 2) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("2 inputs required") }));
        }
    }
};