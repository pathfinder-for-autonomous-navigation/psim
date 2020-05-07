#include "mex.hpp"
#include "mexAdapter.hpp"

#include <gnc/attitude_estimator.hpp>
#include <lin.hpp>

class MexFunction : public matlab::mex::Function {
public:

    using namespace matlab::data;

    template<size_t N>
    ArrayStruct<double> create_from_lin_vec(lin::Vector<double, N> lin_vec){
        ArrayStruct<double> ret = f.createArray<double>({N, 1});
        for(int r = 0; r < N; r++){
            ret[r][1] = lin_vec(r, 1);
        }
    }

    ArrayStruct<double> create_from_lin_vec_array(lin::Vector<double, N>* lin_vec, size_t L){
        ArrayStruct<double> ret = f.createArray<double>({N, 1, L});
        for(int i = 0; i < L; i++){
            for(int r = 0; r < N; r++){
                ret[r][1][i] = (lin_vec[i])(r, 1);
            }
        }
    }

    template<size_t R, size_t C>
    ArrayStruct<double> create_from_lin_mat(lin::Matrixd<R, C> lin_mat){
        ArrayStruct<double> ret = f.createArray<double>({R, C});
        for(int r = 0; r < R; r++){
            for(int c = 0; c < C; c++)
                ret[r][c] = lin_mat(r, c);
        }
    }

    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        checkArguments(outputs, inputs);
        matlab::data::TypedArray<double> in = std::move(inputs[1]);
        lin::Vector3f q_body_eci;
        for (int i = 0; i<3; i++) {
            q_body_eci(i) = in[i];
        }
        double time = inputs[0][0];

        gnc::AttitudeEstimatorState state = gnc::AttitudeEstimatorState();
        gnc::attitude_estimator_reset(state, q_body_eci, time);
        
        ArrayFactor f;
        // will only contain one element
        StructArray S = f.createStructArray({1,1}, 
            {"x_bar", "sigmas", "z_bar", "measures", "P_bar", "P_vv", "P_xy", "q", "x","P","t"});

        S[0]["x_bar"] = create_from_lin_vec(state.x_bar);
        S[0]["z_bar"] = create_from_lin_vec(state.z_bar);
        
        // S[0]["sigmas"] = f.createArray<double>({6, 1, 13});
        // for(int i = 0; i<13; i++){
        //     S[0]["sigmas"][i] = create_from_lin_vec(state.sigmas[i]);
        // }
        S[0]["sigmas"] = create_from_lin_vec_arr(state.sigmas, 13);
        S[0]["measures"] = create_from_lin_vec_arr(state.measures, 13);

        S[0]["P_bar"] = create_from_lin_mat(state.P_bar);
        S[0]["P_vv"] = create_from_lin_mat(state.P_vv);
        S[0]["P_xy"] = create_from_lin_mat(state.P_xy);

        S[0]["q"] = create_from_lin_vec(state.q);
        S[0]["x"] = create_from_lin_vec(state.x);
        S[0]["P"] = create_from_lin_mat(state.P);

        S[0]["t"] = state.t;

        //outputs[0] is the struct
        outputs[0] = S;
    }

    // void adcs_estimator_reset(lin::Vector3f q_body_eci, double time) {
        
    //     gnc::attitude_estimator_reset(state, q_body_eci, time);

    // }

    void checkArguments(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();
        matlab::data::ArrayFactory factory;

        // assuming t, then q

        if (inputs.size() != 2) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("2 inputs required") }));
        }

        // if (inputs[0].getNumberOfElements() != 1) {
        //     matlabPtr->feval(u"error", 
        //         0, std::vector<matlab::data::Array>({ factory.createScalar("Input multiplier must be a scalar") }));
        // }
        
        // if (inputs[0].getType() != matlab::data::ArrayType::DOUBLE ||
        //     inputs[0].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
        //     matlabPtr->feval(u"error", 
        //         0, std::vector<matlab::data::Array>({ factory.createScalar("Input multiplier must be a noncomplex scalar double") }));
        // }

        // if (inputs[1].getType() != matlab::data::ArrayType::DOUBLE ||
        //     inputs[1].getType() == matlab::data::ArrayType::COMPLEX_DOUBLE) {
        //     matlabPtr->feval(u"error", 
        //         0, std::vector<matlab::data::Array>({ factory.createScalar("Input matrix must be type double") }));
        // }

        // if (inputs[1].getDimensions().size() != 2) {
        //     matlabPtr->feval(u"error", 
        //         0, std::vector<matlab::data::Array>({ factory.createScalar("Input must be m-by-n dimension") }));
        // }
    }
};