#include "mex.hpp"
#include "mexAdapter.hpp"

#include <gnc/attitude_estimator.hpp>
#include <lin.hpp>

class MexFunction : public matlab::mex::Function {
public:

    template<lin::size_t N, typename T>
    TypedArray<T> create_from_lin_vec(ArrayFactory& f, lin::Vector<T, N> lin_vec){
        TypedArray<T> ret = f.createArray<T>({N, 1});
        for(int r = 0; r < N; r++){
            ret[r][0] = lin_vec(r);
        }
        return ret;
    }
    
    template<lin::size_t N, typename T>
    TypedArray<T> create_from_lin_vec_arr(ArrayFactory& f, lin::Vector<T, N>* lin_vec, size_t L){
        TypedArray<T> ret = f.createArray<T>({N, 1, L});
        for(int i = 0; i < L; i++){
            for(int r = 0; r < N; r++){
                ret[r][0][i] = (lin_vec[i])(r);
            }
        }
        return ret;    
    }

    template<lin::size_t R, lin::size_t C, typename T>
    TypedArray<double> create_from_lin_mat(ArrayFactory& f, lin::Matrix<T, R, C> lin_mat){
        TypedArray<double> ret = f.createArray<double>({R, C});
        for(int r = 0; r < R; r++){
            for(int c = 0; c < C; c++)
                ret[r][c] = lin_mat(r, c);
        }
        return ret;  
    }

    template<lin::size_t N, typename T>
    lin::Vector<T, N> typed_array_to_lin_vec(matlab::data::TypedArray<T> typed_arr){
        lin::Vector<T, N> ret;
        for (int i = 0; i<N; i++) {
            ret(i) = typed_arr[i];
        }
        return ret;
    }

    template<lin::size_t N, typename T>
    lin::Vector<T, N> typed_array_to_lin_mat(matlab::data::TypedArray<T> typed_arr, size_t R, size_t C){
        lin::Matrix<T, R, C> ret;
        for( int r = 0; r<R; r++){
            for( int c = 0; c<C; c++){
                ret(r, c) = typed_arr[r][c];
            }
        }
        return ret;
    }

    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        checkArguments(outputs, inputs);
        
        // the struct
        StructArray matlab_state = inputs[0][0];
        
        // time
        // double t = inputs[1][0];
        
        // assemble lin data
        gnc::AttitudeEstimatorData data_in = gnc::AttitudeEstimatorData();
        data_in.t = inputs[1][0];
        data_in.r_ecef = typed_array_to_lin_vec(inputs[2]);
        data_in.b_body = typed_array_to_lin_vec(inputs[3]);
        data_in.s_body = typed_array_to_lin_vec(inputs[4]);
        data_in.w_body = typed_array_to_lin_vec(inputs[5]);

        // assemble lin state
        gnc::AttitudeEstimatorState state = gnc::AttitudeEstimatorState();
        state.q = typed_array_to_lin_vec(matlab_state["q"]);
        state.x = typed_array_to_lin_vec(matlab_state["x"]);
        state.P = typed_array_to_lin_mat(matlab_state["P"], 6, 6);
        state.t = matlab_state["t"]

        // empty estimate
        gnc::AttitudeEstimate estimate = gnc::AttitudeEstimate();

        gnc::attitude_estimator_update(state, data, estimate);
        
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

        if (inputs.size() != 6) {
            matlabPtr->feval(u"error", 
                0, std::vector<matlab::data::Array>({ factory.createScalar("6 inputs required") }));
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