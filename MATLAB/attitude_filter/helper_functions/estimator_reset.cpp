#include "mex.hpp"
#include "mexAdapter.hpp"

#include <gnc/attitude_estimator.hpp>
#include <lin.hpp>

using namespace matlab::data;

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

    template<lin::size_t R, lin::size_t C>
    TypedArray<double> create_from_lin_mat(ArrayFactory& f, lin::Matrixd<R, C> lin_mat){
        TypedArray<double> ret = f.createArray<double>({R, C});
        for(int r = 0; r < R; r++){
            for(int c = 0; c < C; c++)
                ret[r][c] = lin_mat(r, c);
        }
        return ret;  
    }

    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        checkArguments(outputs, inputs);
        matlab::data::TypedArray<double> in = inputs[1];
        lin::Vector4f q_body_eci;
        for (int i = 0; i<4; i++) {
            q_body_eci(i) = in[i];
        }
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