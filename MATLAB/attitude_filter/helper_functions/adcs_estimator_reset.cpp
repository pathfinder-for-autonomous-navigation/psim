#include "mex.hpp"
#include "mexAdapter.hpp"

#include <gnc/attitude_estimator.hpp>
#include <lin.hpp>

class MexFunction : public matlab::mex::Function {
public:

    using namespace matlab::data;

    template<size_t N>
    ArrayStruct<double> create_from_lin_vec(lin::Vector<double, N> lin_vec){
        ArrayStruct<double> ret = f.createBuffer<double>({N, 1});
        for(int r = 0; r < N; r++){
            ret[r][1] = lin_vec(r, 1);
        }
    }

    template<size_t R, size_t C>
    ArrayStruct<double> create_from_lin_mat(lin::Matrixd<R, C> lin_mat){
        ArrayStruct<double> ret = f.createBuffer<double>({R, C});
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

        // S[0]["x_bar"] = f.createArrayFromBuffer<double>({1,6}, &(state.x_bar(0)), MemoryLayout::ROW_MAJOR)
        // S[0]["sigmas"] = f.createArrayFromBuffer<double>({1,6}, &(state.x_bar(0)), MemoryLayout::ROW_MAJOR)
      	// std::for_each(data.begin(), data.end(), [&](const double& e) { *(dataPtr++) = e; });        
        // auto data_p = factory.createBuffer<double>(nnz);

        // S[0]["x_bar"] = f.createBuffer<double>({6,1});
        // for(int r = 0; i < 6; r++){
        //     S[0]["x_bar"][r][0] = state.x_bar(r);
        // }

        // S[0]["z_bar"] = f.createBuffer<double>({5,1});
        // for(int r = 0; r < 5; r++){
        //     S[0]["z_bar"][r][0] = state.z_bar(r);
        // }

        // S[0]["P_bar"] = f.createBuffer<double>({6, 6});
        // for(int r = 0; i < 6; r++){
        //     for(int c = 0; i < 6; c++){
        //         S[0]["P_bar"][r][c] = state.P_bar(r, c);
        //     }
        // }        

        // S[0]["P_vv"] = f.createBuffer<double>({5, 5});
        // for(int r = 0; i < 5; r++){
        //     for(int c = 0; i < 5; c++){
        //         S[0]["P_vv"][r][c] = state.P_vv(r, c);
        //     }
        // }   

        // S[0]["P_xy"] = f.createBuffer<double>({6, 5});
        // for(int r = 0; i < 6; r++){
        //     for(int c = 0; i < 5; c++){
        //         S[0]["P_xy"][r][c] = state.P_vv(r, c);
        //     }
        // }

        // S[0]["P"] = f.createBuffer<double>({6, 6});
        // for(int r = 0; i < 6; r++){
        //     for(int c = 0; i < 6; c++){
        //         S[0]["P"][r][c] = state.P(r, c);
        //     }
        // }

        S[0]["x_bar"] = create_from_lin_vec(state.x_bar);
        S[0]["z_bar"] = create_from_lin_vec(state.z_bar);
        
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