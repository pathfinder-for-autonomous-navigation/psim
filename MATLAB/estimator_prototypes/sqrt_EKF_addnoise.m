function [estimator,initest] = sqrt_EKF_addnoise(jac_updater,SQ,messdiv)
%sqrt_EKF_addnoise Summary of this function goes here
%       [J,x]=jac_updater(x) 
%       SQ The xDimX xDim lower-triangular square root of the process
%              noise covariance matrix.
    l= length(SQ);
    function initialstate= initestfn()
        %function to initialize estimator
        initialstate= struct();
        initialstate.S= nan(l);
        initialstate.x= nan(l,1);
    end
    function [state,estimate]=estimatorfn(state,measure)
        %function to run the estimator
        filtervalid= all(isfinite(state.S),'all') && ...
                     all(isreal(state.S),'all') && ...
                     all(isfinite(state.x)) && ...
                     all(isreal(state.x));
        measurevalid= all(isfinite(measure)) && ...
                      all(isreal(measure));        
        if filtervalid 
        	%update time
            [J,state.x]=feval(jac_updater,state.x);
            [~, state.S]=sqrtDiscKalPred(state.x,state.S,J,SQ);
            if measurevalid
                %correct estimate
                [state.x, state.S,~,~,~]=sqrtKalmanUpdate(state.x,state.S,measure,diag(messdiv),eye(l));
            end
        else
            state.x=measure;
            state.S=diag(messdiv);
        end
        estimate=state.x;
    end
    initest = @initestfn;
    estimator = @estimatorfn;
end

