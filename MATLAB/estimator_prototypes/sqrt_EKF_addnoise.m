function [estimator,initest] = sqrt_EKF_addnoise(jac_updater,procovsqrt,mescovsqrtdiag)
%sqrt_EKF_addnoise Summary of this function goes here
%   Detailed explanation goes here
    statesize= length(procovsqrt);
    measuresize= length(mescovsqrtdiag);
    
    function initialstate= initestfn()
        %function to initialize estimator
        initialstate= struct();
        initialstate.Psqrt= nans(statesize);
        initialstate.est= nans(statesize);
    end
    function [state,estimate]=estimatorfn(state,measure)
        %function to run the estimator
        filtervalid= all(isfinite(state.Psqrt)) && ...
                     all(isreal(state.Psqrt)) && ...
                     all(isfinite(state.est)) && ...
                     all(isreal(state.est));
        measurevalid= all(isfinite(measure)) && ...
                      all(isreal(measure));        
        if filtervalid 
        	%update time
            x=state.est;
            [J,x]=feval(jac_updater,x);
            
        else
            estimate=measure;
            state.Psqrt=diag(mescovsqrtdiag);
        end
    end
    initest = @initestfn;
    estimator = @estimatorfn;
end

