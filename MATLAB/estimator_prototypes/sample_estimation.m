function [estimates,states] = sample_estimation(estimator,initest,measures)
%sample_estimation Feeds the estimator the measures

states{1}=feval(initest);
s=size(measures);
for i=1:(s(2))
    [states{i+1},estimates(:,i)]=feval(estimator,states{i},measures(:,i));
end
end

