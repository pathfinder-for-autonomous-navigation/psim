rng(100);
P= eye(3)*1E4;
B_bias_est= [0;0;0;];
B_bias_true= randn(3,1);
N=10;
error=zeros(N,1);
for i= 1:N
    S= randn(1,3);
    S= S/norm(S);
    B_true= randn(3,1)*0.5;
    B_measured= B_true+B_bias_true+0.02*randn(3,1);
    SdotB_true= dot(B_true,S);
    SdotB_measured= dot(B_measured,S);
    SdotB_bias= SdotB_measured-SdotB_true;
    g=P*S'/(1+S*P*S');
    P= P-g*S*P;
    B_bias_est= B_bias_est + g*(SdotB_bias-dot(S,B_bias_est));
    %P=P-P*S'*S*P/(1+S*P*S');
    %B_bias_est= B_bias_est + P*S'*(SdotB_bias-dot(S,B_bias_est));
    error(i)=norm(B_bias_est-B_bias_true)
end

plot(error)