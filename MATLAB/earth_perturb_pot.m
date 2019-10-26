function R = earth_perturb_pot(r,theta,phi)
%Calculate the perturbing potential due to the first 10 degrees of EGM2008
%INPUTS
%   r     - Orbital radius (m)
%   theta - Azimuth angle (radians)
%   phi   - Zentih angle (radians)
%    
%OUTPUTS
%   R     - Value of the perturbing potential due to the first full 10
%           degrees and all orders

mu_earth = 3986004.415e8; %G*M_earth in m^3/s^2
a_earth = 6378136.3;  %Equatorial radius of the Earth in m
%this will load the input file:
data = load('EGM2008_to10_TideFree','-ASCII'); %assumes that this file is in the same folder

%data = (row #, collumn #)
% degree l/n, order m, Cbar, Sbar, C_sig, S_sig
%P = lengendreP(l,m); %degree l, order m

sum_l = 0;
for l = 2:10
    sum_m = 0;
    for m = 0:l
       
%        %lecture slides equation that doesn't use normalization
%        Clm_bar = data(sum(2:1:l)-1+m,3);
%        Slm_bar = data(sum(2:1:l)-1+m,4);
       
%        Clm_del = data(sum(2:1:l)-1+m,5);
%        Slm_del = data(sum(2:1:l)-1+m,6);
%        
%        Clm = Clm_bar*((factorial(l-m)*(2*l+1)*(2-Clm_del))/factorial(l+m))^0.5;
%        Slm = Slm_bar*((factorial(l-m)*(2*l+1)*(2-Slm_del))/factorial(l+m))^0.5;
%       
%        P = legendre(l, cos(phi)); %specify norm or not 
%        
%        sum_m = sum_m + (Clm*cos(m*theta) + Slm*sin(m*theta)) + P(m+1); 

       % document equation that uses normalization
       Clm_bar = data(sum(2:1:l)-1+m,3);
       Slm_bar = data(sum(2:1:l)-1+m,4);
       
       %Clm_sig = data(sum(2:1:l)-1+m,5);
       %Slm_sig = data(sum(2:1:l)-1+m,6);
       
       %Clm = Clm_bar (;
       %Slm = Slm_bar;
      
       P = legendre(l, cos(phi), 'norm'); 
       sum_m = sum_m + (a_earth/r)^l*(Clm_bar*cos(m*theta) + Slm_bar*sin(m*theta))*P(m+1);
      
    end
    %sum_l = sum_l + sum_m; %lecture equation
    sum_l = sum_l + sum_m; %doc equation
end

R = mu_earth*r^-1*(sum_l);

end