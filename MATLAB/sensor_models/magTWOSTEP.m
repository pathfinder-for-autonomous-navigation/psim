clc
clear all
close all
data = csvread('total.csv',10,0); %%%starts reading from row 10
tic 

data(:,8) = data(:,8)/(2*pi); %wheel 0 on [Hz]
data(:,9) = data(:,9)/(2*pi);
data(:,10) = data(:,10)/(2*pi);

data(:,8) = data(:,10)/(2*pi); %wheel 1 on [Hz]
data(:,9) = data(:,20)/(2*pi);
data(:,10) = data(:,21)/(2*pi);

data(:,8) = data(:,30)/(2*pi); %wheel 2 on [Hz]
data(:,9) = data(:,31)/(2*pi);
data(:,10) = data(:,32)/(2*pi);

%magnetometer reading
data = csvread('lis2outsidemovingtests.csv',1,1); 
time2 = data(:,1); 
%pd2 = fitdist(reading2, 'normal');
%y2 = pdf(pd2, min(reading2):max(reading2));

N = length(data);
B = [data(:,2) data(:,3) data(:,4)];
%B = B(1,:);

sig = var(B);
mu = mean(B);

%xnew = 1;
%tol = 0.001;
[mag_field_vector,hor_intensity,declinatioon,inclination,total_intensity] ...
              = igrfmagm(0,42.4440,-76.5019,decyear(2019,2,3),12);
mag_field_vector = 1E-7.*mag_field_vector; %convert to mG
%%%%%%% convert time column into minute and seconds for mag_field value at
%%%%%%% EACH TIME

R = ones(length(B),3);
R(:,1) = mag_field_vector(1)*R(:,1);
R(:,2) = mag_field_vector(2)*R(:,2);
R(:,3) = mag_field_vector(3)*R(:,3);

y = zeros(length(B),1);
L = zeros(length(B),2); 
Ltild(k) = zeros(length(B),2);
ztild(k) = zeros(length(B),2);
mutild(k) = zeros(length(B),2);
for k = 1:length(B)
    %y(k) = norm(B(k,:))^2 - norm(R(k,:))^2;
    y(k) = norm(B(k,:))^2 - norm(R(k,:))^2;
end

xtrue = zeros(length(B),1);
tol = 0.001;
for k = 1:N
    xnew = 1; 
    xold = 0.01;
    while abs(xnew - xold) > tol
        xold = xnew;
        fun1 = (sig^-2)*(L'*L);
        P = (symsum(fun1,n,1,N)^-1)';
        fun2 = (1/sig(n)^2)*(y(n) + norm(b*xold)^2 ...
            -mu(n))*L(n)';
        xnew = P*symsum(fun2,n,1,N)^-1;
    end
    xtrue(k) = xnew;
end

for
        L(k) = [2*B(k,:)' -S(k,:)];
        sigbar = sqrt(1/symsum((1/sig(k)^2),k,1,N));
        Lbar = sigbar^2*symsum(((1/sig(k)^2)*L(k)),k,1,N);
        zbar = sigbar^2*symsum(((1/sig(k)^2)*y(k)),k,1,N);
        mubar = sigbar^2*symsum(((1/sig(k)^2)*mu(k)),k,1,N);
        Ltild(k) = L(k) - Lbar;
        ztild(k) = z(k) - zbar;
        mutild(k) = mutild(k) - mubar;

        P = symsum(((1/sig(k)^2)*(Ltild(k)'*Ltild(k))), k,1,N)';
        theta = P*symsum(((1/sig(k)^2)*(ztild(k)-mutild(k))*Ltild(k)'), k,1,N);
    end
end


%y = -B'*(2*D + D^2)*B + 2*B'*(I3 + D)*b - norm(b)^2 + v;
%v = 2*((I3 + D)*B-b)'*e - norm(e)^2;


% E_truevec = [E_truescal(1,1) E_truescal(2,2) E_truescal(3,3)...
%     E_truescal(1,2) E_truescal(1,3) E_truescal(2,3)]';

%x = [c_true'; E_truevec'];
%y = L*x - norm(b*(x))^2 + v;

% syms b D c E_truescal S y L v 
% eqns = [L -[2*B -S'] == 0, E_truescal -(2*D + D^2) ==0, ....
%     c - (eye(3) + D)*b == 0; S - [B(1)^2 B(2)^2 B(3)^2 ...
%     2*B(1) B(2) 2*B(1)...
%     B(3) 2*B(2) B(3)]' == 0, P - (sig^-2)*(L'*L) == 0, ...
%     xnew - P*(1/sig^2)*(y + norm(b*xold)^2-mu)*L' == 0];
% S = solve(eqns, [b D c E_truescal S y L v xnew]);
%end
% W = [-1 + sqrt(1 + S(1,1)) 0 0; 0 -1 + sqrt(1 + S(2,2)) 0; 0 0 -1 + sqrt(1 + S(3,3))];
% 
% D = U*W*U';
% b = (I3 + D)^-1*c;


