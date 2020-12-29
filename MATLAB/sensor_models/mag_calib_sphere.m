%magnetometer characterization by MTR
%goal: get mag calibration sphere

%% extract data
clc; clear all; close all; format compact
for j = 1:2
    if j == 1
        filename = 'mtrlogsleader';
    else
         filename = 'mtrlogsfollower';
    end
    D = readtable([filename '.csv']);
    D = table2cell(D);
    mtr_cmd = []; %mag command; Am^2
    mag1 = []; %x,y,z Teslas
    mag2 = []; %x,y,z Teslas
    c = 1; %counter
    for i = 1:length(D)-3
        if strcmp(D{i,2},'adcs_cmd.mtr_cmd') == 1 && ...
                strcmp(D{i+1,2},'adcs_monitor.mag1_vec') == 1 && ...
                strcmp(D{i+2,2},'adcs_monitor.mag2_vec') == 1
         
            mtrcmd(c,1) = D{i,3};
            mtrcmd(c,2) = D{i,4};
            mtrcmd(c,3) = D{i,5};
            
            mag1(c,1) = D{i+1,3};
            mag1(c,2) = D{i+1,4};
            mag1(c,3) = D{i+1,5};

            mag2(c,1) = D{i+2,3};
            mag2(c,2) = D{i+2,4};
            mag2(c,3) = D{i+2,5};
            
            c = c + 1;
        end
    end
    save([filename '.mat'])
end

%% effect of each MTR against each mag axis
%36 TOTAL PLOTS GENERATED: (2 sats)(2 mags)(3 MTRs)(3 mag axes)
clc; clear all; close all;
for sat = 1:2
    if sat == 1
       satname = 'leader';
    else
       satname = 'follower';
    end
    load(['mtrlogs' satname '.mat'])
    
    for m = 1:2
        if m == 1
            magname = 'mag1';
            mag = mag1;
        else
            magname = 'mag2';
            mag = mag2;
        end

        %%%%effect of xMTR on mag axes
        rampx = unique(mtrcmd(:,1)); %ramping xMTR
        submagx = []; submtrcmdx = []; countx = 1;
        
        for r = 1:length(rampx)
            for i = 1:length(mag)
                if mtrcmd(i,1) == rampx(r) && mtrcmd(i,2) == 0 && mtrcmd(i,3) == 0 %isolate MTR
                    submtrcmdx(countx) = mtrcmd(i,1);
                    submagx(countx,1:3) = mag(i,1:3);
                    countx = countx + 1;
                end
            end
        end
        
        f_xMTR = cell(3,1); %[xMTR on mag xaxis; xMTR on mag yaxis; xMTR on mag zaxes]
        for i = 1:3
            figure()
            f = fit(submtrcmdx', submagx(:,i), 'poly2');
            plot(f,submtrcmdx',submagx(:,i));
            xlabel('mtr cmd (Am^2)'); ylabel('mag reading (T)')
            f_xMTR{i,1} = f;
            pause(0.5)
        end
        
        %%%%effect of yMTR on mag axes
        rampy = unique(mtrcmd(:,2)); %ramping yMTR
        submagy = []; submtrcmdy = []; county = 1;
        
        for r = 1:length(rampy)
            for i = 1:length(mag)
                if mtrcmd(i,1) == 0 && mtrcmd(i,2) == rampy(r) && mtrcmd(i,3) == 0
                    submtrcmdy(county) = mtrcmd(i,2);
                    submagy(county,1:3) = mag(i,1:3);
                    county = county + 1;
                end
            end
        end
        
        f_yMTR = cell(3,1); %[xMTR on mag xaxis; xMTR on mag yaxis; xMTR on mag zaxes]
        for i = 1:3
            figure()
            f = fit(submtrcmdy', submagy(:,i), 'poly2');
            plot(f,submtrcmdy',submagy(:,i));
            xlabel('mtr cmd (Am^2)'); ylabel('mag reading (T)')
            f_yMTR{i,1} = f;
            pause(0.5)
        end
        
        %%%%effect of zMTR on mag axes
        rampz = unique(mtrcmd(:,3)); %ramping zMTR
        submagz = []; submtrcmdz = []; countz = 1;
        
        for r = 1:length(rampz)
            for i = 1:length(mag)
                if mtrcmd(i,1) == 0 && mtrcmd(i,2) == 0 && mtrcmd(i,3) == rampz(r)
                    submtrcmdz(countz) = mtrcmd(i,3);
                    submagz(countz,1:3) = mag(i,1:3);
                    countz = countz + 1;
                end
            end
        end

        f_zMTR = cell(3,1); %[zMTR on mag xaxis; zMTR on mag yaxis; zMTR on mag zaxes]
        for i = 1:3
            figure()
            f = fit(submtrcmdz', submagz(:,i), 'poly2');
            plot(f,submtrcmdz',submagz(:,i));
            xlabel('mtr cmd (Am^2)'); ylabel('mag reading (T)')
            f_zMTR{i,1} = f;
            pause(0.5)
        end
        
        filename = [satname magname 'fits.mat'];
        save(filename,'f_xMTR','f_yMTR','f_zMTR',....
            'submtrcmdx','submtrcmdy','submtrcmdz','submagx','submagy','submagz')
                   
    end

end
%% ellipsoid fitting
clc;clear all; close all;

for sat = 1:2
    if sat == 1
       satname = 'leader';
    else
       satname = 'follower';
    end
    
    load(['mtrlogs' satname '.mat']) 
    for m = 1:2
        if m == 1
            %LIS2MDL mag
            mag = mag1;
            R = [1 0 0; 0 0 -1; 0 -1 0];  
        else
            %MMC3416xPJ mag
            mag = mag2; 
            R = [0 1 0; 0 0 1; -1 0 0];
        end

        %rotate sensor frame to the body frame of the sat
        mag = (R*mag')';

        xx = mag(:,1);  %in T
        yy = mag(:,2);
        zz = mag(:,3);

        %[A,b,expMFS]  = magcal([x,y,z]);

        % do the fitting
        dx=xx(:); dy=yy(:); dz=zz(:);
        n=size(dx,1);
        D=[dx.*dx, dy.*dy,  dz.*dz, 2.*dy.*dz, 2.*dx.*dz, 2.*dx.*dy, ...
                2.*dx, 2.*dy, 2.*dz, ones(n,1)]';
        S=D*D';

        [v,M,n,d]=FindFit4(S); %fitting function

        minX=min(dx);  maxX=max(dx);
        minY=min(dy);  maxY=max(dy);
        minZ=min(dz);  maxZ=max(dz);

        % draw fitting:
        nStep=100;
        stepA= (maxX-minX)/nStep; stepB= (maxY-minY)/nStep; stepC=(maxZ-minZ)/nStep;
        [x, y, z]=meshgrid(minX:stepA:maxX, minY:stepB:maxY, minZ:stepC:maxZ);
        SolidObj=v(1)*x.*x+v(2)* y.*y+v(3)*z.*z+ 2*v(4)*y.*z + 2*v(5)*x.*z + 2*v(6)*x.*y...
            + 2*v(7)*x + 2*v(8)*y + 2*v(9)*z + v(10)* ones(size(x));
        p = patch(isosurface(x,y,z,SolidObj, 0.0));
        isonormals(x,y,z,SolidObj, p);
        set(p, 'FaceColor', 'g', 'EdgeColor', 'none');
        daspect([1 1 1]);
        view(3);
        camlight ;
        lighting phong;
        hold on;
        plot3(dx, dy, dz, '.');
        xlabel('x(T)'); ylabel('y(T)'); zlabel('z(T)');
        title([satname ' mag' num2str(m)])
        saveas(gcf,[satname 'mag' num2str(m) '.png'])
        
        [XYZ,H,D,I,K] = wrldmagm(300,42.4434,76.4817,decyear(2020,11,6),'2020'); %2500-6500 nT expected magnitude/intensity;
        F = norm(XYZ)/10^9; %Teslas
        b = -inv(M)*n'; %combined bias vector
        A_inv = F*M^(0.5)/sqrt(n*inv(M)*n'-d);
        A = inv(A_inv); %combines scale factors, misalignments, soft-iron effects
        save([satname 'mag' num2str(m) '.mat'],'M','n','d','A','b') 
        close all;
    end
end

function [v,M,n,d] =FindFit4(S)
  %k=4 --> ellipsoid fit
  % Create constraint matrix C:
  C(6,6)=0; 
  C(1,1)=-1; C(2,2)=-1; C(3,3)=-1;
  C(4,4)=-4; C(5,5)=-4; C(6,6)=-4;
  C(1,2)=1; C(2,1)=1; 
  C(1,3)=1; C(3,1)=1; 
  C(2,3)=1; C(3,2)=1;
  % Solve generalized eigensystem
  S11=S(1:6, 1:6);  S12=S(1:6, 7:10);
  S22=S(7:10,7:10);
  A=S11-S12*pinv(S22)*S12';
  %[gevec, geval]=eig(A, C);
   
  CA=inv(C)*A;
  [gevec, geval]=eig(CA);
  % Find the largest eigenvalue(the only positive eigenvalue)
  In=1;
  maxVal=geval(1,1);
  for i=2:6
    if (geval(i,i)>maxVal)
        maxVal=geval(i,i);
        In=i;
    end
  end
  % Find the fitting
  v1=gevec(:, In); 
  v2=-pinv(S22)*S12'*v1;
  v=[v1; v2];
  
  %quadric surface parameters
  M = [v1(1) v1(4) v1(5); v1(4) v1(2) v1(6); v1(5) v1(6) v1(3)];
  n = [v2(1) v2(2) v2(3)];
  d = v2(4);
end