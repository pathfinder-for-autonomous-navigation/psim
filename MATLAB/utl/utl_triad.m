function q_triad = utl_triad(N_sun, N_mag, B_sun, B_mag)
%Based on time and the location of the spacecraft, the sun and magnetic 
%field vectors in the reference frame N are provided. Based on the sun
%sensor and magnetometer readings, the sun and magnetic field vectors are
%provided in the spacecraft's body frame B. This TRIAD algorithm is very
%straightforward and will be used on the spacecraft.
%B/c the algorithm has the orientation of the same vectors in
%different reference frame, it is able to compute the rotation between
%those reference frames. The algorithm will fail if either of the reference
%vectors or observed vectors are parallel or anti parallel.

% Normalize all vectors (should be normalized already)
N_sun=N_sun/norm(N_sun);
B_sun=B_sun/norm(B_sun);
N_mag=N_mag/norm(N_mag);
B_mag=B_mag/norm(B_mag);

% Ensure all vectors have three components and vectors in the same frame
% are not parallel
if size(N_sun)~=[3 1]
    error('N_sun must be a 3-component column vector')
end

if size(B_sun)~=[3 1]
    error('B_sun must be a 3-component column vector')
end

if size(N_mag)~=[3 1]
    error('N_mag must be a 3-component column vector')
end

if size(B_mag)~=[3 1]
    error('B_mag must be a 3-component column vector')
end

if dot(N_sun,N_mag)==1
    error('Reference frame N vectors can not be parallel')
end

if dot(B_sun,B_mag)==1
    error('Body frame B vectors can not be parallel')
end

% Compute orthonogoal right-handed vectors of reference frame
v1=N_sun;
v2=cross(N_sun,N_mag)/norm(cross(N_sun,N_mag));
v3=cross(v1,v2);

% Compute orthonogoal right-handed vectors of body frame
w1=B_sun;
w2=cross(B_sun,B_mag)/norm(cross(B_sun,B_mag));
w3=cross(w1,w2);

DCM_be=w1(:)*v1(:)'+w3(:)*v3(:)'+w2(:)*v2(:)';
%tr= DCM_be(1,1) + DCM_be(2,2) + DCM_be(3,3);
%th= acos((tr-1)/2);
%axis= 0.5/sin(th)*[DCM_be(2,3)-DCM_be(3,2);DCM_be(3,1)-DCM_be(1,3);DCM_be(1,2)-DCM_be(2,1)];
%q_triad= [axis*sin(th/2);cos(th/2)];
q_triad=utl_dcm2quat(DCM_be);
end

