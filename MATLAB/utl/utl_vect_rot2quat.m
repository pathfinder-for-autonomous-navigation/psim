function qout = utl_vect_rot2quat(r1,r2)
%UTL_VECT_ROT2QUAT Returns a unit quaternion that will rotate r2 to r1 with minimal angle.
%   If r1 and r2 are very close to anti parallel, [0;0;1;0] is default.
%     Quaternions have the forth component as scalar.
%     r1 and r2 must unit vectors or have NaN or +- inf
    qout= zeros(4,1);
    qout(4)= sqrt((1.0+dot(r1,r2))/2.0);
    qout(1:3)= cross(r1,r2);
    %check for 180 degree rotation
    if (qout(4)<=0.00001)
        % rotate around z by default
        qout=[0;0;1;0;];
    else
        qout(1:3)=qout(1:3)*1.0/(2.0*qout(4));
    end
end

