function A = utl_area(v_body_unit)
%%% utl_area calculates the total area of satellite exposed that is normal
%%% to inputted velocity vector
%%% input: unit velocty column vector in body frame, output: area in m^2
A = dot(abs(v_body_unit),[0.03,0.03,0.01]);
end