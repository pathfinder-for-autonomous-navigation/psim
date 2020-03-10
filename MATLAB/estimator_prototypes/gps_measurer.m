function measure= gps_measurer(state,t)
%MEASURER single gps measurement
gpspossdiv=5;
gpsvelsdiv=5;
if (mod(t,90*60)<90*60/2)
    measure= state + [gpspossdiv*randn(3,1)+5;gpsvelsdiv*randn(3,1)+5];
else
    %in eclipse
    measure=nan(6,1);
end

