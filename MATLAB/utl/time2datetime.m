function out_datetime = time2datetime(time,init_GPS_week_number)
%time2datetime converts time to a datetime with timezone 'UTCLeapSeconds'
%   time(double): time since init_GPS_week_number in seconds
%   init_GPS_week_number(int): initial GPS week number.
nano_secs_gps= uint64(init_GPS_week_number)*7*24*3600*uint64(1E9)+uint64(time*1E9);
out_datetime = datetime(nano_secs_gps,'ConvertFrom',...
'epochtime','Epoch',datetime(1980,1,6,'TimeZone','UTCLeapSeconds'),'TicksPerSecond',1E9,'TimeZone','UTCLeapSeconds');
end

