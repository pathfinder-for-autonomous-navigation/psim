function out_datetime = utl_time2datetime(time,init_GPS_week_number)
%time2datetime converts time to a datetime with timezone 'UTCLeapSeconds'
%   time(double): time since init_GPS_week_number in seconds
%   init_GPS_week_number(int): initial GPS week number.
pan_epoch=seconds(init_GPS_week_number*7*24*3600)+datetime(1980,1,6,'TimeZone','UTCLeapSeconds');
out_datetime = seconds(time)+pan_epoch;
end

