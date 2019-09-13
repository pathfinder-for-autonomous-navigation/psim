function time = utl_datetime2time(in_datetime,init_GPS_week_number)
%DATETIME2TIME converts in_datetime to time(s) since init_GPS_week_number
%   in_datetime(datetime timezone 'UTCLeapSeconds'): input datetime
%   init_GPS_week_number(int): initial GPS week number.
assert(strcmp(in_datetime.TimeZone,'UTCLeapSeconds'),'TimeZone must be ''UTCLeapSeconds''') 
time = seconds(in_datetime-utl_time2datetime(0,init_GPS_week_number));
end

