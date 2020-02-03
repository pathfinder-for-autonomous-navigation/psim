function pantime = utl_grace2pantime(gracetime)
global const
g_epoch = datetime(2000,01,01,11,59,47,00,'TimeZone','UTCLeapSeconds');
diff = seconds(gracetime); %default timezone is UTC
g_datetime = g_epoch + diff;
pantime = utl_datetime2time(g_datetime, const.INITGPS_WN);
end
