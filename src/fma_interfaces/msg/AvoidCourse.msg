# Explicit fixed-frame boundary and zone stations for the matching GpsRoute.
# GPS owns the centerline; this message supplies no driving path.
std_msgs/Header header
string route_id
RefPoint[] boundary
float64 entry_station_m
float64 exit_station_m
