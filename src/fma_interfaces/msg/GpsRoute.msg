# Full active GPS route in a fixed ENU frame. Static, reliable, transient-local.
# Geometry changes invalidate any avoidance plan; never reconstruct it from a preview.
std_msgs/Header header
string route_id
RouteFeedback route
RefPoint[] points
