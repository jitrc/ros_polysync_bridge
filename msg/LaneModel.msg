

# Header with frame id, timestamp and so on
# The Timestamp is the Time of when the center point of this object was observed.
#Header header

#
float64 m_marker_width  #meters
uint16  m_heading_angle #radians
float64 m_view_range    #meters
uint16 m_time_to_crossing #microseconds
bool m_lane_crossing
float64 m_lane_offset
float64 m_curvature
float64 m_curvature_derivative

bool m_is_valid
