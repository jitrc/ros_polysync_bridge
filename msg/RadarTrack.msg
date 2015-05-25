# Header with frame id, timestamp and so on. Timestamp of track
Header header

uint16 m_objectId

# The track status kind of radar track
int16 NOT_ACTIVE = 0
int16 ACTIVE = 1
int16 CIPV = 2
int16 NEW = 3
int16 NEW_UPDATED = 4
int16 UPDATED = 5
int16 COASTED = 6
int16 MERGED = 7
int16 INVALID_COASTED = 8
int16 NEW_COASTED = 9
int16 TARGET_VALIDITY0 = 10
int16 TARGET_VALIDITY1 = 11
int16 TARGET_VALIDITY2 = 12
int16 TARGET_VALIDITY3 = 13
int16 TARGET_VALIDITY4 = 14
int16 TARGET_VALIDITY5 = 15
int16 TARGET_VALIDITY6 = 16           
int16 m_track_status


float64 m_position_x
float64 m_position_y
float64 m_position_z

float64 m_velocity_x
float64 m_velocity_y
float64 m_velocity_z

float64 m_amplitude # track amp in db

# The range mode kind for radar
int16 NotKnown = 0              
int16 Short = 1              
int16 Medium = 2                
int16 Long = 3                 
int16 m_range_mode

uint64 m_scan_index
