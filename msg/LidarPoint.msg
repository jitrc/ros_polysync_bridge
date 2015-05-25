
# The layer kind of lidar point.
int16 Layer_None = 0              
int16 Layer_0 = 1              
int16 Layer_1 = 2                
int16 Layer_2 = 3                 
int16 Layer_3 = 4                      
int16 Layer_4 = 5                    
int16 Layer_5 = 6                    
int16 Layer_6 = 7          
int16 Layer_7 = 8           
int16 m_layer_kind

# The echo kind of lidar point.
int16 Echo_None = 0              
int16 Echo_0 = 1              
int16 Echo_1 = 2                
int16 Echo_2 = 3                 
int16 Echo_3 = 4                      
int16 Echo_4 = 5                    
int16 Echo_5 = 6                    
int16 Echo_6 = 7          
int16 Echo_7 = 8           
int16 m_echo_kind

# The point kind of lidar point.
int16 None = 0
int16 Unknown = 1
int16 Transparent = 2
int16 Clutter = 3 
int16 Ground = 4
int16 m_point_kind

float64 m_echo_pulse_width

float64 m_position_x
float64 m_position_y
float64 m_position_z

