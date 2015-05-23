# The Ibeo Message
# Ask baer@fzi.de for help
#
# The content of the message was (mainly) taken from Ibeo/Object.hpp

#
# Header with frame id, timestamp and so on
# The Timestamp is the Time of when the center point of this object was observed.
Header header

#
uint16 m_objectId
uint16  m_flags

# number of scans in which this object has been tracked, or instead time?
uint32 m_objectAge            

# Counts how long the object has not been observed but only predicted.
uint16 m_hiddenStatusAge


# The object class that is most likely for this object.
int16 Unclassified = 0               # 0: No classification determined (yet)
int16 UnknownSmall = 1               # 1: Relatively small structure like pole, tree, single bush, etc. which does not fit to other classes.
int16 UnknownBig = 2                 # 2: Bigger structure which does not fit other classes.
int16 Pedestrian = 3                 # 3: Pedestrian, usually determined by moving behaviour.
int16 Bike = 4                       # 4: bike, motor bike
int16 Car = 5                        # 5: Passenger car.
int16 Truck = 6                      # 6: Big vehicle, bus, truck.
int16 Structure_Pylon = 7            # 7: Only enhanced, special classification (staf).
int16 Structure_Beacon = 8           # 8: Only enhanced, special classification (staf).
int16 Structure_GuardRail = 9        # 9: Only enhanced, special classification (staf).
int16 Structure_ConcreteBarrier = 10 # 10: Only enhanced, special classification (staf).
int16 NumClasses = 11		   # Number of currently defined classes except Unknown.
int16 Unknown = 15		   # 15: Compatibility to earlier software which does not determine between UnknownBig or -Small or Unclassified.
int16 m_classification

# Counts how long the object has been classified in the current classification.
uint32 m_classificationAge

# The quality of the current classification.
float32 m_classificationQuality 

# Center point of object rectangle, given in Vehicle coordinate system.
float32 m_centerPoint_x      
float32 m_centerPoint_y

float32 m_centerPointSigma_x
float32 m_centerPointSigma_y

# named by ISO 8855; also called Orientation or Heading [rad]
float32 m_courseAngle

# in [rad]
float32 m_courseAngleSigma 

# Velocity of this object [meter/seconds], relative to the vehicle coordinate system.
float32 m_relativeVelocity_x
float32 m_relativeVelocity_y

float32 m_relativeVelocitySigma_x
float32 m_relativeVelocitySigma_y

# Velocity of this object [meter/seconds] as absolute velocity; the orientation is relative to the vehicle coordinate system.
float32 m_absoluteVelocity_x
float32 m_absoluteVelocity_y
float32 m_absoluteVelocitySigma_x
float32 m_absoluteVelocitySigma_y

# The object's length and width as a rectangle, relative to the object's coordinate system.
float32 m_objectBox_x
float32 m_objectBox_y

float32 m_objectBoxSigma_x
float32 m_objectBoxSigma_y

# Center of the bounding box.
float32 m_boundingBoxCenter_x 
float32 m_boundingBoxCenter_y

# A rectangle in parallel to the vehicle coordinate system (a paraxial rectangle) that contains (bounds) all of this object's points
float32 m_boundingBox_x 
float32 m_boundingBox_y

# These components are also proposed
# The point of this object that is closest to the origin of the vehicle coordinate system.
float32 m_closestPoint_x 
float32 m_closestPoint_y

# A poly-line that describes the outline of the current object measurement.
float32[] m_contourPoints_x
float32[] m_contourPoints_y

# An identifier to be used by WLAN fusion algorithms.
uint16 m_vehicleWLANid 

# The height of this object in [m] (most probably received through WLAN data).
float32 m_objectHeight 

# The standard deviation of the height of this object in [m] (most probably received through WLAN data).
float32 m_objectHeightSigma 

# The mass of this object in [kilogram] (as received e.g. through WLAN data)
float32 m_objectMass

# True, if this object is valid.
bool m_isValid
