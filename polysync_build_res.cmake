#add_definitions( -std=c++11 )

set( CMAKE_CXX_FLAGS "-DPSYNC_CORE_CAN_API")

set( PSYNC_HOME
    
    /usr/local/polysync
)

link_directories(

    ${PSYNC_HOME}/lib
)


set( PSYNC_INCLUDE_DIRS

    ${PSYNC_HOME}/include
    ${PSYNC_HOME}/include/deps
    ${PSYNC_HOME}/include/deps/dcps/C/SAC
    ${PSYNC_HOME}/include/deps/sys
    /usr/include/glib-2.0
    /usr/lib/x86_64-linux-gnu/glib-2.0/include
)

set( PSYNC_LIBS
    
    #polysync++
    polysync
    xstypes
    avcodec
    avutil
    swscale
    avformat
    canlib
    dl
    nsl
    m
    pthread
    rt
    sqlite3
    crypto
    gmodule-2.0
    glib-2.0
    gthread-2.0
    dcpssac
    dcpsgapi
    xml2
    meschach
    ddsuser
    ddskernel
    ddsserialization
    ddsconfparser
    ddsconf
    ddsdatabase
    ddsutil
    ddsos
)
