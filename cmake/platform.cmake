# # Check environment
if(WIN32 AND NOT CYGWIN)
    set(WINDOWS TRUE) # Windows (native)
elseif(UNIX)
    set(POSIX TRUE) # Linux, BSD, Mac OS X, ...

    if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
        set(LINUX TRUE) # GNU/Linux
    else()
        # Unkown UNIX
        message(WARNING "Unknown UNIX operating system!")
    endif()
else()
    # Unkown OS
    message(WARNING "Unknown operating system!")
endif()