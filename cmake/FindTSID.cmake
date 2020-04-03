# Searches for TSID includes and library files
#cmake module modified from rbdl.cmake from opensot_superbuild

SET (TSID_FOUND FALSE)
FIND_PATH (TSID_INCLUDE_DIR tsid/config.hh
	HINTS
	$ENV{RF_LIBRARY_PATH}/include
	$ENV{HOME}/local/include
	$ENV{TSID_PATH}/include
	$ENV{TSID_INCLUDE_PATH}
	/usr/local/include
	/usr/include
	/opt/openrobots/include
	)

FIND_LIBRARY (TSID_LIBRARY NAMES tsid
	PATHS
	$ENV{RF_LIBRARY_PATH}/lib
	$ENV{HOME}/local/lib
	$ENV{HOME}/local/lib/x86_64-linux-gnu
	$ENV{TSID_PATH}/lib
	$ENV{TSID_LIBRARY_PATH}
	/usr/local/lib
	/usr/local/lib/x86_64-linux-gnu
	/usr/lib
	/usr/lib/x86_64-linux-gnu
	/opt/openrobots/lib
)


IF (NOT TSID_LIBRARY)
	MESSAGE (ERROR "Could not find TSID")
ENDIF (NOT TSID_LIBRARY)

IF (TSID_INCLUDE_DIR AND TSID_LIBRARY)
	SET (TSID_FOUND TRUE)
ENDIF (TSID_INCLUDE_DIR AND TSID_LIBRARY)


IF (TSID_FOUND)
   IF (NOT TSID_FIND_QUIETLY)
      MESSAGE(STATUS "Found TSID: ${TSID_LIBRARY}")
   ENDIF (NOT TSID_FIND_QUIETLY)

ELSE (TSID_FOUND)
   IF (TSID_FIND_REQUIRED)
		 MESSAGE(SEND_ERROR "Could not find TSID")
   ENDIF (TSID_FIND_REQUIRED)
ENDIF (TSID_FOUND)

MARK_AS_ADVANCED (
	TSID_INCLUDE_DIR
	TSID_LIBRARY
)
