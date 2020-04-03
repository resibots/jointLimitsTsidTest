# Searches for pinocchio includes and library files
#cmake module modified from rbdl.cmake from opensot_superbuild

SET (PIN_FOUND FALSE)
FIND_PATH (PIN_INCLUDE_DIR pinocchio/config.hpp
	HINTS
	$ENV{RF_LIBRARY_PATH}/include
	$ENV{HOME}/local/include
	$ENV{PIN_PATH}/include
	$ENV{PIN_INCLUDE_PATH}
	/usr/local/include
	/usr/include
	/opt/openrobots/include
	)

FIND_LIBRARY (PIN_LIBRARY NAMES pinocchio
	PATHS
	$ENV{RF_LIBRARY_PATH}/lib
	$ENV{HOME}/local/lib
	$ENV{HOME}/local/lib/x86_64-linux-gnu
	$ENV{PIN_PATH}/lib
	$ENV{PIN_LIBRARY_PATH}
	/usr/local/lib
	/usr/local/lib/x86_64-linux-gnu
	/usr/lib
	/usr/lib/x86_64-linux-gnu
	/opt/openrobots/lib
)


IF (NOT PIN_LIBRARY)
	MESSAGE (ERROR "Could not find pinocchio")
ENDIF (NOT PIN_LIBRARY)

IF (PIN_INCLUDE_DIR AND PIN_LIBRARY)
	SET (PIN_FOUND TRUE)
ENDIF (PIN_INCLUDE_DIR AND PIN_LIBRARY)


IF (PIN_FOUND)
   IF (NOT PIN_FIND_QUIETLY)
      MESSAGE(STATUS "Found pinocchio: ${PIN_LIBRARY}")
   ENDIF (NOT PIN_FIND_QUIETLY)

ELSE (PIN_FOUND)
   IF (PIN_FIND_REQUIRED)
		 MESSAGE(SEND_ERROR "Could not find pinocchio")
   ENDIF (PIN_FIND_REQUIRED)
ENDIF (PIN_FOUND)

MARK_AS_ADVANCED (
	PIN_INCLUDE_DIR
	PIN_LIBRARY
)
