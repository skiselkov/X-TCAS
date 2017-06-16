# CDDL HEADER START
#
# This file and its contents are supplied under the terms of the
# Common Development and Distribution License ("CDDL"), version 1.0.
# You may only use this file in accordance with the terms of version
# 1.0 of the CDDL.
#
# A full copy of the text of the CDDL should have accompanied this
# source.  A copy of the CDDL is also available via the Internet at
# http://www.illumos.org/license/CDDL.
#
# CDDL HEADER END

# Copyright 2017 Saso Kiselkov. All rights reserved.

cmake_minimum_required(VERSION 2.8)
project(xtcas C)

SET(SRC SL.c avl.c geom.c helpers.c list.c log.c math.c pos.c
    xtcas.c thread.c time.c wav.c snd_sys.c riff.c)
SET(HDR SL.h avl.h avl_impl.h geom.h helpers.h log.h list.h list_impl.h
    math.h pos.h types.h xtcas.h time.h thread.h wav.h snd_sys.h
    riff.h)

SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELEASE} -DDEBUG")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DDEBUG")

if("${TEST_STANDALONE_BUILD}" EQUAL "1")
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DTEST_STANDALONE_BUILD")
	LIST(APPEND SRC test.c)
	LIST(APPEND HDR test.h)
else()
	LIST(APPEND SRC xplane.c)
	LIST(APPEND HDR xplane.h)
endif()

SET(ALL_SRC ${SRC} ${HDR})
LIST(SORT ALL_SRC)

if(APPLE)
	add_executable(xtcas ${ALL_SRC})
elseif(UNIX)
	if("${TEST_STANDALONE_BUILD}" EQUAL "1")
		add_executable(xtcas ${ALL_SRC})
	else()
		add_library(xtcas SHARED ${ALL_SRC})
	endif()
elseif(WIN32)
	if("${TEST_STANDALONE_BUILD}" EQUAL "1")
		add_executable(xtcas ${ALL_SRC})
	else()
		add_library(xtcas MODULE ${ALL_SRC})
	endif()
	SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MD")
	SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELEASE} /MD")
	SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MDd")
endif()

#compiler flags
if(WIN32)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DCHECK_RESULT_USED=_Check_return_")
else()
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -W -Wall -Wextra -Werror \
	    --std=c99 \
	    -DCHECK_RESULT_USED=\"__attribute__ ((warn_unused_result))\"")
endif()

#libraries
if(UNIX)
	include_directories(xtcas PUBLIC "../OpenAL/include")
	if(NOT "${TEST_STANDALONE_BUILD}" EQUAL "1")
		include_directories(xtcas PUBLIC "../SDK/CHeaders/XPLM"
		    "../SDK/CHeaders/Widgets" "../SDK/CHeaders/Wrappers")
	endif()
else()
	include_directories(xtcas PUBLIC "..\\OpenAL\\include")
	if(NOT "${TEST_STANDALONE_BUILD}" EQUAL "1")
		include_directories(xtcas PUBLIC "..\\SDK\\CHeaders\\XPLM"
		    "..\\SDK\\CHeaders\\Widgets" "..\\SDK\\CHeaders\\Wrappers")
	endif()
endif(UNIX)
if(APPLE)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -arch x86_64 -Wno-deprecated")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mmacosx-version-min=10.7")
endif(APPLE)
if(WIN32)
	target_include_directories(xtcas PUBLIC "..\\SDK\\CHeaders\\XPLM"
	    "..\\SDK\\CHeaders\\Widgets" "..\\SDK\\CHeaders\\Wrappers")
endif(WIN32)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DXPLM200=1 -DXPLM210=1")
if(APPLE)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DAPL=1 -DIBM=0 -DLIN=0")
elseif(UNIX)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DAPL=0 -DIBM=0 -DLIN=1")
elseif(WIN32)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DAPL=0 -DIBM=1 -DLIN=0 \
	    -D_CRT_SECURE_NO_WARNINGS /bigobj")
endif()

#linking
link_libraries(m)
if(APPLE)
	set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} ../SDK/Libraries/Mac/)
	FIND_LIBRARY(XPLM_LIBRARY XPLM ../SDK/Libraries/Mac/)
	FIND_LIBRARY(XPWIDGETS_LIBRARY XPWidgets ../SDK/Libraries/Mac/)
	FIND_LIBRARY(OPENAL_LIBRARY OpenAL)
	FIND_LIBRARY(CURSES_LIBRARY ncurses)
	if("${TEST_STANDALONE_BUILD}" EQUAL "1")
		target_link_libraries(xtcas ${OPENAL_LIBRARY}
		    ${CURSES_LIBRARY} "pthread")
	else()
		target_link_libraries(xtcas ${OPENAL_LIBRARY}
		    ${XPLM_LIBRARY} ${XPWIDGETS_LIBRARY})
	endif()
elseif(UNIX)
	INCLUDE(FindPkgConfig)
	pkg_check_modules(UDEV REQUIRED libudev)
	include_directories(${UDEV_INCLUDE_DIRS})
	link_directories(${UDEV_LIBRARY_DIRS})
	target_link_libraries(xtcas ${UDEV_LIBRARIES})
elseif(WIN32)
	set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH}
	    "..\\SDK\\Libraries\\Win\\")
	FIND_LIBRARY(XPLM_LIBRARY XPLM_64
	    "..\\SDK\\Libraries\\Win\\")
	FIND_LIBRARY(XPWIDGETS_LIBRARY XPWidgets_64
	    "..\\SDK\\Libraries\\Win\\")
	if("${TEST_STANDALONE_BUILD}" EQUAL "1")
		target_link_libraries(xtcas ${OPENAL_LIBRARY})
	else()
		target_link_libraries(xtcas ${OPENAL_LIBRARY}
		    ${XPLM_LIBRARY} ${XPWIDGETS_LIBRARY})
	endif()
endif()

SET_TARGET_PROPERTIES(xtcas PROPERTIES PREFIX "")
SET_TARGET_PROPERTIES(xtcas PROPERTIES SUFFIX "")

if("${TEST_STANDALONE_BUILD}" EQUAL "1")
	set(OUTPUT_FILENAME "xtcas")
else()
	if(APPLE)
		set(OUTPUT_FILENAME "mac.xpl")
		SET_TARGET_PROPERTIES(xtcas PROPERTIES LINK_FLAGS
		    "${CMAKE_SHARED_LINKER_FLAGS} -fPIC -fvisibility=hidden \
		    -bundle")
	elseif(UNIX)
		set(OUTPUT_FILENAME "lin.xpl")
		SET_TARGET_PROPERTIES(xtcas PROPERTIES LINK_FLAGS
		    "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic -nodefaultlibs \
		    -undefined_warning -fPIC -fvisibility=hidden")
	elseif(WIN32)
		set(OUTPUT_FILENAME "win.xpl")
	endif()
endif()

set_target_properties(xtcas PROPERTIES RUNTIME_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/bin" )
set_target_properties(xtcas PROPERTIES LIBRARY_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/bin" )
set_target_properties(xtcas PROPERTIES OUTPUT_NAME "${OUTPUT_FILENAME}")
