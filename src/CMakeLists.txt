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

# Copyright 2025 Saso Kiselkov. All rights reserved.

cmake_minimum_required(VERSION 3.10)
project(xtcas)

option(VSI_MODE			"Enable VSI mode")
option(VSI_STYLE		"VSI drawing style")
option(GTS820_MODE		"Enable GTS820 mode")
option(AUDIO			"Enable/disable audio generation")
option(LIBACFUTILS		"libacfutils repo")
option(TEST_STANDALONE_BUILD	"Standalone build mode")

if(APPLE)
	set(PLAT_SHORT "mac64")
	set(PLAT_LONG "mac-64")
	set(PLAT_ONLY "mac")
	set(PLUGIN_BIN_OUTDIR "mac_x64")
elseif(WIN32)
	set(PLAT_SHORT "win64")
	set(PLAT_LONG "win-64")
	set(PLAT_ONLY "win")
	set(PLUGIN_BIN_OUTDIR "win_x64")
else()
	set(PLAT_SHORT "lin64")
	set(PLAT_LONG "linux-64")
	set(PLAT_ONLY "linux")
	set(PLUGIN_BIN_OUTDIR "lin_x64")
endif()

set(SRC SL.c dbg_log.c pos.c xtcas.c snd_sys.c)
set(HDR SL.h dbg_log.h pos.h xtcas.h snd_sys.h)

if(${AUDIO} STREQUAL "OFF")
	add_definitions(-DXTCAS_NO_AUDIO)
endif()

execute_process(COMMAND git rev-parse --short HEAD OUTPUT_VARIABLE XTCAS_VER)
string(REGEX REPLACE "\n$" "" XTCAS_VER "${XTCAS_VER}")
add_definitions(-DXTCAS_VER=0x${XTCAS_VER})

if("${VSI_MODE}" STREQUAL "ON")
	add_definitions(-DVSI_DRAW_MODE=1)
else()
	add_definitions(-DVSI_DRAW_MODE=0)
endif()

if("${VSI_STYLE}" STREQUAL "")
	add_definitions(-DVSI_STYLE=0)
else()
	add_definitions(-DVSI_STYLE=${VSI_STYLE})
endif()

if("${GTS820_MODE}" STREQUAL "ON")
	add_definitions(-DGTS820_MODE=1)
else()
	add_definitions(-DGTS820_MODE=0)
endif()

set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELEASE} -DDEBUG")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DDEBUG -O0")

if(${TEST_STANDALONE_BUILD})
	add_definitions(-DTEST_STANDALONE_BUILD)
	list(APPEND SRC test.c)
	list(APPEND HDR test.h)
else()
	list(APPEND SRC
	    generic_intf.c
	    xplane.c
	    xplane_test.c
	    ff_a320_intf.c
	    vsi.c
	)
	list(APPEND HDR
	    ../xtcas/generic_intf.h
	    xplane.h
	    xplane_test.h
	    ff_a320_intf.h
	    vsi.h
	)
endif()

set(ALL_SRC ${SRC} ${HDR})
list(SORT ALL_SRC)

if(APPLE)
	add_executable(xtcas ${ALL_SRC})
else()
	if(${TEST_STANDALONE_BUILD})
		add_executable(xtcas ${ALL_SRC})
	else()
		add_library(xtcas SHARED ${ALL_SRC})
	endif()
endif()

if(${TEST_STANDALONE_BUILD})
	set(OUTPUT_FILENAME "xtcas")
else()
	set(OUTPUT_FILENAME "X-TCAS.xpl")
	if(WIN32)
		set_target_properties(xtcas PROPERTIES LINK_FLAGS
		    "${CMAKE_SHARED_LINKER_FLAGS} \
		    -undefined_warning -fvisibility=hidden \
		    -static-libgcc -static-libstdc++")
	elseif(APPLE)
		set_target_properties(xtcas PROPERTIES LINK_FLAGS
		    "${CMAKE_SHARED_LINKER_FLAGS} -fvisibility=hidden -bundle")
	else()
		set_target_properties(xtcas PROPERTIES LINK_FLAGS
		    "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic -nodefaultlibs \
		    -undefined_warning -fPIC -fvisibility=hidden \
		    -static-libgcc -static-libstdc++ \
		    -Wl,--version-script=symbols.version")
	endif()
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -W -Wall -Wextra -Werror \
    -fvisibility=hidden \
    -DCHECK_RESULT_USED=\"__attribute__ ((warn_unused_result))\"")
if(NOT APPLE)
	add_definitions(-D_GNU_SOURCE)
endif()

if(NOT ${TEST_STANDALONE_BUILD})
	include_directories(xtcas PUBLIC
	    "${LIBACFUTILS}/SDK/CHeaders/XPLM"
	    "${LIBACFUTILS}/SDK/CHeaders/Widgets"
	    "${LIBACFUTILS}/SDK/CHeaders/Wrappers"
	    "../SDK")
endif()

if(APPLE)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-deprecated \
	    -mmacosx-version-min=10.12")
endif()

add_definitions(-DXPLM200=1 -DXPLM210=1)
if(WIN32)
	add_definitions(-DIBM=1 -DAPL=0 -DLIN=0 -D_WIN32_WINNT=0x0600)
elseif(APPLE)
	add_definitions(-DIBM=0 -DAPL=1 -DLIN=0)
else()
	add_definitions(-DIBM=0 -DAPL=0 -DLIN=1)
endif()

execute_process(COMMAND
    ${LIBACFUTILS}/pkg-config-deps ${PLAT_LONG} --static-openal --cflags
    OUTPUT_VARIABLE DEP_CFLAGS)
string(REGEX REPLACE "\n$" "" DEP_CFLAGS "${DEP_CFLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${DEP_CFLAGS}")

execute_process(COMMAND
    ${LIBACFUTILS}/pkg-config-deps ${PLAT_LONG} --static-openal --libs
    OUTPUT_VARIABLE DEP_LIBS)
string(REGEX REPLACE "\n$" "" DEP_LIBS "${DEP_LIBS}")

find_library(LIBACFUTILS_LIBRARY acfutils "${LIBACFUTILS}/qmake/${PLAT_SHORT}")
if(WIN32)
	find_library(XPLM_LIBRARY XPLM_64 "${LIBACFUTILS}/SDK/Libraries/Win")
	find_library(XPWIDGETS_LIBRARY XPWidgets_64
	    "${LIBACFUTILS}/SDK/Libraries/Win")
	find_library(OPENGL_LIBRARY opengl32 "../GL_for_Windows/lib")
	target_link_libraries(xtcas
	    ${LIBACFUTILS_LIBRARY}
	    ${XPLM_LIBRARY}
	    ${XPWIDGETS_LIBRARY}
	    ${OPENGL_LIBRARY}
	    ${DEP_LIBS}
	)
elseif(APPLE)
	find_library(XPLM_LIBRARY XPLM "${LIBACFUTILS}/SDK/Libraries/Mac")
	find_library(XPWIDGETS_LIBRARY XPWidgets
	    "${LIBACFUTILS}/SDK/Libraries/Mac")
	find_library(OPENGL_FRAMEWORK OpenGL)
	find_library(AUDIOUNIT_FRAMEWORK AudioUnit)
	find_library(COREAUDIO_FRAMEWORK CoreAudio)
	find_library(COREFOUNDATION_FRAMEWORK CoreFoundation)
	target_link_libraries(xtcas
	    ${LIBACFUTILS_LIBRARY}
	    ${XPLM_LIBRARY}
	    ${XPWIDGETS_LIBRARY}
	    ${OPENGL_FRAMEWORK}
	    ${AUDIOUNIT_FRAMEWORK}
	    ${COREAUDIO_FRAMEWORK}
	    ${COREFOUNDATION_FRAMEWORK}
	    ${DEP_LIBS}
	)
else()
	if(${TEST_STANDALONE_BUILD})
		find_library(CURSES_LIBRARY ncurses)
		target_link_libraries(xtcas
		    ${LIBACFUTILS_LIBRARY}
		    ${DEP_LIBS}
		    ${CURSES_LIBRARY}
		    "pthread"
		    "m"
		)
	else()
		target_link_libraries(xtcas
		    ${LIBACFUTILS_LIBRARY}
		    ${DEP_LIBS}
		)
	endif()
endif()

set_target_properties(xtcas PROPERTIES PREFIX "")
set_target_properties(xtcas PROPERTIES SUFFIX "")
set_target_properties(xtcas PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(xtcas PROPERTIES C_STANDARD 11)
set_target_properties(xtcas PROPERTIES CXX_STANDARD 17)

set_target_properties(xtcas PROPERTIES RUNTIME_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/../${PLUGIN_BIN_OUTDIR}")
set_target_properties(xtcas PROPERTIES LIBRARY_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/../${PLUGIN_BIN_OUTDIR}")
set_target_properties(xtcas PROPERTIES OUTPUT_NAME "${OUTPUT_FILENAME}")
