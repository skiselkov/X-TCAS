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

SET(SRC SL.c dbg_log.c pos.c xtcas.c snd_sys.c)
SET(HDR SL.h dbg_log.h pos.h xtcas.h snd_sys.h)

execute_process(COMMAND git rev-parse --short HEAD OUTPUT_VARIABLE XTCAS_VER)
string(REGEX REPLACE "\n$" "" XTCAS_VER "${XTCAS_VER}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DXTCAS_VER=0x${XTCAS_VER}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DVSI_DRAW_MODE=${VSI_DRAW_MODE}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DGTS820_MODE=${GTS820_MODE}")
if(NOT "${VSI_STYLE}" EQUAL "")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DVSI_STYLE=${VSI_STYLE}")
endif()

SET(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELEASE} -DDEBUG")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DDEBUG -O0")

if("${TEST_STANDALONE_BUILD}" EQUAL "1")
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DTEST_STANDALONE_BUILD")
	LIST(APPEND SRC test.c)
	LIST(APPEND HDR test.h)
else()
	LIST(APPEND SRC generic_intf.c xplane.c xplane_test.c ff_a320_intf.c
	    vsi.c)
	LIST(APPEND HDR ../xtcas/generic_intf.h xplane.h xplane_test.h
	    ff_a320_intf.h vsi.h)
	if(APPLE)
		execute_process(COMMAND
		    ${LIBACFUTILS}/pkg-config-deps mac-64 --cflags
		    OUTPUT_VARIABLE DEP_CFLAGS)
	else()
		execute_process(COMMAND
		    ${LIBACFUTILS}/pkg-config-deps linux-64 --cflags
		    OUTPUT_VARIABLE DEP_CFLAGS)
	endif()
	string(REGEX REPLACE "\n$" "" DEP_CFLAGS "${DEP_CFLAGS}")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${DEP_CFLAGS}")
endif()

SET(ALL_SRC ${SRC} ${HDR})
LIST(SORT ALL_SRC)

if(APPLE)
	add_executable(xtcas ${ALL_SRC})
else()
	if("${TEST_STANDALONE_BUILD}" EQUAL "1")
		add_executable(xtcas ${ALL_SRC})
	else()
		add_library(xtcas SHARED ${ALL_SRC})
	endif()
endif()

#compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -W -Wall -Wextra -Werror \
    -fvisibility=hidden \
    -DCHECK_RESULT_USED=\"__attribute__ ((warn_unused_result))\"")
if(APPLE)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
else()
	# Because Linux header fixes are a pile of stinking garbage.
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11 -D_GNU_SOURCE")
endif()

#libraries
if(UNIX)
	include_directories(xtcas PUBLIC "../OpenAL/include")
	if(NOT "${TEST_STANDALONE_BUILD}" EQUAL "1")
		include_directories(xtcas PUBLIC "../SDK/CHeaders/XPLM"
		    "../SDK/CHeaders/Widgets" "../SDK/CHeaders/Wrappers"
		    "../SDK")
	endif()
endif()

if(APPLE)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -arch x86_64 -Wno-deprecated")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mmacosx-version-min=10.7")
endif()

include_directories(bp PUBLIC "${LIBACFUTILS}/src")
include_directories(bp PUBLIC "${LIBACFUTILS}/glew/glew-1.13.0-linux-64/install/include")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DXPLM200=1 -DXPLM210=1")
if(APPLE)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DAPL=1 -DIBM=0 -DLIN=0")
else()
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DAPL=0 -DIBM=0 -DLIN=1")
endif()

#linking
link_libraries(m)
if(APPLE)
	set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} ../SDK/Libraries/Mac/
	    ${LIBACFUTILS}/qmake/mac64
	    ${LIBACFUTILS}/opus/opusfile-mac-64/lib
	    ${LIBACFUTILS}/opus/opus-mac-64/lib
	    ${LIBACFUTILS}/opus/libogg-mac-64/install/lib
	    ${LIBACFUTILS}/freetype/freetype-mac-64/lib
	    ${LIBACFUTILS}/cairo/pixman-mac-64/lib
	    ${LIBACFUTILS}/cairo/cairo-mac-64/lib
	    ${LIBACFUTILS}/zlib/zlib-mac-64/lib
	    ${LIBACFUTILS}/libpng/libpng-mac-64/lib)
	FIND_LIBRARY(XPLM_LIBRARY XPLM ../SDK/Libraries/Mac/)
	FIND_LIBRARY(XPWIDGETS_LIBRARY XPWidgets ../SDK/Libraries/Mac/)
	FIND_LIBRARY(OPENAL_LIBRARY OpenAL)
	FIND_LIBRARY(CURSES_LIBRARY ncurses)
	FIND_LIBRARY(LIBACFUTILS_LIBRARY acfutils)
	FIND_LIBRARY(OPUSFILE_LIBRARY opusfile)
	FIND_LIBRARY(OPUS_LIBRARY opus)
	FIND_LIBRARY(OGG_LIBRARY ogg)
	target_link_libraries(xtcas
	    ${LIBACFUTILS_LIBRARY}
	    ${OPUSFILE_LIBRARY}
	    ${OPUS_LIBRARY}
	    ${OGG_LIBRARY}
	    ${OPENAL_LIBRARY}
	    ${CAIRO_LIBRARY}
	    ${PIXMAN_LIBRARY}
	    ${FREETYPE_LIBRARY}
	    ${LIBPNG_LIBRARY}
	    ${ZLIB_LIBRARY})
	if("${TEST_STANDALONE_BUILD}" EQUAL "1")
		target_link_libraries(xtcas ${CURSES_LIBRARY})
	else()
		target_link_libraries(xtcas ${XPLM_LIBRARY}
		    ${XPWIDGETS_LIBRARY})
	endif()
else()
	set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH}
	    ${LIBACFUTILS}/qmake/lin64
	    ${LIBACFUTILS}/opus/opusfile-linux-64/lib
	    ${LIBACFUTILS}/opus/opus-linux-64/lib
	    ${LIBACFUTILS}/opus/libogg-linux-64/install/lib
	    ${LIBACFUTILS}/freetype/freetype-linux-64/lib
	    ${LIBACFUTILS}/cairo/pixman-linux-64/lib
	    ${LIBACFUTILS}/cairo/cairo-linux-64/lib
	    ${LIBACFUTILS}/zlib/zlib-linux-64/lib
	    ${LIBACFUTILS}/libpng/libpng-linux-64/lib)

	INCLUDE(FindPkgConfig)
	pkg_check_modules(OPENAL REQUIRED openal)
	link_directories(${OPENAL_LIBRARY_DIRS})

	FIND_LIBRARY(CURSES_LIBRARY ncurses)
	FIND_LIBRARY(LIBACFUTILS_LIBRARY acfutils)
	FIND_LIBRARY(OPUSFILE_LIBRARY opusfile)
	FIND_LIBRARY(OPUS_LIBRARY opus)
	FIND_LIBRARY(OGG_LIBRARY ogg)

	target_link_libraries(xtcas
	    ${LIBACFUTILS_LIBRARY}
	    ${OPUSFILE_LIBRARY}
	    ${OPUS_LIBRARY}
	    ${OGG_LIBRARY}
	    ${CAIRO_LIBRARY}
	    ${PIXMAN_LIBRARY}
	    ${FREETYPE_LIBRARY}
	    ${LIBPNG_LIBRARY}
	    "${LIBACFUTILS}/glew/glew-1.13.0-linux-64/install/lib64/libGLEWmx.a"
	    ${ZLIB_LIBRARY}
	    ${LIBC_NONSHARED}
	    )
	if("${TEST_STANDALONE_BUILD}" EQUAL "1")
		target_link_libraries(xtcas
		    ${CURSES_LIBRARY} ${OPENAL_LIBRARIES} "pthread" "m")
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
	else()
		set(OUTPUT_FILENAME "lin.xpl")
		SET_TARGET_PROPERTIES(xtcas PROPERTIES LINK_FLAGS
		    "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic -nodefaultlibs \
		    -undefined_warning -fPIC -fvisibility=hidden")
	endif()
endif()

set_target_properties(xtcas PROPERTIES RUNTIME_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/bin" )
set_target_properties(xtcas PROPERTIES LIBRARY_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/bin" )
set_target_properties(xtcas PROPERTIES OUTPUT_NAME "${OUTPUT_FILENAME}")
