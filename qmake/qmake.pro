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

# Shared library without any Qt functionality
TEMPLATE = lib
QT -= gui core

CONFIG += warn_on plugin release
CONFIG -= thread exceptions qt rtti debug

VERSION = 1.0.0

INCLUDEPATH += ../SDK/CHeaders/XPLM
INCLUDEPATH += ../SDK
INCLUDEPATH += $$[LIBACFUTILS]/src
INCLUDEPATH += $$[LIBACFUTILS]/glew

QMAKE_CFLAGS += -std=c99 -g -W -Wall -Wextra -Werror -fvisibility=hidden
QMAKE_CFLAGS += -Wunused-result -DTEST_STANDALONE_BUILD=0
QMAKE_CFLAGS += -DXTCAS_VER=0x$$system("git rev-parse --short HEAD")

QMAKE_APPLE_DEVICE_ARCHS = x86_64 arm64

# _GNU_SOURCE needed on Linux for getline()
# DEBUG - used by our ASSERT macro
# _FILE_OFFSET_BITS=64 to get 64-bit ftell and fseek on 32-bit platforms.
# _USE_MATH_DEFINES - sometimes helps getting M_PI defined from system headers
DEFINES += _GNU_SOURCE DEBUG _FILE_OFFSET_BITS=64 _USE_MATH_DEFINES

# Latest X-Plane APIs. No legacy support needed.
DEFINES += XPLM200 XPLM210

VSI_DRAW_MODE = $$[VSI_DRAW_MODE]
NO_AUDIO = $$[XTCAS_NO_AUDIO]

DEFINES += VSI_DRAW_MODE=$$[VSI_DRAW_MODE] VSI_STYLE=$$[VSI_STYLE]
DEFINES += GTS820_MODE=$$[GTS820_MODE]

contains(NO_AUDIO, 1) {
	DEFINES += XTCAS_NO_AUDIO
}

# Just a generally good idea not to depend on shipped libgcc.
!macx {
	LIBS += -static-libgcc
}

win32 {
	CONFIG += dll
	DEFINES += APL=0 IBM=1 LIN=0 _WIN32_WINNT=0x0600
	LIBS += -ldbghelp
	LIBS += -L../SDK/Libraries/Win
	TARGET = win.xpl
	INCLUDEPATH += /usr/include/GL
	QMAKE_DEL_FILE = rm -f
	QMAKE_LFLAGS_RELEASE =
}

!contains(VSI_DRAW_MODE, 0) {
}

win32:contains(CROSS_COMPILE, x86_64-w64-mingw32-) {
	QMAKE_CFLAGS += $$system("$$[LIBACFUTILS]/pkg-config-deps win-64 \
	    --static-openal --cflags")

	# This must go first so GCC finds the deps in the latter libraries
	LIBS += -L $$[LIBACFUTILS]/qmake/win64 -lacfutils
	LIBS += $$system("$$[LIBACFUTILS]/pkg-config-deps win-64 \
	    --static-openal --libs")
	LIBS += -lXPLM_64
	LIBS += -L../GL_for_Windows/lib -lopengl32
	LIBS += -ldbghelp
}

unix:!macx {
	DEFINES += APL=0 IBM=0 LIN=1
	TARGET = lin.xpl
	LIBS += -nodefaultlibs
}

linux-g++-64 {
	QMAKE_CFLAGS += $$system("$$[LIBACFUTILS]/pkg-config-deps linux-64 \
	    --static-openal --cflags")
	LIBS += -L $$[LIBACFUTILS]/qmake/lin64 -lacfutils
	LIBS += $$system("$$[LIBACFUTILS]/pkg-config-deps linux-64 \
	    --static-openal --libs")
}

macx {
	DEFINES += APL=1 IBM=0 LIN=0
	TARGET = mac.xpl
	LIBS += -F../SDK/Libraries/Mac
	LIBS += -framework XPLM -framework OpenGL -framework AudioToolbox
	LIBS += -framework CoreAudio -framework AudioUnit -framework Foundation
	QMAKE_CFLAGS += $$system("$$[LIBACFUTILS]/pkg-config-deps mac-64 \
	    --static-openal --cflags")

	# To make sure we run on everything that X-Plane 10 ran on
	QMAKE_MACOSX_DEPLOYMENT_TARGET=10.13
}

macx-clang {
	LIBS += $$system("$$[LIBACFUTILS]/pkg-config-deps mac-64 \
	    --static-openal --libs")
	LIBS += -L$$[LIBACFUTILS]/qmake/mac64 -lacfutils
}

HEADERS += \
	../src/dbg_log.h \
	../src/ff_a320_intf.h \
	../xtcas/generic_intf.h \
	../src/pos.h \
	../src/SL.h \
	../src/snd_sys.h \
	../src/xplane.h \
	../src/xplane_test.h \
	../src/xtcas.h

SOURCES += \
	../src/dbg_log.c \
	../src/ff_a320_intf.c \
	../src/generic_intf.c \
	../src/pos.c \
	../src/SL.c \
	../src/xplane.c \
	../src/xplane_test.c \
	../src/xtcas.c

!contains(NO_AUDIO, 1) {
	HEADERS += ../src/snd_sys.h
	SOURCES += ../src/snd_sys.c
}

!contains(VSI_DRAW_MODE, 0) {
	HEADERS += ../src/vsi.h
	SOURCES += ../src/vsi.c
}
