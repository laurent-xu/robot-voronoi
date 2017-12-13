
# Makefile for MobileSim.
# 
# This Makefile should work on Linux, MinGW on Windows, and perhaps other
# Unix-like systems as well.
#
# Some variables you can override with environment variables:
# 	DESTDIR		Set an alternative filesystem root (normally only used for making
#   INSTALL_DIR Where to install MobileSim, default /usr/local/MobileSim
# 						e.g. Debian packages)
# 	CXX				C++ compiler (default is make's default, usually 'c++')
# 	CC      	C compiler (default is make's default, usually 'cc')
# 	CFLAGS    Compilation flags.  This Makefile will also append various options
# 						(see below)
# 	LFLAGS    Linker flags (e.g. library directories). This Makefile will also
# 						append various options (see below)
# 	STAGEDIR  Where to find the Stage source code (default is stage/)
# 	STAGELIBDIR Where to find the Stage library for linking (default is
# 						$(STAGEDIR)/src/)
#
# Some variables that set build options:
#   MOBILESIM_RELEASE  If defined, then an optimized version 
#                          will be built instead of an unoptimized debug
#                          version. Both libstage and MobileSim are built
#                          with optimization.  Also, stage tests will be 
#                          skipped. Use this for releases.
#   
#   MOBILESIM_PROFILE   If defined, then profiling will be enabled
#                          with -pg (for gprof).
#
# Used for variations on the tar.gz bindist package:
# 	TAR_DIRECTORY		If defined, put files to be tarred in a new directory with
# 	this name, rather than MobileSim-$(VERSION). 
# 	BINDIST_SUFFIX	If defined, append this string to the tar.gz package name.
#

ifdef TRACE
REAL_SHELL:=$(SHELL)
SHELL=$(warning at rule   $@.   Modified prerequisites are: [$?]    All prerequisites are: [$^])$(REAL_SHELL)
endif


default: all

DATESTAMP=$(shell date +'%Y%m%d')

ifndef VERSION
ifeq (dist/version.num,$(wildcard dist/version.num))
VERSION:=$(shell cat dist/version.num)
else
VERSION:=dev$(DATESTAMP)
endif
endif

DATESTR:=$(shell date +'%B %e, %Y')

DEBIAN_PKG_REV_APPEND:=
# e.g. DEBIAN_PKG_REV_APPEND:="-3"
RPM_PKG_REV:=0

debver:=$(shell cat /etc/debian_version | cut -d '.' -f 1)
ifneq ($(debver), 6)
$(info debver=$(debver))
SYSTEM_SUFFIX:=+debian$(debver)
endif

arch:=$(shell dpkg-architecture -qDEB_BUILD_ARCH || echo "")
ifeq ($(arch),)
arch=i386
endif
ifneq ($(arch), i386)
$(info arch=$(arch))
SYSTEM_SUFFIX:=$(SYSTEM_SUFFIX)+$(arch)
endif


# TODO automatically use debchange to change debian changelog (see aria/arnl
# makefiles)


COMMON_DISTRIBUTED_FILES=Makefile LICENSE.txt Changes.txt README.md README.html screenshot.png PioneerRobotModels.world.inc AMROffice.map columbia.map icon.png MobileSim.desktop version.txt INSTALL.txt

BINARY_DISTRIBUTED_FILES=$(COMMON_DISTRIBUTED_FILES) MobileSim gdbhelper 

SOURCE_DISTRIBUTED_FILES=$(COMMON_DISTRIBUTED_FILES) main.cc README.src.txt *.cpp *.h *.hh *.cc stage/src/*.c stage/src/*.h stage/src/stagecpp.cc stage/src/worldfile.hh stage/src/worldfile.cc stage/src/config.h.in stage/tests/*.c stage/configure.in stage/*/Makefile.in stage/*/Makefile.am stage/*/*/Makefile.in stage/*/*/Makefile.am stage/Makefile.in stage/Makefile.am stage/AUTHORS stage/COPYING stage/ChangeLog stage/README stage/README-Windows.txt stage/aclocal.m4 stage/doxygen.conf.in stage/INSTALL stage/NEWS stage/autoconf-mingw.sh stage/stage.pc.in stage/replace/*.c stage/replace/*.h convertBitmapToArMap.cc util.h
SOURCE_DISTRIBUTED_FILES_EXEC=stage/configure stage/config.guess stage/config.sub stage/install-sh stage/missing  gdbhelper 

SOURCE_DISTRIBUTED_FILES_MAYBE=stage/compile stage/depcomp stage/ltmain.sh

CFLAGS+=-fPIC -DMOBILESIM
SIMBOX_EXCLUDE_FILES=README.html columbia.map MobileSim.desktop icon.png screenshot.png Makefile INSTALL.txt AMROffice.map



ifdef MOBILESIM_DEBUG

$(info Debug build)
CFLAGS+=-g -O0 -Wall -W -Wconversion # -W is the same as -Wextra but supported by gcc 2
STAGE_CONFIGURE_ARGS:=--disable-optimize --enable-debug --enable-tests
datestamp:=$(shell date +%Y%m%d)
-include lastDevReleaseVer
ifdef lastDevReleaseVer
lastDevReleaseVerDatestamp:=$(shell echo $(lastDevReleaseVer) | cut -d '.' -f 1)
lastDevReleaseVerRev:=$(shell echo $(lastDevReleaseVer) | cut -d '.' -f 2)
ifeq ($(lastDevReleaseVerDatestamp),$(datestamp))
 ifeq ($(lastDevReleaseVerDatestamp),$(lastDevReleaseVerRev))
  # has no revision
  DEV_RELEASE_VER:=$(datestamp).1
 else
  # increment revision
  DEV_RELEASE_VER:=$(datestamp).$(shell let foo=$(lastDevReleaseVerRev)+1; echo $$foo)
 endif
else
  # new date
  DEV_RELEASE_VER:=$(datestamp)
endif #ifeq lastDevRelease, datestamp
else
  DEV_RELEASE_VER:=$(datestamp)
endif #ifdef lastDevReleaseVer
# In debug mode, replace version with special dev tag with timestamp:
VERSION:=$(VERSION)-dev$(DEV_RELEASE_VER)

dist-all:
	$(warning Re-running make dist-all with MOBILESIM_RELEASE=1)
	$(MAKE) dist-all MOBILESIM_RELEASE=1

else

CFLAGS+=-O2
STAGE_CONFIGURE_ARGS:=--enable-debug --enable-optimize=2 --disable-tests
LFLAGS+=-Wl,--gc-sections 

dist-all: all

endif	 #ifdef MOBILESIM_DEBUG


ifdef MOBILESIM_PROFILE
CXX+=-pg -g -O0
CC+=-pg -g -O0
STAGE_CONFIGURE_ARGS+=--enable-profile --enable-debug --disable-optimize
endif	 #MOBILESIM_PROFILE


# Simbox rule overrides this 
ifndef TAR_DIRECTORY
TAR_DIRECTORY:=MobileSim-$(VERSION)
endif

ifndef ARIA
ARIA=../Aria
endif

#### Different options for Windows or Linux:
host:=$(shell uname | cut -d _ -f 1)
ifeq ($(host),MINGW32)	# host is MINGW32 on Windows:

ifdef MOBILESIM_DEBUG
# Omitting -mwindows makes it show stdout to a DOS command window:
CFLAGS+=-mms-bitfields -DMINGW -D__MINGW__ -DARIA_STATIC
else
CFLAGS+=-mwindows -mms-bitfields -DMINGW -D__MINGW__ -DARIA_STATIC
endif #MOBILESIM_DEBUG
binary_suffix:=.exe

platformsuffix:=_WIN
#SYSTEM_LINK:=-lpthreadGC2 -lwinmm -lws2_32 -lstdc++ 
SYSTEM_LINK:=-lwinmm -lws2_32 -lpthread #-lstdc++ 

LIBNETPBM:=libnetpbm/lib/libnetpbm.a
$(LIBNETPBM):
	$(MAKE) -C libnetpbm

ARIA_LINK=-L$(ARIA)/lib -Wl,-Bstatic -lAria -Wl,-Bdynamic -lm
STAGE_AUTOCONF_ARGS:=-I gtk-win/share/aclocal

PKG_CONFIG_PATH:=$(PKG_CONFIG_PATH):stage/gtk-win/lib/pkgconfig

else #else assume Linux or Unix:

  SYSTEM_LINK:=-ldl -lm
  LIBNETPBM:=-lnetpbm
  ARIA_LINK=$(ARIA)/lib/libAria.a
  platformsuffix:=_LIN

  ifeq ($(host),Darwin)
    $(info Building on Mac OSX (Darwin))
    CFLAGS+=-DMACOSX
    PKG_CONFIG_PATH:=$(PKG_CONFIG_PATH):../gtk/lib/pkgconfig:../gtk/share/pkgconfig
    STAGE_AUTOCONF_ARGS:=-I ../gtk/inst/share/aclocal
    EXTRA_TARGETS=MobileSim.app 
  else
    SYSTEM_LINK+=-lrt
    STAGE_AUTOCONF_ARGS:=""
    #ARIA_LINK:=-L$(ARIA)/lib -Wl,-Bstatic -lAria -Wl,-Bdynamic  
  endif

endif #host is MINGW32 or not

LIBARIA:=$(ARIA)/lib/libAria.a
ARIA_CFLAGS:=-I$(ARIA)/include 



###


#ARIA_SOURCES:=\
#	ariaUtil.cpp \
#	ArArgumentBuilder.cpp \
#	ArArgumentParser.cpp \
#	ArBasePacket.cpp \
#	ArConfig.cpp \
#	ArConfigArg.cpp \
#	ArDeviceConnection.cpp \
#	ArFileParser.cpp \
#	ArLog.cpp \
#	ArMap.cpp \
#	ArRobotPacket.cpp \
#	ArRobotPacketReceiver.cpp \
#	ArRobotPacketSender.cpp \
#	ArRobotParams.cpp \
#	ArSocket.cpp \
#	ArSocket$(platformsuffix).cpp \
#	ArTcpConnection.cpp \
#  ArGPSCoords.cpp

SOURCES:=\
	main.cc \
	EmulatePioneer.cc \
	RobotFactory.cc \
	StageInterface.cc \
	StageRobotFactory.cc \
	CrashHandler.cc \
	Config.cc \
  MapLoader.cc \
  ClientPacketReceiver.cpp \
  Socket.cc 

#	$(ARIA_SOURCES)

OBJS:=$(patsubst %.cc,%.o,$(patsubst %.cpp,%.o,$(SOURCES)))

#ARIA_OBJS:=$(patsubst %.cpp,%.o,$(ARIA_SOURCES))


CFLAGS+=-I/usr/local/include
LFLAGS+=-L/usr/local/lib

ifndef STAGEDIR
STAGEDIR:=stage
endif

ifndef STAGELIBDIR
STAGELIBDIR:=$(STAGEDIR)/src
endif

# Root directory that install is relative too. Normally relative 
# to nothing (i.e. in /)
ifndef DESTDIR
DESTDIR:=
endif

STAGELIBS:=$(STAGELIBDIR)/libstage.a $(STAGEDIR)/replace/libreplace.a


#GTK_LIBS=`pkg-config --libs gtk+-2.0`
#GTK_CFLAGS=`pkg-config --cflags gtk+-2.0`
GTK_LIBS:=$(shell pkg-config --libs gtk+-2.0)
GTK_CFLAGS:=$(shell pkg-config --cflags gtk+-2.0)


# For dynamic linkage (always works fine):
GTK_LINK:=$(GTK_LIBS)

# A try at static linkage, has mysterious problems with gobject initialization at runtime:
#GTK_CFLAGS=`pkg-config --cflags --static gtk+-2.0`
#GTK_LINK = \
#	-Wl,-Bstatic \
#		`pkg-config --libs --static gtk+-2.0 | sed 's/-ldl//' | sed 's/-lm//'` \
#		`pkg-config --libs --static xft | sed 's/-ldl//'` \
#		`pkg-config --libs --static xcursor | sed 's/-ldl//' | sed 's/-lX11//'` \
#		`pkg-config --libs --static xrender | sed 's/-ldl//' | sed 's/-lX11//'` \
#		-lXinerama -lXrandr -lfreetype -lexpat -ltiff -ljpeg -lpng -lz -lm \
#	-Wl,-Bdynamic \
#		-lXi -lX11 -ldl


MSIM_CFLAGS := -DMOBILESIM_VERSION=\"$(VERSION)\" -DMOBILESIM_BUILDDATE="\"$(DATESTR)\"" \
  -I. $(CFLAGS) -I$(STAGEDIR) -I$(STAGEDIR)/replace  -I$(STAGEDIR)/src \
	$(GTK_CFLAGS) $(ARIA_CFLAGS)

MSIM_LFLAGS := $(LFLAGS) 


# For installation:
ifndef INSTALL_DIR
INSTALL_DIR:=/usr/local/MobileSim
endif

bindir:=$(DESTDIR)/$(INSTALL_DIR)
docdir:=$(DESTDIR)/$(INSTALL_DIR)
confdir:=$(DESTDIR)/$(INSTALL_DIR)
ourlibdir:=$(DESTDIR)/$(INSTALL_DIR)
sysbindir:=$(DESTDIR)/usr/local/bin

# Bypases DESTDIR:
ifndef installed_bindir
installed_bindir:=$(bindir)
endif



all: MobileSim$(binary_suffix) $(EXTRA_TARGETS) columbia.map


debug:  MobileSim_debug$(binary_suffix)

%.o: %.cc
	$(CXX) -c $(MSIM_CFLAGS) -o $@ $<

%.o: %.cpp
	$(CXX) -c $(MSIM_CFLAGS) -o $@ $<

%.o: %.c
	$(CC) -c $(MSIM_CFLAGS) -o $@ $<

# Manually rebuild dep
dep: clean clean-dep 
	$(MAKE) Makefile.dep

include Makefile.dep

Makefile.dep:  $(STAGEDIR)/src/config.h
	$(info Building Makefile.dep)
	$(CXX) $(MSIM_CFLAGS) -MM $(SOURCES) >Makefile.dep

cleanDep: clean-dep

clean-dep:
	-rm Makefile.dep

ifeq (main.cc,$(wildcard main.cc))

# We have source code, depend on it being built
# could depend on $(STAGELIBS) instead of just libstage.a to get both libs, but that can cause make to try
# to build stage twice in parallel if using parallel jobserver, which causes
# corrupted output files.
MobileSim: $(STAGEDIR)/src/stage.h $(STAGEDIR)/src/config.h $(STAGELIBDIR)/libstage.a $(OBJS) $(LIBARIA)
	$(CXX) $(MSIM_CFLAGS) $(MSIM_LFLAGS) -o MobileSim $(OBJS) $(STAGELIBS) $(GTK_LINK) $(ARIA_LINK) $(SYSTEM_LINK)

mobilesimd: $(STAGE_DIR)/src/stage.h $(STAGEDIR)/src/config.h $(STAGELIBDIR)/libstage_nogui.a $(OBJS) $(LIBARIA)
	$(CXX) $(MSIM_CFLAGS) -DMOBILESIM_NOGUI $(MSIM_LFLAGS) -o mobilesimd $(OBJS) $(STAGELIBS) $(ARIA_LINK) $(SYSTEM_LINK)

else

# We don't have source code, no dependencies for MobileSim

endif

MobileSim_debug$(binary_suffix): MobileSim
	cp MobileSim$(binary_suffix) MobileSim_debug$(binary_suffix)

MobileSim.app: MobileSim
	-mkdir $@
	-mkdir $@/Contents
	-mkdir $@/Contents/MacOS
	echo -n 'APPL????' > $@/Contents/PkgInfo
	cp $< $@/Contents/MacOS/$<
# TODO should include PioneerRobotModels.world.inc, README etc. here and get MobileSim to 
# use that instead of /usr/local/MobileSim/...

# Add a resource fork to MobileSim plain binary. Only used on Mac OSX.
rezMobileSim: MobileSim
	rez -o $^

%.map: $(ARIA)/maps/%.map
	cp $< $@

markdown_format:=markdown+multiline_tables+simple_tables
%.rtf: %.md
	pandoc -s --toc --template pandoc_template -f $(markdown_format) -t rtf -o $@ $<

%.txt: %.md
	pandoc -s --toc -f $(markdown_format) -t plain -o $@ $<

%.pdf: %.md
	pandoc -s --toc -f $(markdown_format) -o $@ $<

%.html: %.md
	pandoc -s --toc -f $(markdown_format) -o $@ $<

$(ARIA)/lib/libAria.a: FORCE
	$(MAKE) -C $(ARIA) lib/libAria.a

#test_mainloop: test_mainloop.cc $(ARIA_OBJS)
#	$(CXX) $(MSIM_CFLAGS) $(MSIM_LFLAGS) -O2 -o $@ $< $(ARIA_OBJS) $(SYSTEM_LINK) && strip $@
#
#test_sleep_time: test_sleep_time.cc $(ARIA_OBJS)
#	$(CXX) $(MSIM_CFLAGS) $(MSIM_LFLAGS) -O2 -o $@ $< $(ARIA_OBJS) $(SYSTEM_LINK) && strip $@

test_mainloop: test_mainloop.cc $(LIBARIA)
	$(CXX) $(MSIM_CFLAGS) $(MSIM_LFLAGS) -O2 -o $@ $< $(ARIA_LINK) $(SYSTEM_LINK) && strip $@

test_sleep_time: test_sleep_time.cc $(LIBARIA)
	$(CXX) $(MSIM_CFLAGS) $(MSIM_LFLAGS) -O2 -o $@ $< $(ARIA_LINK) $(SYSTEM_LINK) && strip $@


ifndef AUTOCONF
AUTOCONF:=autoconf
endif

ifndef AUTORECONF
AUTORECONF:=autoreconf
endif

ifndef ACLOCAL
ACLOCAL:=aclocal
endif

$(STAGEDIR)/config.status $(STAGEDIR)/Makefile $(STAGEDIR)/src/config.h: $(STAGEDIR)/configure
	$(info Configuring stage to generate complete header files (config.h))
	test -d $(STAGEDIR) || { echo "STAGEDIR \"$(STAGEDIR)\" does not exist. Set STAGEDIR in the environment to correct path or omit to use stage subdirectory provided with MobileSim source code."; false; }
	+cd $(STAGEDIR); \
    export PKG_CONFIG_PATH=$$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig; \
    ./configure --disable-shared --enable-static \
      --disable-fiducial-model --disable-blobfinder-model --disable-gripper-model \
      --disable-save-world --disable-grid-labels --disable-reload-world \
	  	$(STAGE_CONFIGURE_ARGS)
	touch $@



$(STAGEDIR)/configure:  $(STAGEDIR)/configure.in
	+cd $(STAGEDIR); if test -d /usr/local/share/aclocal; then args="$(STAGE_AUTOCONF_ARGS) -I /usr/local/share/aclocal"; else args="$(STAGE_AUTOCONF_ARGS)"; fi; echo running $(AUTORECONF) -i $$args and $(AUTOCONF) $$args && $(AUTORECONF) -i $$args && $(ACLOCAL) $$args && $(AUTOCONF) $$args

stageconf: $(STAGEDIR)/config.status

#	@echo "ERROR: configure script missing from stage! enter stage directory and run 'autoreconf -i' (use --force to force creation or re-creation of files), or unpack the tar archive again."
#	false


convertBitmapToArMap: convertBitmapToArMap.cc $(LIBARIA)
	$(CXX) -o $@ $< $(ARIA_LINK) $(LIBNETPBM) -lm -ldl $(SYSTEM_LINK)


help:
	@echo 'Usage:'
	@echo '       make help       This message'
	@echo '       make all or make MobileSim  Build MobileSim program (in debug/dev mode)'
	@echo '       make opt        Build MobileSim program (in optimized/release mode)'
	@echo '       make clean'
	@echo '       make install'
	@echo '       make distclean  Clean temporary files, but leave binaries to be distributed'
	@echo '       make release-dist'
	@echo '       make dev-dist'
	@echo '       make release-srcdist'
	@echo '       make dev-srcdist'
	@echo '       make release-bindist'
	@echo '       make dev-bindist'
	@echo '       make release-windist'
	@echo '       make dev-windist'
	@echo '       make rpm'
	@echo '       make dev-rpm'
	@echo '       make deb'
	@echo '       make dev-deb'
	@echo '       make simbox'
	@echo '       make dev-simbox'
	@echo 'Set environment variable MOBILESIM_DEBUG to build a debug version (with optimization disabled, more debugger information, no compiler warnings, and with logging to terminal on Windows)'
	@echo 'Set environment variable MOBILESIM_PROFILE to build with profiling information (for use with gprof)'

info:
	@echo SOURCES=$(SOURCES)
	@echo
	@echo OBJS=$(OBJS)
	@echo
	@echo MSIM_CFLAGS=$(MSIM_CFLAGS)
	@echo
	@echo MSIM_LFLAGS=$(MSIM_LFLAGS)
	@echo
	@echo GTK_LINK=$(GTK_LINK)
	@echo
	@echo SYSTEM_LINK=$(SYSTEM_LINK)
	@echo
	@echo arch=$(arch)
	@echo SYSTEM_SUFFIX=$(SYSTEM_SUFFIX)
	@echo	
	@echo BINDIST_SUFFIX=$(BINDIST_SUFFIX)

$(STAGELIBDIR)/libstage.a $(STAGEDIR)/replace/libreplace.a: $(STAGEDIR)/config.status $(STAGEDIR)/src/*.c $(STAGEDIR)/src/*.h Makefile $(STAGEDIR)/Makefile
	test -d $(STAGEDIR) || { echo "STAGEDIR \"$(STAGEDIR)\" does not exist. Set STAGEDIR in the environment, or check if there is something wrong with your original source archive or VCS checkout..."; false; }
	$(MAKE) -C $(STAGEDIR) -j1

clean:
	+$(MAKE) -C $(STAGEDIR) -j1 clean
	-rm MobileSim *.o

cleanAll: clean

distclean: 
	$(MAKE) -C $(STAGEDIR) -k -j1 clean distclean 
	-rm *.o


sudo-install:
	sudo $(MAKE) install

ifdef MOBILESIM_RELEASE
dist-install: $(BINARY_DISTRIBUTED_FILES) 
	$(MAKE) install
else
dist-install:
	$(MAKE) dist-install MOBILESIM_RELEASE=1
endif

install:
	install -d $(bindir) $(docdir) $(confdir) $(DESTDIR)/usr/share/applications $(sysbindir) 
	install -s -m 755 MobileSim $(bindir)/MobileSim
	install -m 644 icon.png $(confdir)/icon.png
	install -m 644 columbia.map $(confdir)/columbia.map
	install -m 644 AMROffice.map $(confdir)/AMROffice.map
	install -m 644 PioneerRobotModels.world.inc $(confdir)/PioneerRobotModels.world.inc
	install -m 644 MobileSim.desktop $(DESTDIR)/usr/share/applications/MobileSim.desktop
	install -m 644 README.html $(docdir)/README.html
	install -m 644 screenshot.png $(docdir)/screenshot.png
	install -m 644 Changes.txt $(docdir)/Changes.txt
	install -m 644 INSTALL.txt $(docdir)/INSTALL.txt
	install -m 644 LICENSE.txt $(docdir)/LICENSE.txt
	install -m 644 version.txt $(docdir)/version.txt
	install -m 755 gdbhelper $(bindir)/gdbhelper
	ln -s -f $(installed_bindir)/MobileSim $(sysbindir)/MobileSim

uninstall:
	-rm $(bindir)/MobileSim $(confdir)/icon.png \
		$(docdir)/README.html $(docdir)/Changes.txt $(docdir)/LICENSE.txt \
		$(docdir)/INSTALL.txt \
		/usr/share/applications/MobileSim.desktop \
    $(sysbindir)/MobileSim

rpm:
	$(MAKE) real-rpm MOBILESIM_RELEASE=1

real-rpm: $(BINARY_DISTRIBUTED_FILES) sudo-dist-install MobileSim.spec 
	sudo rpm -bb MobileSim.spec && cp /usr/src/redhat/RPMS/i386/MobileSim-$(VERSION_NODASH)-$(RPM_PKG_REV).i386.rpm . && echo "Copied RPM into current directory."


debian: deb
deb: 
	fakeroot debian/rules binary MOBILESIM_RELEASE=1

dev-deb: 
	fakeroot debian/rules binary
	echo lastDevReleaseVer=$(DEV_RELEASE_VER) > lastDevReleaseVer

dev-debian: dev-deb

dist: release-dist

release-dist:
	$(MAKE) base-dist MOBILESIM_RELEASE=1

dev-dist: base-dist
	echo lastDevReleaseVer=$(DEV_RELEASE_VER) > lastDevReleaseVer

base-dist: base-srcdist base-bindist

srcdist: release-srcdist

release-srcdist:
	$(MAKE) base-srcdist MOBILESIM_RELEASE=1

dev-srcdist: base-srcdist
	echo lastDevReleaseVer=$(DEV_RELEASE_VER) > lastDevReleaseVer

base-srcdist: srcdist-install 
	tar czf MobileSim-src-$(VERSION).tar.gz MobileSim-src-$(VERSION)

srcdist-install: $(SOURCE_DISTRIBUTED_FILES)  $(SOURCE_DISTRIBUTED_FILES_EXEC)
	-mkdir MobileSim-src-$(VERSION)
	for d in $(SOURCE_DISTRIBUTED_FILES); do install -m 644 -D -p $$d MobileSim-src-$(VERSION)/$$d; done
	for d in $(SOURCE_DISTRIBUTED_FILES_MAYBE); do if test -f $$d; then install -m 644 -D -p $$d MobileSim-src-$(VERSION)/$$d; fi; done
	for d in $(SOURCE_DISTRIBUTED_FILES_EXEC); do install -m 755 -D -p $$d MobileSim-src-$(VERSION)/$$d; done

#	if test -n "$(ZIPFILE)"; then zip -9 -r MobileSim-src-$(VERSION).zip MobileSim-src-$(VERSION);\
#  else tar cf MobileSim-src-$(VERSION).tar MobileSim-src-$(VERSION) && gzip -9 MobileSim-src-$(VERSION).tar && mv MobileSim-src-$(VERSION).tar.gz MobileSim-src-$(VERSION).tgz; fi

tgz: release-bindist

bindist: release-bindist

release-bindist:
	$(MAKE) base-bindist MOBILESIM_RELEASE=1

dev-bindist: base-bindist
	echo lastDevReleaseVer=$(VERSION) > lastDevReleaseVer


PKGFILE_TAR=MobileSim-$(VERSION)$(BINDIST_SUFFIX)$(SYSTEM_SUFFIX).tar
PKGFILE_TGZ=MobileSim-$(VERSION)$(BINDIST_SUFFIX)$(SYSTEM_SUFFIX).tgz
PKGFILE=$(PKGFILE_TGZ)
DISTINFO_FILE=MobileSim-$(VERSION)$(BINDIST_SUFFIX)$(SYSTEM_SUFFIX)__info.txt

base-bindist: $(BINARY_DISTRIBUTED_FILES)
	-mkdir -p tmp/$(TAR_DIRECTORY)
	for f in $(BINARY_DISTRIBUTED_FILES); do cp $$f tmp/$(TAR_DIRECTORY)/$$f; done
	if test -n "$(EXCLUDE_FILES)"; then for f in $(EXCLUDE_FILES); do rm tmp/$(TAR_DIRECTORY)/$$f; done; fi
	strip tmp/$(TAR_DIRECTORY)/MobileSim
	cd tmp; tar cf ../$(PKGFILE_TAR) $(TAR_DIRECTORY)
	gzip -9 $(PKGFILE_TAR) && mv $(PKGFILE_TAR).gz $(PKGFILE_TGZ) 
	ln -sf $(PKGFILE_TGZ) MobileSim-latest.tgz
	rm -r tmp
	$(MAKE) $(DISTINFO_FILE)

bindist-info: $(DISTINFO_FILE)


simbank-dist: release-simbox

simbox: release-simbox

simbox-dist: release-simbox

release-simbox:
	$(MAKE) base-bindist MOBILESIM_RELEASE=1 TAR_DIRECTORY=MobileSim BINDIST_SUFFIX=-simbox EXCLUDE_FILES="$(SIMBOX_EXCLUDE_FILES)"
	$(MAKE) MobileSim/control
	$(MAKE) MobileSim/md5sums

MobileSim/control: Makefile dist/version.num
	echo Package: MobileSim >$@
	echo FullName: MobileSim >>$@
	echo Description: Mobile Robot Simulation Engine >>$@
	echo Removable: yes >>$@
	echo Executable: yes >>$@
	echo Visible: yes >>$@
	echo HasDownload: no >>$@
	echo Platform: EM >>$@
	echo Version: $(VERSION) >>$@

MobileSim/md5sums: MobileSim/MobileSim MobileSim/PioneerRobotModels.world.inc MobileSim/version.txt MobileSim/LICENSE.txt MobileSim/gdbhelper Makefile
	-rm $@
	for f in MobileSim/*; do echo md5sum $$f; md5sum $$f >>$@ ; done

#	@if [ ! -e /is_MTX_chroot ] ; then \
#		echo "Cannot build MTX generation (4.0+) releases (like SimBox) except on the official MTX chroot"; \
#		exit -1; \
#	fi; 

release-simbank: release-simbox

dev-simbox:
	$(MAKE) base-bindist TAR_DIRECTORY=MobileSim BINDIST_SUFFIX=-simbox EXCLUDE_FILES="$(SIMBOX_EXCLUDE_FILES)"
	echo lastDevReleaseVer=$(DEV_RELEASE_VER) > lastDevReleaseVer

dev-simbank: dev-simbox

optimize: opt

opt: 
	$(MAKE) MOBILESIM_RELEASE=1

%-opt:
	$(MAKE) $* MOBILESIM_RELEASE=1

WEBSITE_VERSION_NUMBERS_FILES:=MobileSimVersion.php

# Make PHP files with version numbers for use on website
website-version-numbers: $(WEBSITE_VERSION_NUMBERS_FILES)

# XXX TODO get debian and redhat package revisions from somewhere
%Version.php:
	echo "<?php" > $@
	echo "$$"$*Version=\"$(VERSION)\"\; >> $@
	echo "$$"$*DebianVersion=\"$(VERSION)$(DEBIAN_PKG_REV_APPEND)\"\; >> $@
	echo "$$"$*RedHatVersion=\"$(VERSION)-$(RPM_PKG_REV)\"\; >> $@
	echo "$$"$*TgzVersion=\"$(VERSION)\"\; >> $@
	echo "$$"$*WindowsVersion=\"$(VERSION)\"\; >> $@
	echo "?>" >> $@

upload-website-version-numbers: $(WEBSITE_VERSION_NUMBERS_FILES)
	rsync --update $^ root@192.168.0.8:/var/www/mediawiki/MobileRobotsSoftwareVersionNumbers/

# XXX TODO add rule to rsync the package
# files in with website download directories, creating them as neccesary
# for new versions.

.PHONY: all clean distclean dep cleanDep install uninstall dist-install sudo-install sudo-dist-install rpm deb debian dist srcdist bindist test-dist undo-dist simbank opt optimize  stageconf windist base-windist base-bindist base-srcdist dev-debian dev-deb dev-srcdist dev-bindist dev-simbank release-simbank release-srcdist release-bindist

FORCE:
