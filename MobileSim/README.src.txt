
=========================================
MobileSim Source Code Information:
 * Building MobileSim 
 * How MobileSim works
=========================================

Reed Hedges <reed@mobilerobots.com>

This document provides an overview of MobileSim for people interested in
compiling MobileSim from source, modifying MobileSim, or just understanding 
how it works.

MobileSim is software for simulating MobileRobots mobile robots
and their environments, for debugging and experimentation.

MobileSim is based on the Stage library, created by the 
Player/Stage project <http://playerstage.sourceforge.net>,
but modify to add some needed features.  

All of the actual simulation is done by the Stage library,
as well as the graphical display of the robot, sensors and map.
MobileSim merely packages up the library into an easy to use
application, and interprets Pioneer robot-compatible commands
and data for Stage.

Player, Stage and all MobileSim components are free software,
distributed under the terms of the GNU General Public License:
see the file LICENSE.txt for details.



----------------------------
Modifying MobileSim or Stage
----------------------------

Both Stage and MobileSim are distributed to you under the terms of the 
GNU General Public Lincese (GPL).  See the file named LICENSE.txt for the
text of this copyright license, which in particular, requires you to
release source code of any modified versions make under the
GPL as well, if you also distribute the modified program.  See 
LICENSE.txt for the details.

If you do make any modifications and would like to share them, please
do so via the appropriate channel:
 * For modifications to Stage (libstage), you can share the change with all
   Stage developers and users (not just MobileSim developers and users) via 
   the patch tracker and mailing list at <http://playerstage.sf.net>. 
 * For modifications to just MobileSim (emulatePioneer; MobileSim main.cc or tools)
   post them to the public aria-users mailing list or email them to MobileRobots
   technical support. See <http://robots.mobilerobots.com>

If you have any questions about how MobileSim works or about modifying it,
please discuss on the aria-users mailing list.



----------------------------------
How to Compile Stage and MobileSim
----------------------------------


Linux
-----

To build MobileSim on Linux, you will need Aria 2.8 or later, GTK 2.x, including 
development packages, and the full GNU development tools: 
G++ (3.2 or later), make, libtool, automake, and autoconf.  
If you have multiple versions of automake or autoconf, or have
them installed somewhere unusual, you can choose which commands
to use by setting the AUTOMAKE, AUTOCONF, AUTORECONF and ACLOCAL
environment variables before running make (e.g set them to 
run the newer version).  A modified version of Stage required by 
MobileSim is included in the 'stage' subdirectory; you may enter 
that directory and manually configure and build libstage if 
required, or you may just run 'make' here in the main MobileSim 
directory to build both libstage.a and MobileSim.  Then run 
'make install' to install MobileSim into /usr/local.  

On Debian and Ubuntu Linux, you can use apt-get to install the required
packages as follows:
  apt-get install libgtk2.0-dev automake autoconf libtool make g++

Follow the instructions below to build MobileSim.

Windows
-------

On Windows, you must use MinGW, MSYS and the MSYS DTK (http://www.mingw.org) 
to build MobileSim. Follow the instructions provided with MinGW and MSYS
to install and set up MinGW and MSYS.

Stage uses the autotools, (autoconf, automake, libtool, etc.) so
if you plan on customizing anything about Stage's build process, or
otherwise need to regenerate its configure script, config.h header, or its 
Makefiles, you will need those.   MobileSim's Makefile will run the autotools
to regenerate the aclocal.m4 and configure scripts if neccesary, and run
configure to generate the Makefiles etc., or you can do it by entering the 
stage directory, running "autoreconf -i" or "autoreconf -i --force". Then go 
back up to the MobileSim directory, and run "make stage/config.status" to 
run the stage configure script, and "make" to build everything.

ARIA in normally built using Visual C++ on Windows, but it must be rebuilt
using MinGW for use with MobileSim.  In addition to being built with a different
compiler, ARIA uses pthreads and a few other POSIX features when built with
MinGW rather than Windows implementations; it will still use ws2_32 
from Windows for network sockets, and winmm for sound playback, joystick
and other features however.  When you run "make" to build MobileSim,
it will enter the Aria directory (see below) and rebuild ARIA using MinGW,
resulting in libAria.a in ARIA's lib directory.

Next, follow the instructons below to build MobileSim.

Stage and MobileSim use GTK 2 for the GUI.  GTK libraries for MinGW are
included in stage/gtk-win and are used automatically when built in MinGW.

Mac OSX
-------

It is possible to build MobileSim for OSX 10.04 (Tiger) or later using the 
OSX port of GTK.

You will need to install XCode, and then install the command-line development
tools (Run XCode, then open "Preferenceo..." from the "XCode" application menu. Select
the "Downloads" tab. Select "Command Line Tools" and click its "Install"
button. This allows you to use Terminal shells to build using "make",
"c++" and other commands directly. 

Follow the instructions at https://wiki.gnome.org/GTK+/OSX/Building to 
download and build GTK 2 and other required libraries.

You will also need to downoald the ARIA source code and compile ARIA for
OSX. 

Follow the instructions below to build MobileSim, but add the tools installed
for GTK to your PATH first:

   export PATH=$PATH:~/gtk/inst/bin


Building MobileSim
------------------

Unless an ARIA environment variable was set, MobileSim looks for 
ARIA in an "../Aria" directory (in the parent directory, i.e.
both the Aria source code and MobileSim source code should be
in the same parent directory).  If you want to use the version of
ARIA already installed on your system, set the ARIA environment variable to 
its installation location:

For Linux:
  export ARIA=/usr/local/Aria

For Windows with MinGW:
  export ARIA="/c/Program Files/MobileRobots/Aria"

When building MobileSim, you can set environment variables that affect
compilation options:
 
  CXX                     Set the C++ compiler; if unset, "c++" is used.

  CC                      Set the C compiler; if unset, "cc" is used.

  CFLAGS                  Additional compilation flags for C.
  
  CXXFLAGS                Additional compilation flags for C++.

  LFLAGS                  Additional linker flags.

  MOBILESIM_RELEASE       Build MobileSim with optimizations enabled and
                          debugging disabled and skip some libstage test programs.

  MOBILESIM_PROFILE       Build with profiling enabled with -pg; analyze later
                          with gprof.

  prefix                  Installation directory base (default is /usr/local)
   
  DESTDIR                 Alternate root for installation (default is none,
                          therefore the root filesystem is used).

  STAGEDIR                Alternate Stage directory (default is stage/)

  STAGELIBDIR             Alternate Stage library directory (default is
                          stage/src/.libs)

  AUTORECONF		      autoreconf command

  AUTOCONF                autoconf command

  ACLOCAL                 aclocal command

  AUTOMAKE                automake command

You can also edit Makefile if neccesary.

MobileSim requires certain resources at runtime (such as the robot models definitions 
file) to work correctly, so you must either install MobileSim with 
'make install', or set a MOBILESIM environment variable to the MobileSim
source directory (or another directory containing MobileSim's resources).

-------------------
How MobileSim Works
-------------------

MobileSim glues together three main components: Stage simulation; Pioneer emulation; 
and various utilities from ARIA to load a map file and to send and
receive packets to and from clients.  The source file "main.cc" contains 
the main() function and brings these three components together, and provides 
a command-line interface and also an initial dialog box for loading a map 
file and selecting a robot model to create. The files RobotFactory.cc,
RobotFactory.hh, StageRobotFactory.cc and StageRobotFactory.hh implement
the "robot factory" feature (on-demand model creation).

Stage
-----

Stage is the core of the simulator. It provides the main GUI, and simulation of 
all the robots, devices, their environment, and collisions between them. 
Stage is not a MobileRobots project, but was created by a variety of individuals
and is related to the Player project.   Stage consists of a library, libstage.a,
which MobileSim uses to set up the simulated world, the GUI, and requested
robots and their devices. 

We have modified Stage to add some missing features and to allow it 
to be built in MSYS/MinGW.  This modified version of stage is included
in the source code package in the 'stage' subdirectory, and must be built
before building MobileSim.  (Patches are available on the Stage project
patch tracker at http://sourceforge.net/p/playerstage/patches/)

The source code to Stage is in the 'stage/src' subdirectory.  Simulation
of robots and devices ("models") are in files beginning with the "model_"
prefix. E.g. a movable robot base is a "position" model and is implemented
in "model_position.c".  Sonar is a kind of "ranger" model.  The laser LRF
is a "laser" model.  Stage begins by creating a "world" (MobileSim creates
a temporary world file for Stage to start with).  MobileSim then creates
models for the robots and devices, represented using stg_model_t structs
which store properties and other state.  These are initialized with an init
function for the appropriate type (e.g. position_init for position models,
ranger_init for ranger models, etc.)  which creates the model's property
objects, registers callbacks for future events, etc.  It then calls the
load function (e.g. position_load for position models, etc.) which reads
configuration information from the world.  Then, after the simulation has
been started, MobileSim's main loop calls Stage's main update function, which
calls the update functions for all model in the simulation (e.g. position_update
for position models) every 100ms (by default).  This update function can examine 
"command" properties and make appropriate changes, and otherwise maintain the 
model's state in the simulation.  Each model-specific function ends by calling 
a common update function that updates the model's position in the simulated world 
(based on its current velocity properties) and updates the GUI.

In setting up the world configuration for Stage, MobileSim relies on definitions of various 
Pioneer robot models, which are kept in a file called PioneerRobotModels.world.inc.  
When installed, this file is alongside the MobileSim program (e.g. 
/usr/local/MobileSim directory on Linux).  

More about Stage (including library API documentation) is on the web at 
<http://playerstage.sf.net>.


Pioneer emulation
-----------------

The Pioneer emulator component encapsulates the Pioneer robot specific aspects
of the MobileSim code. The central class in this part is called EmulatePioneer.
EmulatePioneer accepts command packets and sends information packets through a 
TCP port, the same way that a real Pioneer robot communicates over a serial port.  
These commands are de-marshalled and EmulatePioneer makes appropriate calls into 
a RobotInterface object.  Likewise, information is retrieved from the 
RobotInterface to be sent in information packets. RobotInterface is the 
interface through which the Pioneer emulation component communicates with the 
simulation component.   In MobileSim, a subclass of RobotInterface called 
StageInterface is used to connect the Pioneer server emulation with the Stage 
simulation.   The StageInterface class also contains a function to load the map 
file and set up the Stage world accordingly.

For each robot requested, MobileSim's main() function creates a 
StageInterface and an  EmulatePioneer object. The StageInterface object 
locates appropriate "model" pointers in the Stage simulation for the robot 
(a "position" model), sonar (a "ranger" model) and possibly a laser model.
EmulatePioneer uses a StageInterface object to communicate with Stage 
through it's public functional API or by getting and setting 
"properties" (a generic data container) of these models.


