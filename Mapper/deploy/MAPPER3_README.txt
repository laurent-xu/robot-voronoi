	----------------------
	MobileRobots - Mapper3
	----------------------

2.2.5    February 12, 2009

Copyright (c) 2004, 2005 ActivMedia Robotics, LLC.
Copyright (c) 2006 - 2009 MobileRobots Inc
All rights reserved.

The license for use of this software is described in the LICENSE.txt file.

Mapper3(TM) is a graphical-user interface (GUI) application for 
the display and editing of MobileRobots map files.  Maps are used
by ARNL, SONARNL, MOGS, MobileSim, and other software.

There are two variants of Mapper3. Mapper3 has the ability to import
laser scan data and process it, resulting in a corrected map.
Mapper3-Basic does not have this ability.


                             Getting Started
                             ===============

To launch Mapper3 in Windows, choose 
    Start menu->All Programs->MobileRobots->Mapper3->Mapper3 
or double-click its icon in that directory or elsewhere, such as on your
desktop.

In Linux, choose Mapper3 from the Gnome or KDE menu if available, 
or enter /usr/local/bin/Mapper3 at the command line.

Basic Operation
===============

Mapper3 consists of a main menu, one or more panes, and a toolbar 
area with tools for displaying and editing the map.  The primary pane is 
the map pane, which contains a view of the current map.  The toolbar 
buttons provide quick access to common functions.  To see a more 
detailed explanation of a toolbar button, move the mouse pointer over 
the button and a description will appear after a brief moment.

Most editing operations in Mapper3 can be undone by selecting Undo 
from the Edit menu, or by using the Undo button on the toolbar.

Files
=====

To open a file in Mapper3, choose the Open... item from the File 
menu and select the file to be edited.  You may open either a map file 
or a laser scan file (.2d file).  If you choose a map file, it will be 
read and displayed in the main map pane and the toolbars will be enabled.  
Alternatively, you can choose a laser scan file in order to create a new 
map.  This process is described below in the Creating a Map section.

If a robot is running software with network map transfer services included
(e.g. arnlServer on Linux), you can also download that map and edit
it immediately. Choose Open on Robot from the 
File menu, and select the robot. If the robot is not listed, choose 
Select Robot.   To upload a map to the robot, use "save on robot".
Note that not all robot server software supports map upload/download.

The File menu also contains options for saving the map, and a list of
recently edited maps.  

Note that you can open more than one map file at a time.  By default, 
only one map will be visible.  The Windows menu contains a list of all 
opened files, and enables you to switch between them and change their 
layout.


Map 
===

Zoom and pan the map contents to get the best view by using the toolbar 
buttons, the mouse, or the keyboard. These are not exclusive; you may 
pan with the mouse and zoom from the keyboard, for example.

To pan the map contents with the mouse, click and hold the right button 
and drag the mouse. Zoom in and out by rotating the mouse wheel. If you 
do not have a mouse wheel, hold the Shift key and right-click to zoom 
in, or hold Shift and left-click to zoom out, or use the toolbar zoom 
buttons.

Also, when the map is in focus, you may pan it with the left, right, 
down, and up arrow keys.  To zoom, hold the Shift key and press the 
up-arrow key (in) or down-arrow key (out).

At any time, you may use the Fit in Window toolbar button or Map menu 
item to reset the map view and make all of the map data visible.


				
                               Creating a Map
                               ==============

Map files are initially created by converting *.2d laser scan files, which 
are produced by sickLogger, arnlServer, or guiServer.  Mapper3 imports and converts scan files 
into the map format.

Mapper3-Basic does not include this ability.


Importing Laser Scans
=====================

To convert a scan, begin by opening the .2d scan file from the Open... 
item in the File menu.  An empty map pane that only contains a red robot 
icon will be displayed, and the Scan Tools "floating" toolbar will 
appear.  Processing of the scan should start automatically, but if it 
does not, then click the Start button on the Scan Tools toolbar. 

Mapper3 processes the raw scan data by retracing the path of the 
robot that was used to capture the scan.  As the path is 
retraced, new data points appear in the map pane.  Once this phase is 
complete, Mapper3 filters and registers the scans.  Depending on 
the size of the scan, this can be a lengthy process.  When it is 
complete, the Save As... dialog appears, allowing you to save the new 
map file.  

After saving the map file, click "Finish" in the Scan Tools toolbar.  The
new map file will be opened automatically and the draw toolbar buttons 
will be enabled so that you can edit the map.

At any time, you can pause and resume processing of the scan with the 
Pause button, or cancel it with Finish.  Advanced scan preferences are 
available from the Settings button.



                                Editing a Map
                                =============

Drawing Tools
=============

There are three groups of tools for modifying the map by drawing:

   -- The Select Tool is used edit existing objects; it is the default 
      tool that is on when Mapper3 is first started.
   -- Object Tools are used to add new objects to the map.
   -- The Eraser Tool is used to erase the map data points and lines.

These are described in greater detail below.

Each toolbar button is a toggle button that may be turned on or off.  
Only one of the buttons may be on at any given time; this button will 
remain on until a different button is turned on.

To enable a tool, click its button in the toolbar. To apply the enabled 
tool, click on the map pane. Some tools require click and drag to place 
objects.


Select Tool
-----------

The Select Tool is used to pick objects that already exist on the map.  
As you move the mouse over an object, the object will be highlighted and 
surrounded by a dark blue rectangle.

Left-click on an object to select it.  A magenta rectangle with "resize 
handles" (small filled rectangles at the ends or corners) is displayed 
on the object.  To relocate the selected object, hold the left mouse 
button down and drag the mouse.  To reshape or reorient the object, 
position the mouse so that the select cursor is above a corner resize 
handle and then press the left mouse button and drag the mouse.  Use the 
same technique to rotate or change the heading of simple poses on the 
map.  If the pose should not have a specific heading (and this is 
permissible for the object type), then drag the resize handle to the 
center of the pose.

To edit the properties of a selected object, right click on the object 
and choose Properties. Or, double-click on the object.

To delete an object, right click and choose Cut. Or, use the Cut item 
in the Edit menu, or the Delete key.

Multiple objects may be selected at once by positioning the mouse on the 
map background so that no object has focus.  Press the left mouse button 
and drag the mouse.  A green dashed rectangle will appear.  Any object 
that is contained within the rectangle will be selected.  When multiple 
objects are selected, the resize and edit features are not available, 
but the selected objects can be moved and deleted.

   Note: Only map objects (not scanned obstacle data) will be selected 
   when the mouse is dragged.  If you wish to select the map data lines, 
   then press the Ctrl key before pressing the left mouse button and 
   hold it down while dragging the mouse.


Object Tools
------------

The Draw Toolbar contains a button for each type of object that may be 
added to the map.  These objects include goals, charging docks, 
forbidden lines and areas.  Depending on your robot's capabilities and 
loaded map, some specialized objects may be grouped together under the 
generic Advanced Areas and Advanced Lines buttons.  To access these 
objects, click the drop down menu indicated by the arrow button that is 
next to the Advanced button.

To add an object to the map, first turn on its button.  The cursor on the 
map changes to reflect the object type.  A small plus sign within the 
cursor indicates the exact "hot spot" where the drawing will be started.

Move the mouse to the desired location in the map and press the left 
mouse button down.  Hold the button down and drag the mouse to reshape 
or reorient the object.  Release the mouse button when the object is 
complete.  If the object type requires additional information (such as a 
goal name), then the edit box will immediately be displayed for the 
object.

If you change your mind and wish to cancel an object as it is being 
added, then simply drag the mouse outside of the map pane so that the 
object is displayed as a shadow.  Release the mouse button to complete 
the cancellation.


Eraser Tool
-----------

The Eraser Tool is used to erase obstacle points and lines on the map.  
It can be used to remove transient obstacles that were temporarily 
present when scanning your operating area, but which the robot should not 
expect to find when operating there in the future. When the Eraser Tool 
is turned on, the map cursor will look like a pencil eraser.  The small 
plus sign indicates the center point of the eraser, and a gray outline 
rectangle indicates the area that will be erased.  (If you cannot see the 
gray rectangle, then you may need to zoom in.)

Move the mouse to the area to be erased and press the left mouse button.  
All points and portions of lines under the eraser square will be erased.  
Continue to hold the mouse button down and drag the mouse to erase more 
points.

The size of the eraser rectangle is based in millimeters.  To change the 
size, click the drop down menu indicated by the arrow next to the eraser 
button.  Since the eraser is primarily intended for map "touch-ups", its 
maximum size is one meter.

  Note: Only scanned obstacle data can be erased with the eraser tool. To
  delete individual objects, select them with the select too, and press 
  the Delete key or choose Cut from the Edit menu.


Snap Mode
---------

If enabled, Snap mode restricts positioning of objects to regularly spaced 
points on a grid.  It restricts lines to 0, 45, or 90 degree angles.


Lines
-----

Obstacle lines may be added to a map with the Line tool.  SonARNL and 
MobileSim use Lines instead of scan points as obstacles.



Inserting Map Segments
======================

You may insert portions of other maps into the current map. This can be 
used to combine multiple scans into a bigger map, or to replace a poorly 
scanned map area with better data.

Choose Insert Map... from the File menu, and then choose the source map 
file for the new portion to be inserted.  The map insert will be 
displayed on top of the current map, but with blue data points and a 
translucent blue background.  A "floating" toolbar that contains the 
Insert Map tools will also appear.  

First, position the map insert at the desired location on top of the 
current map.  You can either drag the blue region with the mouse, or you 
can use the positioning buttons in the Insert Map toolbar. For precise 
adjustments, it is helpful to zoom into the map.

It is important to note that, during the insertion process, all of the 
current map data points and lines that are beneath the translucent blue 
area will be erased.  If the blue area is too big, then you can define 
smaller regions to be inserted by clicking Region on the Insert Map 
toolbar.  The blue background will change to shaded gray.  Drag the mouse
to draw new blue rectangles over the area that should be inserted. 

When you are finished positioning and selecting a region, click "Insert" 
on the Insert Map toolbar, and the existing obstacle data in the current 
map will be replaced by the inserted map's data.

    Note: All objects (such as goals and forbidden lines) will remain in 
    the current map.



                                Notes
                                =====

Toolbar Customization
=====================

The first time you start Mapper3, all of its toolbars and buttons 
appear in the display. Thereafter, you may customize the display for the 
current and future sessions.

Right-click anywhere in a toolbar or in the main menu bar to activate a 
pop-up menu of display options. To hide an entire toolbar, click to 
deselect its checkbox.  Or click again to re-select it for display. To 
hide an individual toolbar button, select the pop-up Customize... option. 
Then, in the Customize Toolbar dialog's Current Tool Button list, select 
the buttons you want or don't want displayed and use the Add and Remove 
buttons to hide or display them.


Additional Help
===============

If you need additional information about using Mapper3 or your 
robot, consult any additional documentation that came with your robot.  
If you have any questions or problems unanswered in that documentation, 
contact MobileRobots user groups or support team at http://robots.mobilerobots.com




--
$Id: MAPPER3_README.txt.template,v 1.1.4.1 2009-01-19 18:03:52 kathleen Exp $
