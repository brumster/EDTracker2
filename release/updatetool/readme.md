EDTracker Update Tool
---------------------

Version 1.0.3

Copyright Dan Howell 2014

PRE-REQUISITES
==============

- The Update Tool requires Oracle's Java 7 (or greater) runtime environment;
it should work with 32-bit and 64-bit versions.

- At the moment, the tool requires Microsoft Windows operating systems due to 
a reliance on the RXTX serial library, which includes a native DLL specific to
the Windows OS. A future release may include Linux compatibility.

- Your device must have the Arduino drivers installed and recognised. These
are available by downloading the Arduino IDE software ZIP from http://arduino.cc
and unpacking the "drivers" folder from within it.

RUNNING THE TOOL
================

1. Open a command prompt
2. Navigate to the directory where you unpacked the tool in step (1). e.g.

  cd c:\temp\updatetool
  
3. Run the tool with the following command :

  java -jar UpdateTool.jar <parameters>
  
Omitting parameters will display the help text, including a list of parameters.

RETRIEVING A LIST OF AVAILABLE IMAGES FOR FLASHING (-l)
==================================================
** Deprecated - no longer works as edtracker website no longer available **
** Use the flash from local file option detailed below instead           **
	
Run the command with the -l option. Provided an internet connection is available,
the tool will connect to the edtracker.org.uk website and retrieve a list of
available images and display them. The tool then exits.

FLASHING AN IMAGE INTO THE DEVICE (-f)
=================================

Run the command with the -f <RELEASE_ID> parameter. The release ID of available
images is displayed with the previous command. The tool will download the image,
verify it, and then attempt connection to the device before flashing the image
into it.

The tool does not verify the image following successful flashing into the device;
if you wish to do so, include the parameter -v.

DEBUG OUTPUT (-d)
============

Should it assist you or any member of the EDTracker team, you can specify the -d
command to enable additional debug information.

QUERY DEVICE (-q)
============

The tool can non-destructively query the Atmel bootloader on the device and
retrieve technical information about it's type. This may be useful in debugging
devices that refuse to accept an image.

SPECIFYING COM PORT (-c)
===================

By default, the tool assumes the highest COM port number is the EDTracker device. If
this is not the case, you can override the behaviour by specifying the exact COM
port upon which the EDTracker is configured. E.g.

	-c COM12
	
CONNECTING DIRECT TO BOOTLOADER (-b)
===============================

If your device is blank and has no flashed image in it at all, it may not present
itself as a device except *briefly* when it is plugged in. If this is the case, you
can use this facility to specify which COM port the bootloader is available upon.
The bootloader does not normally stay available for more than a second or so, so
it is important to issue this command immediately after plugging in the device.

1) Prepare the command but do not press enter. e.g.

	java -jar UpdateTool.jar -f EDTracker2 -b COM5

2) Plug your device into a USB port
3) Press enter to execute the command

FLASHING AN IMAGE FROM A LOCAL FILE INTO THE DEVICE (-i)
===================================================

Run the command with the -i <full_filename> parameter. Easiest option is to put your
binary image file into the same folder as the update tool. The tool will then
upload this binary image into the device. No internet connectivity is required for
this operation. It is provided as a debug option for users, but please verify the
source of your image file before using, as we cannot guarantee where it came from!

The tool does not verify the image following successful flashing into the device;
if you wish to do so, include the parameter -v.

LEGAL STUFF
===========

The EDTracker UpdateTool is placed under the MIT License

Copyright (c) 2014 Dan Howell / www.edtracker.org.uk

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.


RXTX binary builds provided as a courtesy of Mfizz Inc. (http://mfizz.com/).
Please see http://mfizz.com/oss/rxtx-for-java for more information.

RXTX is provided under the MIT License, full details of which are provided in the
LICENSE_RXTX.txt file.

