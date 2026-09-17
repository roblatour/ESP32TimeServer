## Setup - **Linux (Debian 13)**

**Setup Step 0 - Prerequisites**

There are a few things you need before you can begin to set up your ESP32TimeServer under Linux.

First, remember in Linux, everything is case-sensitive: Hello, hello, HeLlO, hellO are 4 completely different items.  This is very important when using the command line.  If something does not work, check to make sure you have the case correct.

When you installed ESP-IDF and associated Tools, Python should have been installed as a prerequisite.  Note that for Linux, you are installing the ESP-IDF Installation Manager (EIM) when you follow the instructions at the ESP-IDF Linux link.  Unless you are on a system that does not have a graphical environment installed, install the version with both GUI and CLI.  Before you can use ESP-IDF (and it's tools) you need to set up the environment for your user.  This is where the GUI comes in handy.  After installing EIM, run it, and choose 'Start Installation' under 'New Installation'.  Easy Installation is recommended - it will download and install all the necessary components and set up the Python virtual environment properly.  Kick it off, grab a coffee, and wait for it to complete. When it is done, you can close the EIM window.  These instructions assume you did the default install, which puts the installation in ~/.espressif/v6.1/esp-idf/.  If you chose to put it elsewhere, you will need to know the path and make the appropriate changes to the compile and flash commands later in these instructions. For the installation examples here, the username is 'user' and the host is 'shop'.

While configuration and flashing of the ESP32 Time Server does not require root privileges, your user does have to be a member of the dialout group.  This is the group with permissions to the USB Serial interface on the ESP32.  If you try to flash the ESP32 Time Server and get an access error - doublecheck your group membership:

```
grep -i dialout /etc/group
```
```
dialout:x:20:user  <- You should see your username here!
```

Please, for the love of all that is Linux, do not run as the root user or run everything under root privileges.  It is a very bad idea(tm)!  Have your user added to the dialout group.  It makes life easier (and safer).

In addition to ESP-IDF and it's associated tools (and dependencies), you will need GIT installed on your system.

Once you have met these requirements, you are ready to go forth and conquer!

**Setup Step 1 - Download**

Create a working directory and enter it:
```
mkdir ESP32TimeServer
```
```
cd ESP32TimeServer
```

Clone the git repository:
```
git clone –recursive https://github.com/roblatour/ESP32TimeServer
```

Change into the cloned repository directory:
```
cd ESP32TimeServer
```

Perform a submodule update:
```
git submodule update --init --recursive
```

**Setup Step 2 - Configuration**

This is the same for Linux and Windows.  Edit using a text editor either from the command line (vi, vim, emacs, nano, pico, etc.) or from the gui (gedit, etc.)

**Setup Step 3 - Build**

Before you can build, you must set up the ESP-IDF environment for this project.

Install ESP-IDF for this project:
```
~/.espressif/v6.1/esp-idf/install.sh
```

You must source the export.sh script, you cannot simply execute it.
```
source /home/user/.espressif/v6.1/esp-idf/export.sh
```

The message about outdated tools in the system is not a concern, unless you are getting low on disk space.  Then, follow the instructions in the message.

Remove the existing sdkconfig file. This will throw an error if the file is not there, it is safe to ignore the error.
```
rm sdkconfig
```

Use the appropriate build statement, based on your specific ESP32-O4 module:

For older ESP32-P4 modules with revisions prior to version 3.0 (including the Waveshare ESP32-P4-ETH):
```
- idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;config/esp32p4_rev_pre_v3.defaults" set-target esp32p4 build
```

For ESP32-P4 modules at revision 3.0:
```
- idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;config/esp32p4_rev_v3_0.defaults" set-target esp32p4 build
```

For ESP32-P4 modules at revision 3.1 and above (including the WaveShare ESP32-P4-WIFI6-POE-ETH):
```
- idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;config/esp32p4_rev_v3_1.defaults" set-target esp32p4 build
```

**Setup Step 4 - Flash**

Before you can flash the ESP32, you need to find out what port Linux knows it as.  /dev/ttyACM0 is generally a good assumption, but it is also easy to find.  Look in the /dev directory for devices that are in the dialout group.  Devices that start with ttyS are serial ports, you probably do not want those.  This example is from my system:
```
ls -al /dev | grep -i dialout

crw-rw----   1 root dialout 166,     0 Sep 15 21:34 ttyACM0
crw-rw----   1 root dialout   4,    64 Sep 13 22:03 ttyS0
crw-rw----   1 root dialout   4,    65 Sep 13 22:03 ttyS1
crw-rw----   1 root dialout   4,    66 Sep 13 22:03 ttyS2
crw-rw----   1 root dialout   4,    67 Sep 13 22:03 ttyS3
```

/dev/ttyACM0 is the port on my machine.  We will use this for flashing.  Note that this command does two things - first, it flashes the code to the ESP32-P4.  Second, it opens a monitor so you see console messages when the code executes after flashing.
```
idf.py -p /dev/ttyACM0 flash monitor
```

The monitor is opened when you see this line:
```
Executing action: monitor
```

Shortly after, you will begin to see the console messages.  These lines start with an I (informational), W (warning), or E (error) based on the message type.  Not all error messages are fatal - if you enable MQTT but do NOT install a micro-SD card, you will see errors when the system attempts to initialize the (non-existent) card.  This does not affect operation, other than the MQTT queueing, as described elsewhere.

When you see a line similar to:
```
I (585) main_cpp: ******************* Application Startup *******************
```

this indicates the ESP32-P4 has booted and is now running the code flashed to it.  Unless you have DEBUG enabled, you will eventually see lines similar to:
```
I (53834) main_cpp: ***********************************************
I (53835) main_cpp: * Open for business: 2026-09-16T20:51:20-0400 *
I (53836) main_cpp: ***********************************************
W (53841) main_cpp: DEBUG_ENABLED is disabled in the settings file; this will be the last console message from main_cpp
I (53852) main_task: Returned from app_main()
```

Congratulations.  Your NTP server is up and running!
