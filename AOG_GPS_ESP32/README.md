# esp32-dual-gps
Software to adjust the Dual GPS settings on the ESP32 which is sending position, heading, and roll to AgOpenGPS. Reuses code for position, heading, and roll written by MTZ8302

# Features
* Dual and single GPS compatible
* settings in blue widgets have immediately response
* Serial NMEA out using SoftwareSerial
* Velocity PWM out for 3rd party rate controllers, etc
* Pick AOG messages including PAOGI, GGA, VTG, HDT
* debug code using USB debug options
* uses UDP to send GPS info to AgOpenGPS


* has settings for:
    1. distance between GPS antennas
    2. GPS antenna height
    3. GPS offset foreword
    4. GPS offset right
    3. GPS heading angle correction for left/right antennas
    4. Distance between antenna max deviation
    5. Max heading change in degrees/second
    6. Check UBX flags
    7. Filter position on weak GPS signal
    8. Correct position with roll

# Caveats
* For best results, use the Dual Gps board found in the [Github repo](https://github.com/AOG-addendum/PCB-modules/tree/master/Dual%20GPS)
* **As the WebUI is quite taxing on the Websocket implementation; the ESP32 can crash without warning if left too long connected. Without open connection to a browser, no such crashes are documented. On my hardware, a longtime stresstest of more the five days of uptime was completed successfully, if the tab with the WebUI is closed after using it.** The cause of the crashes is in the implementation of the used TCP-stack/Websocket-API. Not much can be done about it, as it is the default implementation which comes with framework (ESPAsyncWebServer), is really fast/performant and is used by the library to generate the WebUI.

# Schematics

![Schematics](doc/schema.png)

The configuration has to match the connections for the ESP32 and the F9P. This schematics is with ESP32 modules from Espressif and Adafruit, but you can use other brands, if you adjust the firmware for it. This is usually done by changing the `board` in `platformio.ini`. 

An example of a itemlist is below:

Amount |       Id          | Supplier  | Description
-----  | ----------------- | --------- | ----------------------------------------------------------------------------
1x     | ESP32-DEVKITC-32  | Espressif | DevKitC Board
1x     | 4172              | Adafruit  | Huzzah32 ESP32 for RTK correction
2x     | GPS-15136         | Sparkfun  | U-Blox F9P
1x     |                   | PCB       | PCB from [here](https://github.com/AOG-addendum/PCB-modules/tree/master/Dual%20GPS) name: DualGPS Rev x.fzz; use latest version

## PCB module
All docs you need should be in the Dual GPS PCB repo. If parts are missing in the Readme, you can install Fritzing and open the .fzz file, the parts should be listed with the proper values.

## Windows
Follow this [guide](http://iot-bits.com/esp32/esp32-flash-download-tool-tutorial/), but enter the files/addresses as below:

Address   | File
--------- | ----------------------
`0x1000`  | `bootloader_dio_40m.bin`
`0x8000`  | `partitions.bin`
`0xe000`  | `boot_app0.bin`
`0x10000` | `firmware.bin`

Tick the option to enable the flashing of these files and press on `Start`. The defaults should work, but depending on your version of the ESP32, other settings are necessary, which you can find in the documentation of the manufacturer.

## Linux / command line
Install `esptool.py`, preferably with the packet management system of your distribution. Open a new terminal window and change to the directory with the already decompressed archive. Then enter:
```
esptool.py --chip esp32 --before default_reset --after hard_reset write_flash --flash_size detect 0x1000 bootloader_dio_40m.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 firmware.bin
```

## OTA update
If you already have flashed a version of it on your ESP32, you can navigate to the update page by clicking on the link in the last tab. Then upload the file `firmware.bin`. Attention: this firmware uses a custom flash-partition-layout, so it has to be flashed once by a flasher as described above. Using another OTA flasher (like an OTA example) doesn't work.


# Installation
TL;DR: If you want to do your own development, you have to install [Visual Studio Code](https://visualstudio.microsoft.com/), clone the repository with all the submodules, build and upload it.

## Windows
### Warning before you start
Read this file through, before starting to half-ass it. It is not so hard to get a working system, just give it enough time and install it in this order.
Some packets take a real long time to install with no visible progress. Just wait until finished.

### Install Prerequisites
1. install Visual Studio: https://visualstudio.microsoft.com/
1. inside VS Code:
   1. click on "Extensions" left hand side
  12. search for and install platformIO
   1. open the platformio home tab (if not opened automatically, use the menu)
      1. go to "Platforms" (left side tile)
      1. choose the "Embedded" tab
      1. install the "Esspressiv 32" platform
1. install all needed drivers for your platform. This is usually done by installing the CP210x-driver, but consult the documentation of the manufacturer of your esp32.

### Downloading the repository
1. open a folder in the explorer, preferably not too deep inside the drive. `C:\` or a folder under it should work
1. right click on it and choose "Git Bash Here"
1. enter `git clone --recursive https://github.com/eringerli/esp32-aog.git`

### Compiling
1. open the created folder above by right clicking the folder and "Open with Visual Studio Code"
1. click build (the tile with the tick), the missing dependencies should be installed automatically

### Upload to the ESP32
1. connect the ESP32 over USB
1. click on upload (the icon with the arrow in the bottom blue bar)

Alternatively you can use the OTA-update in the WebUI: go to the last tab and upload a new image.

## Linux
Install `platformio` with the package management system of your distribution and find a guide to help you set it up for your particular editor/IDE.

To do it on the console, clone the repository with all the submodules (`git clone` with `--recurse-submodules`), then change into the cloned folder and enter:
```
platformio run -t upload
```

This takes care of all the required libraries and uploads it to a connected ESP32.

## Configuration

All configuration is done in the WebUI. To connect to the created hotspot of the esp32, using a mobile device is normally the simplest solution.

To open a web page of the ESP32, type 192.168.xxx.79 into your web browser if using a dedicated router Wifi, or 192.168.1.1 if using the built in hotspot on initial setup. Alternatively, connect the ESP32 to the USB and open a monitor on it. It should print the SSID/IP address when booting.

After login in to the WebUI, you can then change the wifi to whatever you like. The esp32 tries to login, and if that fails, makes a new hotspot with the ssid Dual GPS XXXXXX using the Mac address to differentiate between different ESP32s. 

**The configuration is saved as long as there is no complete clearing of the flash of the esp32.** To  reset to the defaults, you either press the button in the WebUi or erase the flash of the esp32 with platformio/your editor. A new upload of the firmware doesn't affect the saved settings. 

## Updating the repository
As there are sometimes new repositories/submodules added, enter the following to update:
1. go to the esp32-aog repository and open a git bash-terminal
1. `git pull`
1. `git submodule sync`
1. `git submodule init`
1. `git submodule update`

Repeat them as needed.
