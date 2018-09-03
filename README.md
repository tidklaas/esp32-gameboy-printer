# ESP32 Game Boy Printer

This project lets you rescue all those precious childhood pictures you snapped
with the Game Boy Camera. Simply use an [ESP32][1] module, a level shifter and
~~a WiFi enabled device with a web browser~~ your smartphone to save those
snaps before the camera's on-board battery fails.  
Alternatively, you could go and do some 2-bit street photography and earn some
serious Hipster Cred[tm] on Instagram.

1. Install and set up the [Espressif IoT Development Framework][2].
2. Study the [documentation][3]!
3. Clone this project and initialise it by calling
   ``submodule update --init --recursive``.
4. Run ``make defconfig``.
5. Run ``make erase_flash``.
6. Run ``make flash monitor``.
7. If you run into problems during steps 4 - 6, refer to step 2.
8. Connect the Game Boy to the ESP __using a 3.3V <-> 5V logic level shifter.__
9. Print away.
10. Connect your phone to the WiFi network GB-Printer.
11. Point your web browser to ``http://192.168.4.1``.
12. Revel in the 160x144x4 retro glory.
13. ???
14. Profit!


## How to Connect
Consult the [Game Boy Link pin-out](http://www.hardwarebook.info/Game_Boy_Link)
and connect SO to GPIO 13, SI to GPIO 4 and SC to GPIO 14. If you are not using
bi-directional level shifters, make sure that for SO and SC the signal
direction is from the Game Boy (GB) to the ESP and for SI it is from ESP to
the GB. You will also have to connect GND on both ends and both VDDs to the
respective reference voltage inputs.  
*N.B.* Do not try to power the ESP from the Game Boy. Most ESP32 modules out
there use inefficient linear regulators and can pull more than 500mA of
current.


## Usage
Once the ESP has received a complete image from the GB, it will encode it as a
PNG and store it in flash. Stored images can be viewed at and downloaded from
the HTTP server at address 192.168.4.1. Images are named ``imgxxxxx.png`` where
``xxxxx`` is a sequence number counting up from 0. New images are stored using
the first free number found, starting at the last index used.

On the web page there are two links above the displayed images.
``Delete all pictures`` will remove all stored images without resetting the
sequence counter. So, if you hit this link after printing images 0 to 22, the
next image printed will still be saved as ``img00023.png``. This should help
preventing accidentally overwriting older images when downloading newer ones.

``Reset sequence numbering`` resets the sequence counter to 0, so new images are
again saved at the lowest index available. So, for example, let us say you printed
images 0 to 22, then hit ``Delete all pictures``. Now you continue printing images
from 23 to 41. If you hit ``Reset sequence numbering``, the next image will be
saved as ``img00000.png``. If you keep printing, image numbers will go up until
they reach 22. Since number 23 to 41 are already taken, the next image will have
number 42.

## Limitations and Quirks
The code has only been tested against the Game Boy Pocket with the Camera  module
and might not work with other versions of the Game Boy or different game modules.
The compressed image transfer format is not supported at all. Without a way of
testing there was no point in implementing it.

Since the whole protocol is bit-banged, it is susceptible to delays caused by other
higher priority tasks running on the ESP. This means mostly the WiFi task, so avoid
connecting devices or accessing the web server while printing to the ESP.

Error detection has been designed to err on the side of caution, so the ESP might
signal a transfer error to the GB, even though the image has been transferred and
saved correctly. This was deemed preferable to silently losing data.

Due to the way filesystem access and caching directives are handled in
[libesphttpd][4], browsers might cache images that have been deleted and re-written
with different data. Do not be surprised to see unexpected images after deleting
all images and resetting the image sequence numbering. Clearing the browser cache
should fix this problem.

## Security Considerations
There is no security, only Zuul. Well, not even Zuul. Everything is accessible on an
open WiFi network and HTTP server. Do not use this to pull your d*ck pics from your
Game Boy Camera.

## Acknowledgments
* This Project was originally based on applefreak's [esp8266-gameboy-printer][5],
  but the only part that survived was the ``style.css``, so thanks for that.
* PNG encoding is done using Lode Vandevenne's [LodePNG][6].
* Web server functionality is provided by Chris Morgan's [ESP32 fork][4] of
  Jeroen Domburg's libesphttpd.

[1]: https://espressif.com/en/products/hardware/esp32/overview
[2]: https://github.com/espressif/esp-idf
[3]: https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html
[4]: https://github.com/chmorgan/libesphttpd
[5]: https://github.com/applefreak/esp8266-gameboy-printer
[6]: https://lodev.org/lodepng
