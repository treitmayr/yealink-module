# Yealink Linux Kernel Driver

## Table of Contents

1. [Introduction](#introduction)
1. [Status](#status)
1. [Compilation](#compilation)
1. [Keyboard Features](#keyboard-features)
1. [LCD Features](#lcd-features)
1. [Driver Usage](#driver-usage)
1. [Sound Features](#sound-features)


## Introduction

This document describes the driver for Yealink phones and ATA adapter.

## Status

### Phone Models

* USB-P1K  USB 1.1 hand phone with LCD, keypad and ringer
* USB-P1KH USB 1.1 hand phone with LCD, keypad and ringer
* USB-P4K  USB 1.1 speaker phone desktop model
* USB-B2K  USB 1.1 telbox, ATA adapter for PSTN
* USB-B3K  USB 1.1 telbox, ATA adapter for PSTN  
   (Caller-ID and PSTN-USB-bridge currently not supported)

### Kernel Versions

Currently kernel versions >= 2.6.18 and <= 6.1 are supported.

### References

For manufacturer documentation might still be found at [http://www.yealink.com/](http://www.yealink.com/)
but due to the dated devices this may not be true anymore.

The original development branch can be found at:  
[https://cvs.savannah.gnu.org/viewcvs/usbb2k-api/yealink-module/](https://cvs.savannah.gnu.org/viewcvs/usbb2k-api/yealink-module/)

The updated sources supporting the P1KH and B3G can be found at:  
[https://github.com/treitmayr/yealink-module](https://github.com/treitmayr/yealink-module)

### Matrix of Supported Features

| Function | API | P1K(H) | P4K | B2K | B3G |
| -------- | --- | ------ | --- | --- | --- |
| audio playback | alsa | ok | ok | ok | ok |
| audio record | alsa | ok | ok | ok | ok |
| keyboard | input | ok | ok | ok | ok |
| hookflash | input |  | ok | ok | ok |
| PSTN ring | input |  |  | ok | ok |
| LCD |  | sysfs | ok | ok |
| LED |  | sysfs | ok |
| DIALTONE | sysfs |  | ok | ok | ok |
| RINGTONE | sysfs | ok |  | ok | ok |
| BACKLIGHT | sysfs |  | ok |
| SPEAKER |  | sysfs |  | ok |
| PSTN |  | sysfs |  |  | ok | ok |
| LED (USB/PSTN) | sysfs |  |  | ok | ok |
| Caller-ID |  |  |  |  | wip |
| PSTN-USB-Bridge |  |  |  |  | wip |

*Notes:*  
P4K: Ring tones are implemented by switching on the SPEAKER
and sending a ring tone pcm via the dsp interface.


### Userspace Applications

#### Yeaphone

*Warning:* Yeaphone is outdated and not maintained anymore!

Thomas Reitmayr has released Yeaphone, a great addition to linphone, this
will turn your P1K(H) into a full blown SIP phone. There are even packages
for the Linksys NSLU2 for an energy efficient and fanless solution!

Check it out at [https://www.devbase.at/past-projects/yeaphone](https://www.devbase.at/past-projects/yeaphone).


## Compilation

In order to build the yealink.ko kernel module, just invoke

```
make
```

Note that it should not be necessary to install or build a full-blown Linux kernel source tree!


### Troubleshooting

Q: The phone is working (displays version and accepts keypad input) but I cannot find the sysfs files.  
A: The sysfs files are located on the particular usb endpoint. On most
   distributions you can do: `find /sys/ -name get_icons` for a hint.


## Keyboard Features

Keyboard events are processes through the input layer. A user space application
may issue the `EVIOCGRAB` `ioctl()` on the corresponding `/dev/input/eventX` device
to prevent the key codes from the event device to go to the rest of the system.

The current mappings from scancode to input event are provided by the
`map_p1k_to_key`, `map_p4k_to_key`, ... functions.
See `yealink.c` for a description.


### P1K Keyboard Layout

```
Physical USB-P1K Button Layout          Input Events

             up                             up
       IN           OUT                left,   right
            down                           down

     pickup   C    hangup         enter, backspace, escape
       1      2      3                    1, 2, 3
       4      5      6                    4, 5, 6,
       7      8      9                    7, 8, 9,
       *      0      #                    *, 0, #,
```

  The "up" and "down" keys, are symbolized by arrows on the button.
  The "pickup" and "hangup" keys are symbolized by a green and red phone
  on the button.

### Hookflash

The B2K, B3K, and P4K generate KEY_PHONE up and down events when the phone
taken off- or on-hook.

### PSTN Ring

The B2K and B3K report each individual ringtone on the PSTN line with KEY_P
down (start of tone) and up (end of tone) events.


## LCD Features

The models P1K(H) and P4K feature an LCD which is divided and organized as
a 3 line display:

```
    |[]   [][]   [][]   [][]   in   |[][]
    |[] M [][] D [][] : [][]   out  |[][]
                              store

    NEW REP         SU MO TU WE TH FR SA

    [] [] [] [] [] [] [] [] [] [] [] []
    [] [] [] [] [] [] [] [] [] [] [] []
```

```
Line 1  Format (see below)  : 18.e8.M8.88...188
        Icon names          :   M  D  :  IN OUT STORE
Line 2  Format              : .........
        Icon name           : NEW REP SU MO TU WE TH FR SA
Line 3  Format              : 888888888888
```

**Format description**

From a user space perspective the world is separated in "digits" and "icons".
A digit can have a character set, an icon can only be ON or OFF.

* Generic format specifiers
  * `8` - Generic 7 segment digit with individual addressable segments

* Reduced capability 7 segm digit, when segments are hard wired together.
  * `1` - 2 segment digit only able to produce a 1.
  * `e` - Most significant day of the month digit, able to produce at least 1 2 3.
  * `M` - Most significant minute digit, able to produce at least 0 1 2 3 4 5.

* Icons or pictograms:
  * `.` - For example like AM, PM, SU, a 'dot' .. or other single segment elements.


## Driver Usage

### Overview

#### sysfs interface

For userland the following interfaces are available using the sysfs interface:

| sysfs entry | access | description |
| ----------- | ------ | ----------- |
| `line1` | read/write | LCD line 1 |
| `line2` | read/write | LCD line 2 |
| `line3` | read/write | LCD line 3 |
| `get_icons` | read | returns a set of available icons |
| `hide_icon` | write | hide the element by writing the icon name |
| `show_icon` | write | display the element by writing the icon name |
| `map_seg7` | read/write | the 7 segments char set, common for all Yealink phones. (see map_to_7segment.h) |
| `ringtone` | write | upload binary representation of a ringtone for P1K(H) models, see yealink.c. |
| `model` | read | returns the detected phone model |

#### Module parameters**

None.

### lineX

Reading `/sys/../lineX` will return the format string with its current value`

Example:  
```
cat ./line3
888888888888
Linux Rocks!
```

Writing to `/sys/../lineX` will set the corresponding LCD line.
 - Excess characters are ignored.
 - If less characters are written than allowed, the remaining digits are unchanged.
 - The tab '\t'and '\n' char does not overwrite the original content.
 - Writing a space to an icon will always hide its content.

Example:
```
date +"%m.%e.%k:%M"  | sed 's/^0/ /' > ./line1
```
This will update the LCD with the current date & time.


### get_icons

Reading will return all available icon names for the detected model and its current settings:

```
cat ./get_icons
on M
on D
on :
   IN
   OUT
   STORE
   NEW
   REP
   SU
   MO
   TU
   WE
   TH
   FR
   SA
   LED
   RINGTONE
```

### show/hide_icons

Writing to these files will update the state of the icon.
Only one icon at a time can be updated.

If an icon is also on a ./lineX the corresponding value is updated with the first letter of the icon.

Example - light up the store icon:
```
echo -n "STORE" > ./show_icon

cat ./line1
18.e8.M8.88...188
             S
```

Example - sound the ring tone for 10 seconds:
```
  echo -n RINGTONE > /sys/..../show_icon
  sleep 10
  echo -n RINGTONE > /sys/..../hide_icon
```


### model

This file can be read to print the current phone model.

Example - show the current phone model:
```
cat ./model
P1K
```

## Sound Features

Sound is supported by the generic ALSA driver `snd_usb_audio`.

One 16-bit channel with sample and playback rates of 8000 Hz is the practical
limit of the devices.

Example - recording test:
```
arecord -v -d 10 -r 8000 -f S16_LE -t wav  foobar.wav
```

Example - playback test:
```
aplay foobar.wav
```

## Credits & Acknowledgments

See [yealink.c](yealink.c).
