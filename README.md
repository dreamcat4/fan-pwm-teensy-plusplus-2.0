# fan-pwm teensy++ 2.0

Combines other Arduino sketches, plus extra work. Towards a better fan pwm control, for teensy++ 2.0

<!-- MarkdownTOC -->

* [Setting up PlatformIO](#setting-up-platformio)
  * [VSCode](#vscode)
  * [Sublime Text 3](#sublime-text-3)
* [Credits](#credits)
  * [Arduino Sketch - Initial Code](#arduino-sketch---initial-code)
  * [High frequency pwm on ATMega 328p, by maipulating timer registers](#high-frequency-pwm-on-atmega-328p-by-maipulating-timer-registers)

<!-- /MarkdownTOC -->

<a id="setting-up-platformio"></a>
## Setting up PlatformIO

<a id="vscode"></a>
### VSCode

There are already many tutorials out there, for how to install PlatformIO extension within VSCode IDE. And get things working.

* Recommended Video - [#264​ PlatformIO for Arduino, ESP8266, and ESP32 Tutorial](https://www.youtube.com/watch?v=0poh_2rBq7E)

<a id="sublime-text-3"></a>
### Sublime Text 3

**Note:** You may need certain other specific tools to be installed manually. For example the Arduino IDE.

Follow these official instructions:

* https://docs.platformio.org/en/latest/integration/ide/sublimetext.html
* With multiple projects open.. be sure to `Switch PlatformIO Project Environment`. In the blue status bar at the bottom

In sublime text editor:

* Type: `CTRL+SHIFT+P` to bring up command window
* Type: `Package Control: Install Package`
* Wait for Package Control directory listings to get loaded up
* Type: `Deviot`, and hit `<RETURN>` key

Then open a terminal window. For example if you have the `Terminus` package installed then `CTRL+SHIFT+P` then `Terminus: Toggle Panel`.

In terminal, navigate into this project root folder `arduino-fan-pwm`, and then type:

```sh
pio project init --ide sublimetext --board uno
```

<a id="credits"></a>
## Credits

This is a combined effort, after watching many educational videos explaining PID control algorithm. An also after gathering multiple arduino sketches / examples from around the web. And merging everything together.

<a id="arduino-sketch---initial-code"></a>
### Arduino Sketch - Initial Code

* Name: Chris Barnes
* Url: https://barnesian.com/arduino-powered-smart-fan-controller/
* License Terms: No idea, looks Public Domain license maybe. But Copyright © 2011-2017 Chris Barnes
* Comment: Lol. Can run a diff to see which lines of the original code remains

<a id="high-frequency-pwm-on-atmega-328p-by-maipulating-timer-registers"></a>
### High frequency pwm on ATMega 328p, by maipulating timer registers

* Name: Nick Gammon
* Url: http://www.gammon.com.au/forum/?id=11504
* License: copyleft 2005 DojoDave <http://www.0j0.org>
* Comment: Difficult to understand / follow but... Awesome stuff! Thanks so much & absolutely essential





