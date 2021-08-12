# app-fsk

This app implements the logic to allow an Asterisk dialplan application to send and receive binary data using only voice channels.
It exploits the old-time analog modulation [FSK](https://en.wikipedia.org/wiki/Frequency-shift_keying), just like the old analog modems did.
After having it compiled, it adds a couple of functions **sendFSK** and **receiveFSK** to the asterisk dialplan functions.


**BEWARE**

It is an old project, and it was only tested on asterisk 11.
