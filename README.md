# rospo

This repo contains code to be used with Arduino Uno and STM32 Blue Pill to create a USB oscilloscope. It was written for use with the Rospo DIY Arduino oscilloscope shield (https://youtu.be/z2SUedz2vag), but it's not particularly unique to that hardware. You could condition the incoming signals with whatever hardware is appropriate for your application.

The code expects that the two analog channels to be captured will be on pins A2 and A3 (Uno) or A1 and A0 (STM32F103). It also expects to use output pins D2 and D3 (Uno) or B7 and B8 (STM32F103) to enable/disable a positive offset to these signals to allow for capture of signals that go below their ground.
