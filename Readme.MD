A Smart Controller for the AR-22 Rotator by WA2FZW
Version 5.4

The AR-22 rotator controller is an Arduino Uno based controller for the old
CDE AR-22 antenna rotator which replaces the clunk-clunk controller that the
original used.

The main features include the ability to turn the antenna in 1, 5 or 10
degree increments and the ability to control it with many of the rotator
control programs available such as the N1MM Rotor program.

The files provided here include the latest version of the software, a manual
describing how to build and use the controller and the Gerber files needed
to fabricate the 2 custom PCBs used.

Version 5.3

No changes to the software (Version 5.2 is the latest software version), but
Neal (WA9WYI) discovered a mistake on the power supply PCB; the pins on the
LM7812 regulator were reversed. The Gerber files for the power supply PCB
are corrected and identified as Version 5.3.

Document Update 9/4/2020:

Added information about the commands available on the serial interface. Not
sure why I never listed them in the original!


Update Jan 04, 2021 - Fixed a couple of typos in the docmentation and added
a "Table of Contents".


Version 5.4 - 2/24/2021

I made a number of changes to both the hardware and software and finally
fixed a problem that has been bugging me since the intial version!

Hardware Changes

In the hardware, I added more filtering on the power supply board and on
the controller board I added snubbing diodes across the relay coils (I
thought they were built into the relays; NOT) and added a bypass capacitor
in an effort to reduce the noise on the rotator pulse line.

One additional change on the controller PCB was to add a header connected
to the TX2/RX2 serial interface on the Arduino Mega. This is not used yet,
but I plan to add elevation control using a separate processor.


Software Changes

In the software, I made some changes to the timeout logic to deal with two
problems.

One problem this solves is the fact that there is some inertial delay when
the rotator initially turns on; the bigger the antenna, the more inertia!
The second problem was that sometimes the rotator was stopping with the cam
switch in the closed position which caused an immediate pulse upon the next
startup and the inertia delay was causing an immediate timeout.

Now when the rotator starts (or reverses direction), the cam pulse timeout
value is set to 2 seconds by the symbol INITIAL_TIMEOUT in the header file.
Once the controller has seen 2 cam pulses, the timeout value reverts to the
value of ROTATOR_TIMEOUT.

I also made a few changes in the code to improve performance and I added the
hooks for an interface to an elevation controller for my satellite antenna.

Note that the Version 5.4 software will run on the older hardware (Version
5.2 and later).
