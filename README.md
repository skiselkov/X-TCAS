# X-TCAS

This is a generic implementation of TCAS II v7.1 designed for use in
flight simulators. This is not a generic plugin intended for end users.
Instead, it is designed to be used by aircraft model designers to be
integrated into their simulated avionics package.

## Donations

To leave a voluntary donation, please follow the PayPal link below:

[![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=8DN9LYD5VP4NY)

## Building X-TCAS

X-TCAS currently supports two build types:

1. As a stand-alone text-only mode for testing. This can be invoked by
   setting the following CMake variable: `cmake -DBUILD_STANDALONE_TEST=1 .`.
1. As a plugin for the X-Plane 10 and X-Plane 11 simulators. To invoke a
   build of this type, run: `cmake -DBUILD_STANDALONE_TEST=0 .`.

A standalone test build runs from the command line and outputs an
ASCII-art display of the horizontal situation surrounding the aircraft.
It also requires a scenario "command" file, which defines which aircraft
are involved in an encounter and what maneuvers they will perform. See
`src/xtcas_test.c` for details.

The embeddable version for X-Plane currently supports either displaying
a test overlay in the simulator on the screen, or integrating into the
FlightFactor 320 Ultimate aircraft. Contact the author to add support for
more aircraft, as doing so requires special integration code to work with
the aircraft's electronic flight information system (EFIS) displays.

## Disclaimer

This project is intended strictly for *HOME ENTERTAINMENT* use only! Do
**NOT** use X-TCAS for any serious flight training or real avionics. It
has not passed any certification tests and might be wildly inaccurate.
