Change log
==========

1.7.0 (2019-04-09)
==================

* API changes:
  * Using c++ 14 features: **Ubuntu 16.04 or higher required**
* Deprecated features:
  * None
* New features:
  * DigitInputs: added "quick" click feature to emulate da Vinci
  * QtWidget: added confirmation dialog before resetting encoders from pots
* Bug fixes:
  * When loading XML, set scales to 1 if not provided
  * Set valid flag and timestamps for button events


1.6.0 (2018-05-16)
==================

* API changes:
  * All classes with `osa` and `mts` levels have been merged to simplify code
* Deprecated features:
  * Since all `osa` classes have been removed, programs that used the `osa` level need to be updated.  See for example `applications/current-calibration` 
  * Firmware Rev 3 and below are not supported anymore
* New features:
  * Use FPGA based velocity with firmware Rev 6
  * Application exists when the configuration fails, it makes it easier to find configuration errors
  * Setting encoder/pot error to 0 disables the check completly (e.g. MTM last two joints)
  * Reports 3 different timing statisctics, time to "read", "write" and overall.  These are also exposed using ROS topics
* Bug fixes:
  * Interval statistics were base on the "Read" phase only, it was missing the PID computing time as well as "Write" to IO 
  * Qt widget: brake control requires "direct control"

1.5.0 (2017-11-07)
==================

* API changes:
  * SafetyRelay now uses bool, was unsigned int
* Deprecated features:
  * Qt widget used to continuously set watchdog to prevent timeout, now moved to AmpIO
  * Removed some old ROS code, see dVRK ROS for current features
* New features:
  * Much improved current calibration utility, calibrates current feedback and requested.  Please recalibrate for all your arms!
  * Potentiometer configuration (tolerance and delay) now saved per arm in XML config file.  This allows settings per arm!
  * Single coupling matrix (position actuator->joint) need, all others (3) are computed from primary one
  * Added read command for watchdog timeout
  * Added read command for interval statistics (ROS conversion available too)
  * Support for firmware based velocity estimation (requires rev 6)
  * Check if power is lost because user requested, better message if unexpected power loss
  * Throttle port read error messages
  * Added console warning if not compiled in Release mode
  * Added console warning if old firmware is found (<5)
  * Use mtsMessage for status/warning/error in provided interface
  * Log more information re. FPGA boards (rev, SN...) in cisstLog.txt
  * Added simple example to collect data (used to test firmware velocities)
  * Qt widget:
    * when not in direct mode, hide more buttons/controls
    * added prompt message when switching to direct mode
    * support for inactive joints (not powered, e.g. MTM gripper)
    * more compact layout
* Bug fixes:
  * Update state after the coupling matrices are changed and before advancing
  * HasCoupling was not set to true when using write command SetCoupling
  * Removed unused methods to compute alternate velocities
  * Initialize pointer on ButtonsWidget in Qt factory

1.4.0 (2016-08-31)
==================

* API changes:
  * Added commands to set FireWire protocol (used in sawIntuitiveResearchKit console configuration JSON files)
* Deprecated features:
  * None
* New features:
  * CMake: separated components from applications/examples (catkin build 0.4 compatible)
* Bug fixes:
  * None

1.3.0 (2016-01-08)
==================

* API changes:
  * None
* Deprecated features:
  * None
* New features:
  * Added serial number display in Qt widget
  * Added command/event to change actuator coupling on the fly
  * Added command to bias encoders based on pots using multiple samples (averaging pot noise)
* Bug fixes:
  * Minor fixes to support older C++ compilers

1.2.0 (2015-10-18)
==================

* API changes:
  * Updated XML config file version to 2 for fix in digital inputs
* Deprecated features:
  * None
* New features:
  * Improved velocity estimation
  * Added commands to support PWM (used for SUJ lift motor and MUX)
* Bug fixes:
  * Encoder to pots safety check now looks for multiple consecutive failures to avoid intempestive errors
  * XML value "pressed" was used to negate values twice for events but once for state table
  * Fixed issue when power was lost on error that prevented powering back on

1.1.1 (2015-05-15)
==================

* API changes:
  * None
* Deprecated features:
  * None
* New features:
  * None
* Bug fixes:
  * In current calibration example, preload encoders to avoid encoder bit overflow

1.1.0 (2015-04-28)
==================

* API changes:
  * Change to SI units, i.e. translations are now assumed to be in meters, not millimeters.
  * Encoder/pots discrepencies are now treated as errors.
  * Requires mechatronics library Amp1394 version 1.1 or higher.
  * Removed encoder bits to position offset, now assumes the offset if preloaded on FPGA
* Deprecated features:
  * None
* New features:
  * Added `Error`, `Warning` and `Status` events with `std::string` payload.
  * Preload encoders when component starts.
  * All encoders are now relative to mid-range to avoid overflow.
  * Added detection of encoder bit overflow.
  * C++ exceptions are now caught and trigger cisstMultiTask events `Error`s.
* Bug fixes:
  * None


1.0.1 (2014-12-30)
==================

* No change log file, initial release.
