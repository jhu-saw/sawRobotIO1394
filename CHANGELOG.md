Change log
==========

2.2.0 (2023-11-15)
==================

* API changes:
  * XML file format changed to 5
    * We strongly recommend to use the new configuration generator (`dvrk-config-generator.py`)
    * Requires `HardwareVersion` (`QLA1`, `DQLA`, `dRA1`)
    * Encoder `VelocitySource` (`SOFTWARE` or `FIRMWARE`)
    * Added `PositionLimitsSoft`
    * Removed coupling for all arms, added `JointToActuatorPosition` for MTM potentiometers
    * Added potentiometers `LookupTable` for Si PSMs and ECM
    * Removed effort limits since these can be computed from current limits
  * Using CRTK naming convention when possible
  * Use standard types as much as possible, i.e. from `stdint` instead of typedefs from AmpIO
  * `GripperCalibration` program now replaces the existing XML file and creates a backup of old one
* Deprecated features:
  * None
* New features:
  * Support for Si PSMS and ECM, dRA1 based controllers
  * Support for DQLA controllers for Classic arms
  * Provides both software and firmware based velocity estimations
  * CMake:
    * Updated install targets and debian packages generation
    * Works with ROS2/colcon
  * ROS1: standalone ROS node for IO only
* Bug fixes:
  * Reduced default debounce time for some inputs
  * Fixed event on first run
  * Removed hard clock frequencies, now uses AmpIO method

2.1.0 (2021-08-11)
==================

* API changes:
  * Using prmStateJoint for potentiometer values
  * Instrument name is now following the convention `name:model[version]`.  It used to be `name_model`
  * Removed enum used to define the FireWire protocol, now uses a string passed to AmpIO
* Deprecated features:
  * None
* New features:
  * Added option to plot potentiometers and encoders in IO widget, useful to debug potentiometer issues
  * Monitoring power fault bit from AmpIO
  * Instrument name now includes version
  * Added option to set FireWire protocol for `sawRobotIO1394QtConsole`
* Bug fixes:
  * Fixed bug in current calibration utility (thx to Robert at Twente)
  * Fixed CMake dependency for Qt library using header files generated along the core library (`.cdg`)

2.0.0 (2021-04-08)
==================

* API changes:
  * Using CRTK naming convention
  * XML configuration format change to version 4
* Deprecated features:
  * Removed support for firmware 4 and 5
  * Removed software based velocity estimation, now rely on firmware
  * Removed code for compiling without *cisst*
* New features:
  * Support for latest AmpIO API + firmware 7
  * Added `DallasChip` class to query tool type on dVRK PSMs
  * Support for ethernet/UDP port
  * Monitor period statistics and send warnings/errors if compute time is too long
  * Monitor amplifiers temperature and send warnings/errors if too hot
  * Powering off doesn't automatically open safety relays (optional parameter)
  * Better velocity estimation with firmware 7
  * Check units for scale/offset in configuration file for `xxToPosSI` parameters
  * Added user defined optional threshold for quick tap (click) in configuration files
  * Use *cisstDataGenerator* for configuration, plan to move to replace XML with JSON (or YAML)
  * Minor updates to compile on Ubuntu 20.04, Windows and MacOS
  * Widget:
    * Better names for powering and enable amps (matches component interface)
    * Expose state of safety relays (and control in *Direct mode*)
  * Application console Qt can be used with multiple XML configurations files
* Bug fixes:
  * Better powering sequence, added delay when requesting power to check if power is ok

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

* API changes:https://dvrk.lcsr.jhu.edu
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
