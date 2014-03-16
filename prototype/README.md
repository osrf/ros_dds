Prototype
=========

... reusing existing ROS packages and tools in order to get something running sooner.

Current functionality
---------------------

* `genidl`
  * generates IDL files for msg files (no services yet)
  * the result is placed under share/PKGNAME/dds_idl/MSGNAME.msg
* `genidlcpp`
  * generates DDS C++ code from idl files
  * the result is placed under include/PKGNAME/dds_impl/*

How to build
------------

* `./checkout`

  Clones `std_msgs` and `common_msgs` into the workspace.

* `source /opt/ros/hydro/setup.bash`

  To get catkin and all message related packages.

* `source /SOMEWHERE/release.com`

  To get the OpenSplice environment.

* `catkin_make`

  Configures and builds the workspace.

Constraints
-----------

* All message names and field names use an underscore suffix to avoid collisions with keywords.

* All messages are nested in a subnamespace `dds_impl` to avoid collision with existing generated code.

* Constants can not be defined under the message class in DDS, therefore they are defined in the surounding scope with a prefix.

* Duration and Time types are currently simply mapped to `long long`.
