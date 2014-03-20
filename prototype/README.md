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
* `rclcpp` (ROS Client Library C++)
  * provides `<rclcpp/rclcpp.h>` as single entry point to nodes and pub/sub

How to build
------------

* `./checkout`

  Clones `std_msgs` and `common_msgs` into the workspace as well as a patched version of catkin.
  This command requires the `vcs` tool, `sudo apt-get install python-vcstool` or `sudo pip install vcstool`.

* `source /opt/ros/hydro/setup.bash`

  To get catkin and all message related packages.

* `source /SOMEWHERE/release.com`

  To get the OpenSplice environment.
  There is also the `my_setup.sh` in the `prototype` folder, which can be modified for your use and sourced instead.

* `catkin_make`

  Configures and builds the workspace.

Constraints
-----------

* All message names and field names use an underscore suffix to avoid collisions with keywords.

* All messages are nested in a subnamespace `dds_impl` to avoid collision with existing generated code.

* Constants can not be defined under the message class in DDS, therefore they are defined in the surounding scope with a prefix.

* Duration and Time types are currently simply mapped to `long long`.
