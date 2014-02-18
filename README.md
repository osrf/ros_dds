ros_dds
=======

Prototype system using DDS as the middleware for a ROS like API.

System requirements
-------------------

You need to install OpenSplice.  For now, we're only concerned with the C++
bindings (both API and message generation); the other languages supported by
OpenSplice are excluded by these installation instructions.

Ubuntu Linux
^^^^^^^^^^^^

At the time of writing, we have packaged OpenSplice 6.3 for 64-bit Ubuntu
Precise, Quantal, and Raring.  To install on one of those systems, you need to
add the OSRF apt repo to your system then install `libopensplice63`:

~~~
# Add the appropriate apt repo (substitute `precise` for the right value):
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu precise main" > /etc/apt/sources.list.d/gazebo-latest.list'
# Update your list of packages
sudo apt-get update
# Install opensplice
sudo apt-get install libopensplice63
~~~

Mac OSX
^^^^^^^

TODO

Generic build from source
^^^^^^^^^^^^^^^^^^^^^^^^^

Clone our fork of opensplice, then use our hacky wrapper for their oddball build
system to produce a tarball that you can drop somewhere and build against:

~~~
git clone https://github.com/osrf/opensplice.git
cd opensplice
# When prompted by the following script, pick the build type that looks good to you
./minimal_build.sh
~~~

The script will tell you about a tarball that contains a minimal build of
OpenSplice (just C++ support).  Say that the tarball is called
`$HOME/code/opensplice/install/minimal/opensplice-minimal.tgz`; then to install and use it:

~~~
# Make a directory to contain the unpacked tarball (it doesn't have a single
# top-level directory).
mkdir /tmp/opensplice
cd /tmp/opensplice
tar xf $HOME/code/opensplice/install/minimal/opensplice-minimal.tgz
# Tell CMake to look there for CMake configuration files
export CMAKE_PREFIX_PATH=/tmp/opensplice:$CMAKE_PREFIX_PATH
~~~

Talker / listener example
-------------------------
