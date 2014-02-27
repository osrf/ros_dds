# ros_dds

Prototype system using DDS as the middleware for a ROS like API.

## System requirements

You need to install OpenSplice.  For now, we're only concerned with the C++
bindings (both API and message generation); the other languages supported by
OpenSplice are excluded by these installation instructions.

### Ubuntu Linux

At the time of writing, we have packaged OpenSplice 6.3 for 64-bit Ubuntu
Precise, Quantal, and Raring.  To install on one of those systems, you need to
add the OSRF apt repo to your system then install `libopensplice63`:

~~~
# Add the appropriate apt repo (substitute `precise` for the right value):
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu precise main" >  /etc/apt/sources.list.d/gazebo-latest.list'
# Add the osrfoundation.org package repository key
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
# Update your list of packages
sudo apt-get update
# Install opensplice
sudo apt-get install libopensplice63
~~~

### Mac OSX

#### Homebrew

Assuming you have Homebrew installed already, you can do this to get opensplice:

```
brew tap osrf/ros2
brew install opensplice
```

The above should download and "pour" a binary bottle on OS X 10.8 and 10.9. You can, however, build it with debug symbols like this:

```
brew install --with-debug opensplice
```

### Generic build from source

Clone our fork of opensplice, then use our CMake wrapper around their oddball build system:

~~~
# Install build-time prerequisites, which on Linux are:
# (see http://www.prismtech.com/opensplice/opensplice-dds-community/building for the full list)
sudo apt-get install gawk flex bison perl
git clone https://github.com/osrf/opensplice.git
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/tmp/opensplice
make install
# Tell CMake to look there for CMake configuration files
export CMAKE_PREFIX_PATH=/tmp/opensplice:$CMAKE_PREFIX_PATH
~~~

## Talker / listener example

In the `cpp` subdirectory, there's a first version of a simplified DDS API, along with example publisher and subscriber applications.  It builds like vanilla CMake (assuming that you've already followed one of the installation paths described above):

~~~
cd cpp
mkdir build
cd build
cmake ..
make
~~~

To run the subscriber:

~~~
cd cpp/build
. setup.sh
./subscriber mytopic
~~~

To run the publisher:

~~~
cd cpp/build
. setup.sh
./publisher mytopic mystring 1000
~~~

You should expect to see output like this from the subscriber:

~~~
mytopic: mystring
mytopic: mystring
mytopic: mystring
mytopic: mystring
~~~

### Known limitations

* We're only dealing with string messages (see the definition in `cpp/idl/StringMsg.idl`).
* The publisher publishes a finite number of messages at a fixed frequency, then stops.
* The subscriber can only subscribe to one topic and it takes several seconds to shut down on Ctrl-C because OpenSplice's internal tear-down handlers are running.
* Before running an OpenSplice program, you need to source the generated `cpp/build/setup.sh` file (if you don't, you'll get an error like `Error in DDS::DomainParticipantFactory::create_participant: Creation failed: invalid handle`).  The `setup.sh` is simple, in that it just sets the `OSPL_URI` environment variable to point to the default XML configuration file.  But we need to think about to handle configuration in general.
