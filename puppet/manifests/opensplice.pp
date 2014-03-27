include apt

node default {
    apt::source { 'gazebo-latest':
        location => 'http://packages.osrfoundation.org/gazebo/ubuntu/',
        release => 'precise',
        repos => 'main',
        require => Apt::Key['gazebo-latest'];
    }

    apt::key { 'gazebo-latest':
      key_source => 'http://packages.osrfoundation.org/gazebo.key',
    }

    apt::key { 'ros-latest':
      key_source => 'http://packages.ros.org/ros.key',
    }

    apt::source { 'ros-latest':
        location => 'http://packages.ros.org/ros/ubuntu',
        release => 'raring',
        repos => 'main',
        require => Apt::Key['ros-latest'];
    }

    package {
        'libopensplice63':
             ensure => latest,
             require => Apt::Source['gazebo-latest'];
        'build-essential': ensure => latest;
        'cmake': ensure => latest;
        'vim': ensure => latest;
        'ros-hydro-ros-base':
             ensure => latest,
             require => Apt::Source['ros-latest'];
        'python-vcstool':
             ensure => latest,
             require => Apt::Source['ros-latest'];
    }

    file { ['/etc', '/etc/opensplice', '/etc/opensplice/config']:
        ensure => 'directory'
    }

    file { '/etc/opensplice/config/ospl.xml':
        content => template('opensplice/ospl.xml.erb'),
        require => File['/etc/opensplice/config'],
    }

    file { '/usr/etc/opensplice/config/ospl.xml':
        content => template('opensplice/ospl.xml.erb'),
        require => Package['libopensplice63']
    }

    exec { 'multicast_routing':
        command => '/sbin/ip route add 224.0.0.0/4 dev eth1',
        unless => '/sbin/ip route list|/bin/grep -c 224.0.0.0/4',
    }

    file { '/etc/rc.local':
        content => template('opensplice/rc.local.erb')
    }

#    augeas { 'multicast_routing_augeas':
#        context => "/files/etc/network/interfaces",
#        changes => [
#            "set iface[. = 'eth1']/up '/sbin/ip route add 224.0.0.0/4 dev eth1'",
#            "set iface[. = 'eth1']/down '/sbin/ip route del 224.0.0.0/4 dev eth1'",
#        ],
#    }
}
