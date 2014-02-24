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

    package {
        'libopensplice63':
             ensure => latest,
             require => Apt::Source['gazebo-latest'];
        'build-essential': ensure => latest;
        'cmake': ensure => latest;
    }
}
