#package { 'git':
#        ensure => installed,
#}
    

#file { '/home/vagrant/ros_dds':
##    ensure  => directory,
#    group   => 'vagrant',
#    owner   => 'vagrant',
#    mode    => 0700,
#}

vcsrepo { "/home/vagrant/ros_dds":
    ensure => present,
    provider => git,
    source => 'https://github.com/osrf/ros_dds.git',
    revision => 'master'
}
