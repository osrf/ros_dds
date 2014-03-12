#!/bin/sh

# Directory in which librarian-puppet should manage its modules directory
PUPPET_DIR=/vagrant/puppet

# NB: librarian-puppet might need git installed. If it is not already installed
# in your basebox, this will manually install it at this point using apt or yum

$(which git > /dev/null 2>&1)
FOUND_GIT=$?
$(which apt-get > /dev/null 2>&1)
FOUND_APT=$?
$(which yum > /dev/null 2>&1)
FOUND_YUM=$?

if [ "${FOUND_APT}" -eq '0' ]; then
  apt-get -q -y update
  apt-get -q -y install ruby-json libaugeas-dev pkg-config
  echo 'ruby-json and augeas installed.'
fi

if [ "$FOUND_GIT" -ne '0' ]; then
  echo 'Attempting to install git.'

  if [ "${FOUND_YUM}" -eq '0' ]; then
    yum -q -y makecache
    yum -q -y install git
    echo 'git installed.'
  elif [ "${FOUND_APT}" -eq '0' ]; then
    apt-get -q -y install git
    echo 'git installed.'
  else
    echo 'No package installer available. You may need to install git manually.'
  fi
else
  echo 'git found.'
fi

if [ "$(gem search -i rubygems-update)" = "false" ]; then
  gem install rubygems-update --no-rdoc --no-ri
else
  gem update rubygems-update --no-rdoc --no-ri
fi

gem update --system --no-rdoc --no-ri

if [ "$(gem search -i ruby-augeas)" = "false" ]; then
  gem install ruby-augeas --no-rdoc --no-ri
else
  gem update ruby-augeas --no-rdoc --no-ri
fi

if [ "$(gem search -i puppet)" = "false" ]; then
  gem install puppet --no-rdoc --no-ri
else
  gem update puppet --no-rdoc --no-ri
fi

if [ "$(gem search -i librarian-puppet)" = "false" ]; then
  gem install librarian-puppet --no-rdoc --no-ri
  cd $PUPPET_DIR && librarian-puppet install --path modules-contrib
else
  cd $PUPPET_DIR && librarian-puppet update
fi
