# -*- mode: ruby -*-
# vi: set ft=ruby :

# Vagrantfile API/syntax version. Don't touch unless you know what you're doing!
VAGRANTFILE_API_VERSION = "2"

Vagrant.configure(VAGRANTFILE_API_VERSION) do |config|
  config.vm.box = "precise64"
  config.vm.box_url = "http://files.vagrantup.com/precise64.box"
  config.vbguest.auto_update = true

  config.vm.provision :shell, :path => "shell/main.sh"

  config.vm.provision :puppet do |puppet|
    puppet.manifests_path = "puppet/manifests"
    puppet.manifest_file  = "site.pp"
  end

  config.vm.define "host1" do |host1|
    host1.vm.box = "precise64"
    host1.vm.network "private_network", ip: "10.0.3.20",
      virtualbox__intnet: "dds-network"
  end

  config.vm.define "host2" do |host2|
    host2.vm.box = "precise64"
    host2.vm.network "private_network", ip: "10.0.3.21",
      virtualbox__intnet: "dds-network"
  end

  config.vm.define "host3" do |host3|
    host3.vm.box = "precise64"
    host3.vm.network "private_network", ip: "10.0.3.22",
      virtualbox__intnet: "dds-network"
  end
end
