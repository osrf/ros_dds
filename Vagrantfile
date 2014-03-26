# -*- mode: ruby -*-
# vi: set ft=ruby :

# Vagrantfile API/syntax version. Don't touch unless you know what you're doing!
VAGRANTFILE_API_VERSION = "2"

Vagrant.configure(VAGRANTFILE_API_VERSION) do |config|
  config.vbguest.auto_update = true

  config.vm.provider "virtualbox" do |vb|
    vb.customize ["modifyvm", :id, "--natdnsproxy1", "off"]
    vb.customize ["modifyvm", :id, "--natdnshostresolver1", "off"]
    vb.customize ["modifyvm", :id, "--memory", "1024"]
  end

  config.vm.provision :shell, :path => "shell/main.sh"

  config.vm.provision :puppet do |puppet|
    puppet.manifests_path = "puppet/manifests"
    puppet.manifest_file  = "site.pp"
    puppet.module_path = ['puppet/modules-contrib', 'puppet/modules' ]
  end

  config.vm.define "host1" do |host|
    host.vm.box = "precise64"
    host.vm.box_url = "http://files.vagrantup.com/precise64.box"
    host.vm.network "private_network", ip: "10.0.3.20",
      virtualbox__intnet: "dds-network"
  end

  config.vm.define "host2" do |host|
    host.vm.box = "precise64"
    host.vm.box_url = "http://files.vagrantup.com/precise64.box"
    host.vm.network "private_network", ip: "10.0.3.21",
      virtualbox__intnet: "dds-network"
  end

  config.vm.define "host3" do |host|
    host.vm.box = "precise64"
    host.vm.box_url = "http://files.vagrantup.com/precise64.box"
    host.vm.network "private_network", ip: "10.0.3.22",
      virtualbox__intnet: "dds-network"
  end

  config.vm.define "host1r" do |host|
    host.vm.box = "raring64"
    host.vm.box_url = "http://cloud-images.ubuntu.com/vagrant/raring/current/raring-server-cloudimg-amd64-vagrant-disk1.box"
    host.vm.network "private_network", ip: "10.0.4.20",
      virtualbox__intnet: "dds-network"
  end

  config.vm.define "host2r" do |host|
    host.vm.box = "raring64"
    host.vm.box_url = "http://cloud-images.ubuntu.com/vagrant/raring/current/raring-server-cloudimg-amd64-vagrant-disk1.box"
    host.vm.network "private_network", ip: "10.0.4.21",
      virtualbox__intnet: "dds-network"
  end

  config.vm.define "host3r" do |host|
    host.vm.box = "raring64"
    host.vm.box_url = "http://cloud-images.ubuntu.com/vagrant/raring/current/raring-server-cloudimg-amd64-vagrant-disk1.box"
    host.vm.network "private_network", ip: "10.0.4.22",
      virtualbox__intnet: "dds-network"
  end

  config.vm.define "host1s" do |host|
    host.vm.box = "saucy64"
    host.vm.box_url = "http://cloud-images.ubuntu.com/vagrant/saucy/current/saucy-server-cloudimg-amd64-vagrant-disk1.box"
    host.vm.network "private_network", ip: "10.0.5.20",
      virtualbox__intnet: "dds-network"
  end

  config.vm.define "host1t" do |host|
    host.vm.box = "trusty64"
    host.vm.box_url = "http://cloud-images.ubuntu.com/vagrant/trusty/current/trusty-server-cloudimg-amd64-vagrant-disk1.box"
    host.vm.network "private_network", ip: "10.0.6.20",
      virtualbox__intnet: "dds-network"
  end

  config.vm.define "host2t" do |host|
    host.vm.box = "trusty64"
    host.vm.box_url = "http://cloud-images.ubuntu.com/vagrant/trusty/current/trusty-server-cloudimg-amd64-vagrant-disk1.box"
    host.vm.network "private_network", ip: "10.0.6.21",
      virtualbox__intnet: "dds-network"
  end

  config.vm.define "host3t" do |host|
    host.vm.box = "trusty64"
    host.vm.box_url = "http://cloud-images.ubuntu.com/vagrant/trusty/current/trusty-server-cloudimg-amd64-vagrant-disk1.box"
    host.vm.network "private_network", ip: "10.0.6.22",
      virtualbox__intnet: "dds-network"
  end

end
