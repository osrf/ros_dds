#include <stdio.h>
#include <boost/program_options.hpp>
#include <iostream>
#include <string>
#include <unistd.h>

#include "dds_robot.hh"

namespace po = boost::program_options;

//  ---------------------------------------------------------------------
/// \brief Function is called everytime a topic update is received.
void cb(const std::string &_topic, const std::string &_data)
{
  assert(_topic != "");
  std::cout << "\nCallback [" << _topic << "][" << _data << "]" << std::endl;
}

//  ---------------------------------------------------------------------
/// \brief Print program usage.
void PrintUsage(const po::options_description &_options)
{
  std::cout << "Usage: publisher [options] <topic> <data> <numMessages>\n"
            << "Positional arguments:\n"
            << "  <topic>               Topic to publish\n"
            << "  <data>                Data\n"
            << "  <numMessages>         Number of messages to send\n"
            << _options << "\n";
}

//  ---------------------------------------------------------------------
/// \brief Read the command line arguments.
int ReadArgs(int argc, char *argv[], bool &_verbose, bool &_selfSubscribe,
  std::string &_master, std::string &_topic, std::string &_data,
  int &_numMessages)
{
  // Optional arguments
  po::options_description visibleDesc("Options");
  visibleDesc.add_options()
    ("help,h", "Produce help message")
    ("verbose,v", "Enable verbose mode")
    ("self-subscribe,s", "Self-subscribe to the topic")
    ("master,m", po::value<std::string>(&_master)->default_value(""),
       "Set the master endpoint");

  // Positional arguments
  po::options_description hiddenDesc("Hidden options");
  hiddenDesc.add_options()
    ("topic", po::value<std::string>(&_topic), "Topic to publish")
    ("data", po::value<std::string>(&_data), "Data")
    ("num", po::value<int>(&_numMessages), "Number of messages to send");

  // All the arguments
  po::options_description desc("Options");
  desc.add(visibleDesc).add(hiddenDesc);

  // One value per positional argument
  po::positional_options_description positionalDesc;
  positionalDesc.add("topic", 1).add("data", 1).add("num", 1);

  po::variables_map vm;

  try
  {
    po::store(po::command_line_parser(argc, argv).
              options(desc).positional(positionalDesc).run(), vm);
    po::notify(vm);
  }
  catch(boost::exception &_e)
  {
    PrintUsage(visibleDesc);
    return -1;
  }

  if (vm.count("help")  || !vm.count("topic") ||
      !vm.count("data") || !vm.count("num"))
  {
    PrintUsage(visibleDesc);
    return -1;
  }

  _verbose = false;
  if (vm.count("verbose"))
    _verbose = true;

  if (vm.count("master"))
    _master = vm["master"].as<std::string>();

  _selfSubscribe = false;
  if (vm.count("self-subscribe"))
    _selfSubscribe = true;

  return 0;
}

//  ---------------------------------------------------------------------
int main(int argc, char *argv[])
{
  // Read the command line arguments
  std::string master, topic, data;
  int numMessages, rc;
  bool verbose, selfSubscribe;
  if (ReadArgs(argc, argv, verbose, selfSubscribe, master, topic, data,
               numMessages) != 0)
    return -1;

  // Transport node
  Node node(master, verbose);

  // Advertise a topic
  rc = node.Advertise(topic);
  if (rc != 0)
    std::cout << "Advertise did not work" << std::endl;

  if (selfSubscribe)
  {
    // Self-subscribe to the topic
    rc = node.Subscribe(topic, cb);
    if (rc != 0)
      std::cout << "Subscribe did not work" << std::endl;
  }

  // Publish data
  for (int i = 0; i < numMessages; ++i)
  {
    node.Publish(topic, data);
    usleep(10000);
  }

  // Zzzzzz Zzzzzz
  std::cout << "\nPress any key to exit" << std::endl;
  getchar();

  return 0;
}
