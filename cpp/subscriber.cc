#include <boost/program_options.hpp>
#include <iostream>
#include <string>
#include <vector>

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
  std::cout << "Usage: subscriber [options] <topic1> ... <topicN>\n"
            << "Positional arguments:\n"
            << "  <topic1>              Topic to subscribe\n"
            << "  ...\n"
            << "  <topicN>              Topic to subscribe\n"
            << _options << "\n";
}

//  ---------------------------------------------------------------------
/// \brief Read the command line arguments.
int ReadArgs(int argc, char *argv[], bool &_verbose, std::string &_master,
             std::vector<std::string> &_topics)
{
  // Optional arguments
  po::options_description visibleDesc("Options");
  visibleDesc.add_options()
    ("help,h", "Produce help message")
    ("verbose,v", "Enable verbose mode")
    ("master,m", po::value<std::string>(&_master)->default_value(""),
       "Set the master endpoint");

  // Positional arguments
  po::options_description hiddenDesc("Hidden options");
  hiddenDesc.add_options()
    ("topics", po::value<std::vector<std::string> >(&_topics),
      "Topics to subscribe");

  // All the arguments
  po::options_description desc("Options");
  desc.add(visibleDesc).add(hiddenDesc);

  // One value per positional argument
  po::positional_options_description positionalDesc;
  positionalDesc.add("topics", -1);

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

  if (vm.count("help")  || !vm.count("topics"))
  {
    PrintUsage(visibleDesc);
    return -1;
  }

  _verbose = false;
  if (vm.count("verbose"))
    _verbose = true;

  if (vm.count("master"))
    _master = vm["master"].as<std::string>();

  return 0;
}

//  ---------------------------------------------------------------------
int main(int argc, char *argv[])
{
  // Read the command line arguments
  std::string master;
  bool verbose;
  std::vector<std::string> topics;
  if (ReadArgs(argc, argv, verbose, master, topics) != 0)
    return -1;

  // Transport node
  Node node(master, verbose);

  // Subscribe to the list of topics
  for (int i = 0; i < topics.size(); ++i)
  {
    int rc = node.Subscribe(topics[i], cb);
    if (rc != 0)
      std::cout << "subscribe for topic [" << topics[i] << "] did not work\n";

    /*
    rc = node.UnSubscribe(topics[i]);
    if (rc != 0)
      std::cout << "unsubscribe for topic [" << topics[i] << "] did not work\n";
    */
  }

  // Zzzzzz Zzzzzz
  std::cout << "\nPress any key to exit" << std::endl;
  getchar();

  // node.Spin();

  return 0;
}
