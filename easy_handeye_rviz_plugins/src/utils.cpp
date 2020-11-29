#include "utils.h"

#include <ros/master.h>

bool hasEnding(std::string const& full_string, std::string const& ending)
{
  if (full_string.length() >= ending.length())
  {
    return (0 == full_string.compare(full_string.length() - ending.length(), ending.length(), ending));
  }
  else
  {
    return false;
  }
}

std::vector<std::string> listServices()
{
  XmlRpc::XmlRpcValue req = "/node";
  XmlRpc::XmlRpcValue res;
  XmlRpc::XmlRpcValue pay;

  std::vector<std::string> state;
  ros::master::execute("getSystemState", req, res, pay, true);

  for (int x = 0; x < res[2][2].size(); x++)
  {
    std::string gh = res[2][2][x][0].toXml();
    // remove <value></value>
    state.push_back(gh.substr(7, gh.size() - 15));
  }

  return state;
}