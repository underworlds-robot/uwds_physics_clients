#ifndef BULLET3_CLIENT_HPP
#define BULLET3_CLIENT_HPP

#include <uwds/uwds.h>
#include <ros/ros.h>

#include <bullet/b3RobotSimulatorClientAPI_NoGUI.h>

namespace uwds_physics_clients
{
  class Bullet3Client
  {
  public:
    Bullet3Client(const std::string& ressource_folder,
                  float time_step,
                  std::string global_frame_id);

    virtual void updateBulletNodes(const std::vector<std::string>& nodes_ids);

  private:

    std::string ressource_folder_;

    b3RobotSimulatorClientAPI_NoGUI* sim_;

    float time_step_;

    std::map<std::string, int> node_id_map_;

    std::map<int, std::string> reverse_node_id_map_;

    std::string global_frame_id_;
  }
}
