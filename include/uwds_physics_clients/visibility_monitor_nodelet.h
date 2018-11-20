#ifndef VISIBILITY_MONITOR_HPP
#define VISIBILITY_MONITOR_HPP

#include <uwds/uwds.h>
#include <ros/ros.h>
#include <uwds/core/monitor.h>
#include <uwds/core/reconfigurable_client.h>
#include <tf/transform_listener.h>

#include "btBulletDynamicsCommon.h"
#include "b3RobotSimulatorClientAPI_NoGUI.h"

using namespace uwds_msgs;
using namespace uwds;

namespace uwds_physics_clients
{
  class VisibilityMonitor : public Filter
  {
  public:
    VisibilityMonitor(const WorldPtr input_world_,
                      const int width,
                      const int height);

    virtual Changes monitor(const Invalidations& invalidations);

    virtual void updateBulletNodes(const std::vector<std::string>& nodes_ids);

  private:

    int width_, height_;

    std::string ressource_folder_;

    b3RobotSimulatorClientAPI_NoGUI* sim_;

    float simulation_step_;

    float time_step_;

    std::map<std::string, int> node_id_map_;

    std::map<std::string, std::map<std::string, std::string>> visibilities_;

    WorldPtr input_world_;

    WorldPtr output_world_;

    std::string global_frame_id_;

    tf::TransformListener listener_;
  }

  class PhysicsFilterNodelet : public PhysicsFilter, public ReconfigurableClient
  {

  }
}
