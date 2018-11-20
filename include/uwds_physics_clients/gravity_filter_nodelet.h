#ifndef GRAVITY_FILTER_HPP
#define GRAVITY_FILTER_HPP

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
  class GravityFilter : public Filter
  {
  public:
    GravityFilter(const std::string& ressource_folder,
                  float time_step,
                  float simulation_step,
                  std::string global_frame_id,
                  const WorldPtr input_world_,
                  const WorldPtr output_world_);

    virtual void filter(const Changes& input_changes, const Changes& output_changes);

    virtual void stepSimulation(const float duration);

    virtual void updateBulletNodes(const std::vector<std::string>& nodes_ids);

  private:

    std::string ressource_folder_;

    b3RobotSimulatorClientAPI_NoGUI* sim_;

    float simulation_step_;

    float time_step_;

    std::map<std::string, int> node_id_map_;

    WorldPtr input_world_;

    WorldPtr output_world_;

    std::string global_frame_id_;

    tf::TransformListener listener_;
  }

  class GravityFilterNodelet : public ReconfigurableClient
  {
    public:
      virtual void onInit();

      virtual void onChanges(const std::string &world, const std_msgs::Header &header, const Invalidations &invalidations);

      virtual void onSubscribeChanges(const std::string world);

      virtual void onUnSubscribeChanges(const std::string world);

    private:
      GravityFilter filter_;

  }
}
