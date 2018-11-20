#include "uwds_tutorials/ar_objects_provider.h"

using namespace uwds;

namespace uwds_physics_clients
{
  GravityFilter::GravityFilter(const std::string& ressource_folder,
                               float time_step,
                               float simulation_step,
                               std::string global_frame_id,
                               const WorldPtr input_world_,
                               const WorldPtr output_world_)
  {
    simulation_step_ = simulation_step;
    time_step_ = time_step;
    input_world_ = input_world;
    output_world_ = output_world;
    sim_ = new b3RobotSimulatorClientAPI_NoGUI();
    sim_->connect(eCONNECT_DIRECT);
    sim_->setTimeStep(time_step);
    sim_->setGravity(btVector3(0, 0, -9.8));
    node_id_map_.at(input_world->scene().rootID()) = sim_->LoadURDF("plane.urdf");
  }

  void GravityFilter::filter(const Changes& input_changes, const Changes& output_changes)
  {
    updateBulletNodes(input_changes.node_to_update);
    stepSimulation(simulation_step_);
    for (const auto& node : input_changes.node_to_update)
    {

    }
  }

  void GravityFilter::stepSimulation()
  {
    for(unsigned int i = 0 ; i < simulation_step/time_step_, i++)
     sim_->stepSimulation();
  }

}
