#include "uwds_physics_clients/bullet3_client.h"

using namespace uwds;

namespace uwds_physics_clients
{
  Bullet3Client::Bullet3Client(b3RobotSimulatorClientAPI_NoGUI* simulator,
                               const std::string& ressource_folder,
                               float time_step,
                               std::string global_frame_id)
  {
    time_step_ = time_step;
    global_frame_id_ = global_frame_id;
    sim_ = simulator;
    sim_->connect(eCONNECT_DIRECT);
    sim_->setTimeStep(time_step);
    sim_->setGravity(btVector3(0, 0, -9.8));
    node_id_map_.at(input_world->scene().rootID()) = sim_->LoadURDF("plane.urdf");
  }


  void Bullet3Client::updateBulletNodes(std::vector<std::string> nodes_ids)
  {
    for (const auto& id : nodes_ids)
    {
      if(id != input_world->scene().rootID())
      {
        if(node_id_map_.count(id)==0)
        {
          std::string node_name = input_world->scene().nodes()[id].name;
          try
          {
            tf::StampedTransform transform;
            listener_.lookupTransform(global_frame_id_, node_name, ros::Time(0), transform);
            btVector3 position = btVector3(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
            tf::Quaternion q = transform.getRotation();
            tfScalar yaw, pitch, roll;
            tf::Matrix3x3 mat = mat(q);
            mat.getEulerYPR(&yaw, &pitch, &roll);
            btVector3 orientation = btVector3(yaw, pitch, roll);
            int bullet_id = sim_->loadURDF(ressource_folder_+"/urdf/"+node_name+".urdf", position, orientation);
            node_id_map_.at(id) = bullet_id;
            reverse_node_id_map_.at(bullet_id) = id;
          } catch (tf::TransformException ex){
            ROS_ERROR("[physics_filter] Error occured : %s",ex.what());
          }
        } else {
          if(node_id_map_.at(id) > 0) // If an URDF have been loaded
          {
            try // Try to update the bullet node
            {
              tf::StampedTransform transform;
              listener_.lookupTransform(global_frame_id_, node_name, ros::Time(0), transform);
              btVector3 position = btVector3(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
              tf::Quaternion q = transform.getRotation();
              tfScalar yaw, pitch, roll;
              tf::Matrix3x3 mat = mat(q);
              mat.getEulerYPR(&yaw, &pitch, &roll);
              btVector3 orientation = btVector3(yaw, pitch, roll);
              sim_->resetBaseVelocity(node_id_map_.at(id), btVector3(), btVector3());
              sim_->resetBasePositionAndOrientation(node_id_map_.at(id), position, orientation);
            } catch (tf::TransformException ex){
              ROS_ERROR("[physics_filter] Error occured : %s",ex.what());
            }
          }
        }
      }
    }
  }
}
