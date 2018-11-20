#include "uwds_tutorials/ar_objects_provider.h"

using namespace uwds;

namespace uwds_physics_clients
{
  VisibilityMonitor::VisibilityMonitor(const WorldPtr input_world_, const int width, const int height)
  {
    input_world_ = input_world;
    width_ = width;
    height_ = height;
  }

  Changes VisibilityMonitor::monitor(const Invalidations& invalidations)
  {
    updateBulletNodes(input_changes.node_to_update);
    for (const auto& node : input_changes.node_to_update)
    {

    }
  }

  map<std::string, float>& VisibilityMonitor::computeVisibilities(const std::string camera_id)
  {
    if(input_world_->has(camera_id))
    {
      node = input_world_->[camera_id];
      if(node.type == CAMERA)
      {
        position = btVector3(node.position.pose.position.x, node.position.pose.position.y, node.position.pose.position.z);
        tf::Quaternion q = Quaternion(node.position.pose.orientation.x,
                                      node.position.pose.orientation.y,
                                      node.position.pose.orientation.z,
                                      node.position.pose.orientation.w);
        tfScalar yaw, pitch, roll;
        tf::Matrix3x3 mat = mat(q);
        mat.getEulerYPR(&yaw, &pitch, &roll);
        float focus_distance = getFocus(camera_id);
        auto view_matrix = sim_->computeViewMatrixFromYawPitchRoll(position, focus_distance, yaw, pitch, roll, 2);

        float hfov = getHfov(camera_id);
        float aspect = getAspect(camera_id);
        float near = getClipNear(camera_id);
        float far = getClipFar(camera_id);
        auto projection_matrix = sim_->computeProjectionMatrixFOV(hfov, aspect, near, far);

        auto segmentation_mask = sim_->getCameraImage(width_, height_, view_matrix, projection_matrix, sim_->ER_TINY_RENDERER, sim_->ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)[4];

        map<std::string, float> visibilities;
        for (unsigned int line = 0; line < height_; line++)
          for (unsigned int col = 0; col < width_; col++)
            if(segmentation_mask[line*col+1]>0)
            {
              object_id = segmentation_mask[line*col+1] & ((1<<24)-1)
              if (visibilities.count(reverse_node_id_map_[object_id]) > 0)
                visibilities[reverse_node_id_map_[object_id]]++;
              else:
                visibilities[reverse_node_id_map_[object_id]] = 1;
            }
        for (const auto visibility_pair : visibilities)
        {
          visibilities.second /= width_ * height_;
        }
      }
    }
  }

  std::vector<Situation> computeSituations(std::string camera_id, map<std::string, float> visibily_scores)
  {
    prev_situations = input_world_->timeline().situations().byProperty("predicate", "isVisible");
    for(const auto& situation : prev_situations)
    {
      if (situation.)
      prev_situations
    }
  }

  float getHfov(const std::string camera_id)
  {
    if(input_world_->has(camera_id))
      node = input_world_->[camera_id];
      if(world()[camera_id].type == CAMERA)
      {
        for (const auto property : node.properties)
        {
          if (property.name == "hfov")
            return atof(property.data);
        }
      }
    return 0.0;
  }

  float getAspect()
  {
    if(input_world_->has(camera_id))
      node = input_world_->[camera_id];
      for (const auto property : node.properties)
      {
        if (property.name == "aspect")
          return atof(property.data);
      }
    return 0.0;
  }

  float getClipNear()
  {
    if(input_world_->has(camera_id))
      node = input_world_->[camera_id];
      for (const auto property : node.properties)
      {
        if (property.name == "clipnear")
          return atof(property.data);
      }
    return 0.0;
  }

  float getClipFar()
  {
    if(input_world_->has(camera_id))
      node = input_world_->[camera_id];
      for (const auto property : node.properties)
      {
        if (property.name == "clipnear")
          return atof(property.data);
      }
    return 0.0;
  }

  float getFocusDistance()
  {
    if(input_world_->has(camera_id))
      node = input_world_->[camera_id];
      for (const auto property : node.properties)
      {
        if (property.name == "focus_distance")
          return atof(property.data);
      }
    return 4.0;
  }

  void VisibilityMonitor::publishImage()
  {

  }

}
