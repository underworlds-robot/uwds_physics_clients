#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import pybullet as p
import pybullet_data
from pyuwds.reconfigurable_client import ReconfigurableClient
from uwds_msgs.msg import Changes
from uwds_msgs.srv import *
from pyuwds.types import MONITOR, MESH, CAMERA
import tf

class VisibilityMonitor(ReconfigurableClient):
    def __init__(self):
        """
        """
        self.ressource_folder = rospy.get_param("~ressource_folder")
        ReconfigurableClient.__init__(self, "visibility_monitor", MONITOR)

    def onReconfigure(self, worlds):
        """
        """
        pass

    def onSubscribeChanges(self, world_name):
        """
        """
        p.connect(p.GUI) # Initialize bullet non-graphical version
        p.setGravity(0, 0, -10)
        p.setAdditionalSearchPath(self.ressource_folder)
        self.urdf_available = {}
        self.node_id_map = {}
        self.reverse_node_id_map = {}
        planeId = p.loadURDF("plane.urdf")
        self.node_id_map[self.worlds[world_name].scene.rootID] = planeId
        self.reverse_node_id_map[planeId] = self.worlds[world_name].scene.rootID

    def onUnsubscribeChanges(self, world_name):
        """
        """
        pass

    def onChanges(self, world_name, header, invalidations):
        """
        """
        rospy.loginfo("[%s::onChanges] Changes received for world <%s>", self.node_name, world_name)
        changes = self.monitor(world_name, header, invalidations)
        if len(changes.situations_to_update) > 0:
            self.sendWorldChanges("visibilities", header, changes)
            rospy.loginfo("[%s::onChanges] Changes send in world <%s>", self.node_name, world_name+"_stable")

    def monitor(self, world_name, header, invalidations):
        """
        """
        changes = Changes()
        for node_id in invalidations.node_ids_updated:
            self.updateBulletNode(world_name, node_id)

        for node_id, node in self.worlds[world_name].scene.nodes.items():
            if node_id in invalidations.node_ids_updated:
                if node.type == CAMERA:
                    visibilities = computeVisibilities(world_name, camera_id)
                    print visibilities
                    situations = computeSituations(world_name, header.stamp, node_id, visibilities)
                    for situation in situations:
                        changes.situations_to_update.append(situation)
                changes.nodes_to_update.append(node)
        return changes

    def computeVisibilities(self, world_name, camera_id):
        visibilities = {}
        if not p.isNumpyEnabled():
            rospy.logwarn("[%s::computeVisibilities] Numpy not enabled, copy of image buffer can be slow", self.node_name)
        if camera_id in self.worlds[world_name].scene.nodes:
            camera_node = self.worlds[world_name].scene.nodes[camera_id]
            position = [camera_node.position.pose.position.x, camera_node.position.pose.position.y, camera_node.position.pose.position.z]
            euler = tf.transformations.euler_from_quaternion(camera_node.position.pose.orientation)
            view_matrix = p.computeViewMatrixFromYawPitchRoll(position, 4, euler[0], euler[1], euler[2], 2)
            fov = float(self.worlds[world_name].scene.getNodeProperty(camera_id, "hfov"))
            clipnear = float(self.worlds[world_name].scene.getNodeProperty(camera_id, "clipnear"))
            clipfar = float(self.worlds[world_name].scene.getNodeProperty(camera_id, "clipfar"))
            aspect = float(self.worlds[world_name].scene.getNodeProperty(camera_id, "aspect"))
            proj_matrix = p.computeProjectionMatrixFOV(fov, aspect, clipnear, clipfar)
            img = p.getCameraImage(self.width, self.height, flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
            seg = img[4]
            #cv_image = pyopencv.Mat(height, width)
            for pixel in seg:
                if (pixel >= 0):
                    bullet_id = pixel & ((1 << 24)-1)
                    uwds_id = self.reverse_node_id_map[bullet_id]
                    #cv_image[width * height+1] = pixel
                    if uwds_id not in visibilities:
                        visibilities[uwds_id] = 1
                    else:
                        visibilities[uwds_id] += 1
                for visibility_score in visibilities.items():
                    visibility_score / self.width * self.height
        return visibilities#, cv_image

    def computeSituations(self, world_name, stamp, camera_id, visibilities):
        situations = []
        for id_seen, visibility_score in visibilities:
            if camera_id+id_seen not in self.current_situation_map and visibility_score > self.min_treshold:
                situation = Situation()
                situation.id = str(uuid.uuid4())
                situation.type = FACT
                situation.description = self.worlds[world_name].scene.nodes[id_seen].name + "is visible by" + self.worlds[world_name].scene.nodes[camera_id].name
                predicate = Property()
                predicate.name = "predicate"
                predicate.data = "isVisible"
                situation.properties.append(predicate)
                subject = Property()
                subject.name = "subject"
                subject.data = camera_id
                situation.properties.append(subject)
                object = Property()
                object.name = "object"
                object.data = id_seen
                situation.properties.append(object)
                situation.confidence = visibility_score
                situation.start.data = stamp
                self.current_situation_map[camera_id+entity_seen] = situation
                situations.append(situation)
            else:
                situation = self.current_situation_map[camera_id+entity_seen]
                situation.confidence = visibility_score
                if situation.confidence < self.min_treshold:
                    situation.end = stamp
                    del self.current_situation_map[camera_id+entity_seen]
                situations.append(situation)
        return situations

    def updateBulletNode(self, world_name, node_id):
        """ This function load the urdf corresponding to the uwds node and set it in the environment
        The urdf need to have the same name than the node name
        :return:
        """
        if node_id != self.worlds[world_name].scene.rootID:
            node = self.worlds[world_name].scene.nodes[node_id]
            if node.type == MESH:
                rospy.loginfo("[%s::updateBulletNodes] Update node <%s(%s)> to bullet", self.node_name, node.name, node_id)
                position = [node.position.pose.position.x, node.position.pose.position.y, node.position.pose.position.z]
                orientation = [node.position.pose.orientation.x, node.position.pose.orientation.y, node.position.pose.orientation.z, node.position.pose.orientation.w]
                if node_id not in self.node_id_map:
                    try:
                        bullet_id = p.loadURDF(node.name+".urdf", position, orientation)
                        self.urdf_available[node_id] = True
                    except Exception as e:
                        self.urdf_available[node_id] = False
                        bullet_id = -1
                    if self.urdf_available[node_id]:
                        rospy.loginfo("[%s::updateBulletNodes] "+node.name+".urdf' loaded successfully", self.node_name)
                        self.reverse_node_id_map[bullet_id] = node_id
                    self.node_id_map[node_id] = bullet_id
                else:
                    if self.urdf_available[node_id] > 0:
                        p.resetBaseVelocity(self.node_id_map[node_id], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
                        p.resetBasePositionAndOrientation(self.node_id_map[node_id], position, orientation)
            else:
                self.urdf_available[node_id] = False


if __name__ == '__main__':
    rospy.init_node("visibility_monitor")
    vm = VisibilityMonitor()
    rospy.spin()
