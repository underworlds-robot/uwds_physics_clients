#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import pybullet as p
import pybullet_data
from pyuwds.reconfigurable_client import ReconfigurableClient
from uwds_msgs.msg import Changes
from uwds_msgs.srv import *
from pyuwds.types import FILTER, MESH
import tf_conversions
import geometry_msgs
import tf


class GravityFilter(ReconfigurableClient):
    def __init__(self):
        """
        """
        self.ressource_folder = rospy.get_param("~ressource_folder")
        self.time_step = rospy.get_param("~time_step", 0.01)
        self.simulation_step = rospy.get_param("~simulation_step", 1.0)
        ReconfigurableClient.__init__(self, "gravity_filter", FILTER)

    def onReconfigure(self, worlds_names):
        """
        """
        pass

    def onSubscribeChanges(self, world_name):
        """
        """
        p.connect(p.DIRECT) # Initialize bullet non-graphical version
        p.setGravity(0, 0, -10)
        p.setAdditionalSearchPath(self.ressource_folder)
        p.setTimeStep(self.time_step)
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
        changes = self.filter(world_name, header, invalidations)
        if len(changes.nodes_to_update) > 0:
            self.sendWorldChanges(world_name+"_stable", header, changes)
            rospy.loginfo("[%s::onChanges] Changes send in world <%s>", self.node_name, world_name+"_stable")

    def filter(self, world_name, header, invalidations):
        """
        """
        changes = Changes()
        for node_id in invalidations.node_ids_updated:
            self.updateBulletNodes(world_name, node_id)
        self.stepSimulation()
        for node_id, node in self.worlds[world_name].scene.nodes.items():
            if node_id in invalidations.node_ids_updated:
                if node_id in self.urdf_available:
                    if self.urdf_available[node_id]:
                        t, q = p.getBasePositionAndOrientation(self.node_id_map[node_id])
                        x, y, z = t
                        node.position.pose.position.x = x
                        node.position.pose.position.y = y
                        node.position.pose.position.z = z
                        x, y, z, w = q
                        node.position.pose.orientation.x = x
                        node.position.pose.orientation.y = y
                        node.position.pose.orientation.z = z
                        node.position.pose.orientation.w = w
                if node_id in invalidations.node_ids_updated:
                    changes.nodes_to_update.append(node)

        return changes

    def stepSimulation(self):
        """
        """
        step = int(self.simulation_step / self.time_step)
        for i in range(0, step):
            p.stepSimulation()

    def updateBulletNodes(self, world_name, node_id):
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
    rospy.init_node("gravity_filter")
    gf = GravityFilter()
    rospy.spin()
