#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import pybullet as p
import pybullet_data
from pyuwds.reconfigurable_client import ReconfigurableClient
from uwds_msgs.msg import Changes
from uwds_msgs.srv import *
from pyuwds.types import FILTER


class GravityFilter(ReconfigurableClient):
    def __init__(self):
        """
        """
        self.ressource_folder = rospy.get_param("~ressource_folder")
        self.time_step = rospy.get_param("~time_step")
        self.simulation_step = rospy.get_param("~simulation_step")

        ReconfigurableClient.__init__(self, "gravity_filter", FILTER)

    def onReconfigure(self, worlds_names):
        """
        """
        if len(worlds_names) > 1:
            rospy.logerr("[%s] This filter have only one input. Ignoring the others.", self.node_name)
        world_name = worlds_names[0]
        p.connect(p.DIRECT) # Initialize bullet non-graphical version
        #rospy.loginfo("add search path :"+pybullet_data.getDataPath())
        #p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        #rospy.loginfo("add search path :"+self.ressource_folder)
        p.setAdditionalSearchPath(self.ressource_folder)
        p.setTimeStep(self.time_step)
        self.node_id_map = {}
        self.reverse_node_id_map = {}
        planeId = p.loadURDF("plane.urdf")
        self.node_id_map[self.worlds[world_name].scene.rootID] = planeId
        self.reverse_node_id_map[planeId] = self.worlds[world_name].scene.rootID
        pass

    def onSubscribeChanges(self, world_name):
        """
        """
        pass

    def onUnsubscribeChanges(self, world_name):
        """
        """
        pass

    def onChanges(self, world_name, header, invalidations):
        """
        """
        changes = self.filter(world_name, header, invalidations)
        self.sendWorldChanges(world_name+"_stable", header, changes)

    def filter(self, world_name, header, invalidations):
        """
        """
        changes = Changes()
        self.updateOutputNodes(invalidations.nodes_ids_updated)
        self.stepSimulation()

        for node in self.worlds[world_name].scene.nodes.items():
            node_id = node.id
            if node_id in self.node_id_map:
                if self.node_id_map[node_id] > 0:
                    t, q = p.getBasePositionAndOrientation(self.node_id_map[node_id])
                    node.position.pose.position = t
                    node.position.pose.orientation = q
                    changes.nodes_to_update.push_back(node)
        return changes

    def stepSimulation(self, simulation_step):
        """
        """
        range = self.simulation_step % self.time_step
        for i in range(0, range):
            p.stepSimulation()

    def updateBulletNodes(self, world_name, nodes_ids):
        """ This function load the urdf corresponding to the uwds node and set it in the environment
        The urdf need to have the same name than the node name
        :return:
        """
        for node_id in nodes_ids:
            if node_id != self.worlds[world_name].scene().rootID():
                position = [node.position.pose.position.x, node.position.pose.position.y, node.position.pose.position.z]
                orientation = [node.position.pose.orientation.x, node.position.pose.orientation.y, node.position.pose.orientation.z, node.position.pose.orientation.w]
                if node_id not in node_id_map:
                    node = input_world.scene.nodes[node_id]
                    bullet_id = p.loadURDF("urdf/"+node.name+".urdf", position, orientation)
                    self.node_id_map[node_id] = bullet_id
                    self.reverse_node_id_map[bullet_id] = node_id
                else:
                    if self.node_id_map[node_id] > 0:
                        p.resetBaseVelocity(self.node_id_map[node_id], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
                        p.resetBasePositionAndOrientation(self.node_id_map[node_id], position, orientation)


if __name__ == '__main__':
    rospy.init_node("gravity_filter")
    gf = GravityFilter()
    rospy.spin()
