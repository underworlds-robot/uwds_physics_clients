#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import pybullet as p
import math
from pyuwds.reconfigurable_client import ReconfigurableClient
from uwds_msgs.msg import Changes
from pyuwds.types import FILTER, MESH


class GravityFilter(ReconfigurableClient):
    """
    """
    def __init__(self):
        """
        """
        self.ressource_folder = rospy.get_param("~ressource_folder")
        self.time_step = rospy.get_param("~time_step", 0.01)
        self.simulation_step = rospy.get_param("~simulation_step", 1.0)
        self.max_distance = rospy.get_param("~max_distance", 0.05) # max distance between perceived and simulated before considerig the simulation output false
        self.max_duration = rospy.get_param("~max_duration", 0.5) # max duration in seconds between last observation and current time before considering an object as not perceived
        p.connect(p.DIRECT) # Initialize bullet non-graphical version
        p.setGravity(0, 0, -10)
        p.setAdditionalSearchPath(self.ressource_folder)
        p.setTimeStep(self.time_step)
        self.node_id_map = {}
        self.reverse_node_id_map = {}
        self.perceived = {}
        ReconfigurableClient.__init__(self, "gravity_filter", FILTER)

    def onReconfigure(self, worlds_names):
        """
        """

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
        if len(changes.nodes_to_update) > 0:
            self.sendWorldChanges(world_name+"_stable", header, changes)

    def filter(self, world_name, header, invalidations):
        """
        """
        changes = Changes()

        for node_id in invalidations.node_ids_updated:
            if node_id not in self.perceived:
                self.perceived[node_id] = 1.0
            else:
                self.perceived[node_id] = 1.0

            self.updateBulletNodes(world_name, node_id)

        self.stepSimulation()

        for node_id, node in self.worlds[world_name].scene.nodes.items():
            if node_id in self.node_id_map:
                if self.node_id_map[node_id] > 0:
                    t, q = p.getBasePositionAndOrientation(self.node_id_map[node_id])
                    p.resetBaseVelocity(self.node_id_map[node_id], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
                    self.perceived[node_id] *= 0.97
                    x, y, z = t
                    x_dist = node.position.pose.position.x - x
                    y_dist = node.position.pose.position.y - y
                    z_dist = node.position.pose.position.z - z
                    dist = math.sqrt((x_dist*x_dist) + (y_dist*y_dist) + (z_dist*z_dist))
                    if dist > self.max_distance and self.perceived[node_id] > 0.7:
                        position = [node.position.pose.position.x, node.position.pose.position.y, node.position.pose.position.z]
                        orientation = [node.position.pose.orientation.x, node.position.pose.orientation.y, node.position.pose.orientation.z, node.position.pose.orientation.w]
                        p.resetBasePositionAndOrientation(self.node_id_map[node_id], position, orientation)
                        changes.nodes_to_update.append(node)
                    else:
                        node.position.pose.position.x = x
                        node.position.pose.position.y = y
                        node.position.pose.position.z = z
                        x, y, z, w = q
                        node.position.pose.orientation.x = x
                        node.position.pose.orientation.y = y
                        node.position.pose.orientation.z = z
                        node.position.pose.orientation.w = w
                        changes.nodes_to_update.append(node)
                else:
                    if node_id in invalidations.node_ids_updated:
                        changes.nodes_to_update.append(node)

        for mesh_id in invalidations.mesh_ids_updated:
            changes.meshes_to_update.append(self.meshes[mesh_id])
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
        if self.worlds[world_name].scene.rootID not in self.node_id_map:
            self.node_id_map[self.worlds[world_name].scene.rootID] = p.loadURDF("plane.urdf")
        node = self.worlds[world_name].scene.nodes[node_id]
        if node.type == MESH:
            position = [node.position.pose.position.x, node.position.pose.position.y, node.position.pose.position.z]
            orientation = [node.position.pose.orientation.x, node.position.pose.orientation.y, node.position.pose.orientation.z, node.position.pose.orientation.w]
            if node_id not in self.node_id_map:
                try:
                    self.node_id_map[node_id] = p.loadURDF(node.name+".urdf", position, orientation)
                    rospy.loginfo("[%s::updateBulletNodes] "+node.name+".urdf' loaded successfully", self.node_name)
                except Exception as e:
                    self.node_id_map[node_id] = -1
                if self.node_id_map[node_id] > 0:
                    self.reverse_node_id_map[self.node_id_map[node_id]] = node_id
            else:
                if self.node_id_map[node_id] > 0:
                    p.resetBaseVelocity(self.node_id_map[node_id], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
                    p.resetBasePositionAndOrientation(self.node_id_map[node_id], position, orientation)
        else:
            self.node_id_map[node_id] = -1


if __name__ == '__main__':
    rospy.init_node("gravity_filter")
    gf = GravityFilter()
    rospy.spin()
