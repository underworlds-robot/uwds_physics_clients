#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import pybullet as p
import math
import uuid
from pyuwds.reconfigurable_client import ReconfigurableClient
from uwds_msgs.msg import Changes, Situation, Property
from pyuwds.types import FILTER, MESH, ACTION

STABLE = 0
UNSTABLE = 1
MOVING = 2
NOT_MOVING = 4

UNMARKED = 0
UNSTABLE = 1

PLACE_CONFIDENCE = 0.95
PICK_CONFIDENCE = 0.95
HOLD_CONFIDENCE = 0.95
PUSH_CONFIDENCE = 0.95
RELEASE_CONFIDENCE = 0.55


class GravityFilter(ReconfigurableClient):
    """
    """
    def __init__(self):
        """
        """
        self.ressource_folder = rospy.get_param("~ressource_folder")
        self.time_step = rospy.get_param("~time_step", 0.01)
        self.simulation_step = rospy.get_param("~simulation_step", 1.0)
        self.max_distance = rospy.get_param("~max_distance", 0.06) # max distance between perceived and simulated before considerig the simulation output false
        self.max_duration = rospy.get_param("~max_duration", 0.5) # max duration in seconds between last observation and current time before considering an object as not perceived
        p.connect(p.DIRECT) # Initialize bullet non-graphical version
        p.setGravity(0, 0, -10)
        p.setPhysicsEngineParameter(contactBreakingThreshold=0.01)#, solverResidualThreshold=0.0001)
        p.setAdditionalSearchPath(self.ressource_folder)
        p.setTimeStep(self.time_step)
        self.node_id_map = {}
        self.reverse_node_id_map = {}
        self.node_state = {}
        self.node_simulation_state = {}
        self.previous_placed_positions = {}
        self.previous_positions = {}
        self.previous_orientations = {}
        self.push_situations = {}
        self.hold_situations = {}
        self.last_invalidation_time = {}
        ReconfigurableClient.__init__(self, "gravity_filter", FILTER)

    def onReconfigure(self, worlds_names):
        """
        """
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
        if len(changes.nodes_to_update) > 0:
            self.sendWorldChanges(world_name+"_stable", header, changes)

    def filter(self, world_name, header, invalidations):
        """
        """
        changes = Changes()

        for node_id in invalidations.node_ids_updated:
            time = rospy.Time.now()
            if node_id not in self.last_invalidation_time:
                self.last_invalidation_time[node_id] = time
            else:
                self.last_invalidation_time[node_id] = time
            self.updateBulletNodes(world_name, node_id)

        self.stepSimulation()

        for node_id, node in self.worlds[world_name].scene.nodes.items():
            if node_id in self.node_id_map:
                if self.node_id_map[node_id] > 0:
                    # Initialize
                    if node_id not in self.node_state:
                        self.node_state[node_id] = NOT_MOVING
                    if node_id not in self.node_simulation_state:
                        self.node_simulation_state[node_id] = STABLE
                    infered_position, infered_orientation = p.getBasePositionAndOrientation(self.node_id_map[node_id])
                    infered_linear_velocity, infered_angular_velocity = p.getBaseVelocity(self.node_id_map[node_id])
                    #p.resetBaseVelocity(self.node_id_map[node_id], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
                    is_perceived = (rospy.Time.now() - self.last_invalidation_time[node_id]) < rospy.Duration(self.max_duration)
                    x1, y1, z1 = infered_position
                    dist_between_last_perceived_and_infered = math.sqrt(pow(node.position.pose.position.x-x1, 2) + pow(node.position.pose.position.y-y1, 2) + pow(node.position.pose.position.z-z1, 2))

                    if is_perceived and dist_between_last_perceived_and_infered > self.max_distance:
                        if self.node_simulation_state[node_id] == STABLE:
                            print node.name + " picked up"
                            situation = Situation()
                            situation.id = str(uuid.uuid4())
                            situation.type = ACTION
                            situation.description = node.name + " picked up"
                            situation.confidence = PICK_CONFIDENCE
                            situation.start.data = header.stamp
                            situation.end.data = header.stamp
                            subject_property = Property()
                            subject_property.name = "subject"
                            subject_property.data = node_id
                            situation.properties.append(subject_property)
                            action_property = Property()
                            action_property.name = "action"
                            action_property.data = "Pick"
                            situation.properties.append(action_property)
                            changes.situations_to_update.append(situation)
                            self.node_simulation_state[node_id] = UNSTABLE
                        self.node_state[node_id] = MOVING
                        perceived_position = [node.position.pose.position.x, node.position.pose.position.y, node.position.pose.position.z]
                        perceived_orientation = [node.position.pose.orientation.x, node.position.pose.orientation.y, node.position.pose.orientation.z, node.position.pose.orientation.w]
                        perceived_linear_velocity = [node.velocity.twist.linear.x, node.velocity.twist.linear.y, node.velocity.twist.linear.z]
                        perceived_angular_velocity = [node.velocity.twist.angular.x, node.velocity.twist.angular.y, node.velocity.twist.angular.z]
                        self.previous_positions[node_id] = perceived_position
                        self.previous_orientations[node_id] = perceived_orientation
                        p.resetBasePositionAndOrientation(self.node_id_map[node_id], perceived_position, perceived_orientation)
                        p.resetBaseVelocity(self.node_id_map[node_id], perceived_linear_velocity, perceived_angular_velocity)
                        changes.nodes_to_update.append(node)
                    else:
                        if self.node_state[node_id] == MOVING and self.node_simulation_state[node_id] == UNSTABLE:
                            print node.name + " placed"
                            situation = Situation()
                            situation.id = str(uuid.uuid4())
                            situation.type = ACTION
                            situation.description = node.name + " placed"
                            situation.confidence = PLACE_CONFIDENCE
                            situation.start.data = header.stamp
                            situation.end.data = header.stamp
                            subject_property = Property()
                            subject_property.name = "subject"
                            subject_property.data = node_id
                            situation.properties.append(subject_property)
                            action_property = Property()
                            action_property.name = "action"
                            action_property.data = "Place"
                            situation.properties.append(action_property)
                            changes.situations_to_update.append(situation)
                            self.node_state[node_id] = NOT_MOVING

                        if node_id not in self.previous_placed_positions:
                            self.previous_placed_positions[node_id] = infered_position
                        x1, y1, z1 = infered_position
                        x2, y2, z2 = self.previous_placed_positions[node_id]

                        self.previous_positions[node_id] = infered_position
                        self.previous_orientations[node_id] = infered_orientation

                        self.node_simulation_state[node_id] = STABLE
                        x, y, z = infered_position
                        node.position.pose.position.x = x
                        node.position.pose.position.y = y
                        node.position.pose.position.z = z
                        x, y, z, w = infered_orientation
                        node.position.pose.orientation.x = x
                        node.position.pose.orientation.y = y
                        node.position.pose.orientation.z = z
                        node.position.pose.orientation.w = w
                        x, y, z = infered_linear_velocity
                        node.velocity.twist.linear.x = x
                        node.velocity.twist.linear.y = y
                        node.velocity.twist.linear.z = z
                        x, y, z = infered_angular_velocity
                        node.velocity.twist.angular.x = x
                        node.velocity.twist.angular.y = y
                        node.velocity.twist.angular.z = z
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
            position = [node.position.pose.position.x, node.position.pose.position.y, node.position.pose.position.z]# + self.max_distance]
            orientation = [node.position.pose.orientation.x, node.position.pose.orientation.y, node.position.pose.orientation.z, node.position.pose.orientation.w]
            if node_id not in self.node_id_map:
                try:
                    self.node_id_map[node_id] = p.loadURDF(node.name+".urdf", position, orientation)
                    rospy.loginfo("[%s::updateBulletNodes] ''"+node.name+".urdf' loaded successfully", self.node_name)
                    p.changeDynamics(self.node_id_map[node_id], -1, frictionAnchor=1, rollingFriction=1.0, spinningFriction=1.0, lateralFriction=10.0)
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
