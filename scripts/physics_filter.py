#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import pybullet as p
import numpy as np
import tf
import math
import copy
import uuid
from pyuwds.reconfigurable_client import ReconfigurableClient
from uwds_msgs.msg import Changes, Situation, Property
from pyuwds.types import FILTER, MESH, ACTION, FACT


STABLE = 0
UNSTABLE = 1
MOVING = 2
NOT_MOVING = 4

UNMARKED = 0
UNSTABLE = 1

PLACED = 0
PICKEDUP = 1
PUSHED = 2

PLACE_CONFIDENCE = 0.95
PICK_CONFIDENCE = 0.95

IN_CONFIDENCE = 0.75
ONTOP_CONFIDENCE = 0.95

EPSILON = 0.01  # 1cm

class PhysicsFilter(ReconfigurableClient):
    """
    """
    def __init__(self):
        """
        """
        self.ressource_folder = rospy.get_param("~ressource_folder")

        # reasoning parameters
        self.infer_actions = rospy.get_param("~infer_actions", True)
        self.perception_duration = rospy.get_param("~perception_duration", 0.8)
        self.simulation_tolerance = rospy.get_param("~simulation_tolerance", 0.05)
        self.perception_tolerance = rospy.get_param("~perception_tolerance", 0.01)

        # simulator parameters
        self.time_step = rospy.get_param("~time_step", 0.004)
        self.simulation_step = rospy.get_param("~simulation_step", 0.06)
        self.fall_simulation_step = rospy.get_param("~simulation_step", 0.13)
        self.nb_step_fall = int(self.fall_simulation_step / self.time_step)
        self.nb_step = int(self.simulation_step / self.time_step)
        self.jump_distance = rospy.get_param("~jump_distance", 0.05)

        # init simulator
        p.connect(p.GUI) # Initialize bullet non-graphical version
        p.setGravity(0, 0, -10)
        #p.setPhysicsEngineParameter(contactBreakingThreshold=0.01)
        p.setAdditionalSearchPath(self.ressource_folder)
        p.setTimeStep(self.time_step)

        self.bullet_node_id_map = {}
        self.previous_placed_positions = {}

        self.corrected_position = {}
        self.corrected_orientation = {}
        self.corrected_linear_velocity = {}
        self.corrected_angular_velocity = {}

        self.perceived_position = {}
        self.perceived_orientation = {}
        self.perceived_linear_velocity = {}
        self.perceived_angular_velocity = {}

        self.previous_position = {}
        self.previous_orientation = {}

        self.node_action_state = {}

        self.max_step = 10

        self.simulated_node_ids = []
        self.previous_perceived_position = {}
        self.previous_perceived_orientation = {}

        self.isPerceived = {}
        self.isUnstable = {}
        self.isMoving = {}

        self.isIn = {}
        self.isOnTop = {}
        self.isContaining = {}

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
        self.sendWorldChanges(world_name+"_stable", header, changes)

    def filter(self, world_name, header, invalidations):
        """
        """
        print "start reasoning"
        start_reasoning_time = rospy.Time.now()
        changes = Changes()

        for mesh_id in invalidations.mesh_ids_updated:
            changes.meshes_to_update.append(self.meshes[mesh_id])

        for situation_id in invalidations.situation_ids_updated:
            changes.situations_to_update.append(self.meshes[mesh_id])

        for node_id in invalidations.node_ids_updated:
            node = self.worlds[world_name].scene.nodes[node_id]
            if node_id not in self.isContaining:
                self.isContaining[node_id] = {}
            #print "received invalidation for <"+node.name+">"
            if node_id in self.perceived_position:
                self.previous_perceived_position[node_id] = self.perceived_position[node_id]
            if node_id in self.perceived_orientation:
                self.previous_perceived_orientation[node_id] = self.perceived_orientation[node_id]
            self.perceived_position[node_id] = [node.position.pose.position.x, node.position.pose.position.y, node.position.pose.position.z]
            self.perceived_orientation[node_id] = [node.position.pose.orientation.x, node.position.pose.orientation.y, node.position.pose.orientation.z, node.position.pose.orientation.w]
            self.perceived_linear_velocity[node_id] = [node.velocity.twist.linear.x, node.velocity.twist.linear.y, node.velocity.twist.linear.z]
            self.perceived_angular_velocity[node_id] = [node.velocity.twist.angular.x, node.velocity.twist.angular.y, node.velocity.twist.angular.z]
            if node_id not in self.previous_perceived_position:
                self.updateBulletNode(world_name, node_id, self.perceived_position[node_id], self.perceived_orientation[node_id], self.perceived_linear_velocity[node_id], self.perceived_angular_velocity[node_id])

            # if node_id in self.perceived_position:
            #     self.previous_perceived_position[node_id] = self.perceived_position[node_id]
            # if node_id in self.perceived_orientation:
            #     self.previous_perceived_orientation[node_id] = self.perceived_orientation[node_id]
        #
        for node_id, node in self.worlds[world_name].scene.nodes.items():
            if node.type == MESH:
                self.isPerceived[node_id] = (rospy.Time.now() - node.last_update.data) < rospy.Duration(self.perception_duration)
                if self.isPerceived[node_id]:
                    if node_id not in self.isUnstable:
                        self.isUnstable[node_id] = False
                    if self.isUnstable[node_id] is False:
                        if node_id in self.previous_perceived_position:
                            if not(np.allclose(self.previous_perceived_position[node_id], self.perceived_position[node_id], atol=self.perception_tolerance) \
                                    and np.allclose(self.previous_perceived_orientation[node_id], self.perceived_orientation[node_id], atol=self.perception_tolerance)):
                                self.updateBulletNode(world_name, node_id, self.perceived_position[node_id], self.perceived_orientation[node_id], self.perceived_linear_velocity[node_id], self.perceived_angular_velocity[node_id])
                                self.isMoving[node_id] = True
                                for object_id in self.isContaining[node_id]:
                                    object = self.worlds[world_name].scene.nodes[object_id]
                                    if object_id in self.previous_position:
                                        print "moving "+object.name
                                        if node_id in self.previous_position and object_id in self.previous_position:
                                            t_prev = tf.transformations.translation_matrix(self.previous_position[node_id])
                                            t_perceived = tf.transformations.translation_matrix(self.perceived_position[node_id])
                                            offset = tf.transformations.translation_from_matrix(np.dot(np.linalg.inv(t_prev), t_perceived))
                                            if not np.allclose(offset, [0, 0, 0], atol=0.03):
                                                object_position = [object.position.pose.position.x+offset[0], object.position.pose.position.y+offset[1], object.position.pose.position.z+offset[2]]
                                                new_object_orientation = self.previous_orientation[object_id]
                                                self.updateBulletNode(world_name, object_id, object_position, new_object_orientation, self.perceived_linear_velocity[node_id], self.perceived_angular_velocity[node_id])
                            else:
                                self.isMoving[node_id] = False
                    else:
                        self.updateBulletNode(world_name, node_id, self.perceived_position[node_id], self.perceived_orientation[node_id], self.perceived_linear_velocity[node_id], self.perceived_angular_velocity[node_id])
                        for object_id in self.isContaining[node_id]:
                            object = self.worlds[world_name].scene.nodes[object_id]
                            if object_id in self.previous_position:
                                print "moving "+object.name
                                if node_id in self.previous_position and object_id in self.previous_position:
                                    t_prev = tf.transformations.translation_matrix(self.previous_position[node_id])
                                    t_perceived = tf.transformations.translation_matrix(self.perceived_position[node_id])
                                    offset = tf.transformations.translation_from_matrix(np.dot(np.linalg.inv(t_prev), t_perceived))
                                    if not np.allclose(offset, [0, 0, 0], atol=0.03):
                                        object_position = [object.position.pose.position.x+offset[0], object.position.pose.position.y+offset[1], object.position.pose.position.z+offset[2]]
                                        new_object_orientation = self.previous_orientation[object_id]
                                        self.updateBulletNode(world_name, object_id, object_position, new_object_orientation, self.perceived_linear_velocity[node_id], self.perceived_angular_velocity[node_id])
                else:
                    pass
                    #self.updateBulletNode(world_name, node_id, self.perceived_position[node_id], self.perceived_orientation[node_id], self.perceived_linear_velocity[node_id], self.perceived_angular_velocity[node_id])
                    # for object_id in self.isContaining[node_id]:
                    #     object = self.worlds[world_name].scene.nodes[object_id]
                    #     if object_id in self.previous_position:
                    #         print "moving "+object.name
                    #         if node_id in self.previous_position and object_id in self.previous_position:
                    #             t_prev = tf.transformations.translation_matrix(self.previous_position[node_id])
                    #             t_perceived = tf.transformations.translation_matrix(self.perceived_position[node_id])
                    #             offset = tf.transformations.translation_from_matrix(np.dot(np.linalg.inv(t_prev), t_perceived))
                    #             if not np.allclose(offset, [0, 0, 0], atol=0.03):
                    #                 object_position = [object.position.pose.position.x+offset[0], object.position.pose.position.y+offset[1], object.position.pose.position.z+offset[2]]
                    #                 new_object_orientation = self.previous_orientation[object_id]
                    #                 self.updateBulletNode(world_name, object_id, object_position, new_object_orientation, self.perceived_linear_velocity[node_id], self.perceived_angular_velocity[node_id])

        #         if node_id not in self.isUnstable:
        #             self.isUnstable[node_id] = False
        #         if self.isPerceived[node_id] is True:
        #             if self.isUnstable[node_id] is False:
        #                 if node_id in self.previous_perceived_position:
        #                     if not(np.allclose(self.previous_perceived_position[node_id], self.perceived_position[node_id], atol=self.perception_tolerance) \
        #                             and np.allclose(self.previous_perceived_orientation[node_id], self.perceived_orientation[node_id], atol=self.perception_tolerance)):
        #                         self.updateBulletNode(world_name, node_id, self.perceived_position[node_id], self.perceived_orientation[node_id], self.perceived_linear_velocity[node_id], self.perceived_angular_velocity[node_id])
        #                         #if not np.allclose(self.previous_position[node_id], self.perceived_position[node_id], atol=self.):
        #                         if node_id not in self.isContaining:
        #                             self.isContaining[node_id] = {}
        #                         for object_id in self.isContaining[node_id]:
        #                             object = self.worlds[world_name].scene.nodes[object_id]
        #                             if object_id in self.previous_position:
        #                                 print "moving "+object.name
        #                                 if node_id in self.previous_position and object_id in self.previous_position:
        #                                     t_prev = tf.transformations.translation_matrix(self.previous_position[node_id])
        #                                     t_perceived = tf.transformations.translation_matrix(self.perceived_position[node_id])
        #                                     offset = tf.transformations.translation_from_matrix(np.dot(np.linalg.inv(t_prev), t_perceived))
        #                                     object_position = [object.position.pose.position.x+offset[0], object.position.pose.position.y+offset[1], object.position.pose.position.z+offset[2]]
        #                                     new_object_orientation = self.previous_orientation[object_id]
        #                                     self.updateBulletNode(world_name, object_id, object_position, new_object_orientation, self.perceived_linear_velocity[node_id], self.perceived_angular_velocity[node_id])
        #             else:
        #                 self.updateBulletNode(world_name, node_id, self.perceived_position[node_id], self.perceived_orientation[node_id], self.perceived_linear_velocity[node_id], self.perceived_angular_velocity[node_id])
                        # self.updateBulletNode(world_name, node_id, self.perceived_position[node_id], self.perceived_orientation[node_id], self.perceived_linear_velocity[node_id], self.perceived_angular_velocity[node_id])
                        # if node_id in self.previous_perceived_position:
                        #     if not(np.allclose(self.previous_perceived_position[node_id], self.perceived_position[node_id], atol=self.perception_tolerance) \
                        #             and np.allclose(self.previous_perceived_orientation[node_id], self.perceived_orientation[node_id], atol=self.perception_tolerance)):
                        #         if node_id not in self.isContaining:
                        #             self.isContaining[node_id] = {}
                        #         for object_id in self.isContaining[node_id]:
                        #             object = self.worlds[world_name].scene.nodes[object_id]
                        #             if object_id in self.previous_position:
                        #                 print "moving "+object.name
                        #                 if node_id in self.previous_position and object_id in self.previous_position:
                        #                     t_prev = tf.transformations.translation_matrix(self.previous_position[node_id])
                        #                     t_perceived = tf.transformations.translation_matrix(self.perceived_position[node_id])
                        #                     offset = tf.transformations.translation_from_matrix(np.dot(np.linalg.inv(t_prev), t_perceived))
                        #                     object_position = [object.position.pose.position.x+offset[0], object.position.pose.position.y+offset[1], object.position.pose.position.z+offset[2]]
                        #                     new_object_orientation = self.previous_orientation[object_id]
                        #                     self.updateBulletNode(world_name, object_id, object_position, new_object_orientation, self.perceived_linear_velocity[node_id], self.perceived_angular_velocity[node_id])

        state_id = p.saveState()
        start_fall_reasoning_time = rospy.Time.now()
        for node_id in self.simulated_node_ids:
            if self.isPerceived[node_id]:
                self.updateBulletNode(world_name, node_id, self.perceived_position[node_id], self.perceived_orientation[node_id], self.perceived_linear_velocity[node_id], self.perceived_angular_velocity[node_id])
            self.isUnstable[node_id] = False

        for i in range(0, self.nb_step_fall):
            p.stepSimulation()
            for node_id in self.simulated_node_ids:
                if self.isPerceived[node_id]:
                    node = self.worlds[world_name].scene.nodes[node_id]

                    infered_position, infered_orientation = p.getBasePositionAndOrientation(self.bullet_node_id_map[node_id])
                    infered_linear_velocity, infered_angular_velocity = p.getBaseVelocity(self.bullet_node_id_map[node_id])
                    perceived_position = self.perceived_position[node_id]
                    is_unstable = math.sqrt(pow(perceived_position[0]-infered_position[0], 2) + pow(perceived_position[1]-infered_position[1], 2) + pow(perceived_position[2]-infered_position[2], 2)) > self.simulation_tolerance
                    if self.isUnstable[node_id] is False and is_unstable:
                        self.isUnstable[node_id] = True
                        print node.name + " is unstable after "+str(i)+"/"+str(self.nb_step_fall)+" steps"

                #if self.isUnstable[node_id]:
                    #self.updateBulletNode(world_name, node_id, self.perceived_position[node_id], self.perceived_orientation[node_id], self.perceived_linear_velocity[node_id], self.perceived_angular_velocity[node_id])
        p.restoreState(state_id)
        end_fall_reasoning_time = rospy.Time.now()
        print "fall reasoning duration : "+str((end_fall_reasoning_time - start_fall_reasoning_time).to_sec())

        for i in range(0, self.nb_step):
            p.stepSimulation()
            for node_id in self.simulated_node_ids:
                if self.isUnstable[node_id] and self.isPerceived[node_id]:
                    node = self.worlds[world_name].scene.nodes[node_id]
                    #print node.name + " is unstable"
                    self.updateBulletNode(world_name, node_id, self.perceived_position[node_id], self.perceived_orientation[node_id], self.perceived_linear_velocity[node_id], self.perceived_angular_velocity[node_id])

        for node_id, node in self.worlds[world_name].scene.nodes.items():
            # print len(self.simulated_node_ids)
            if node_id in self.simulated_node_ids:
                if self.isUnstable[node_id] is True and self.isPerceived[node_id] is True:
                    if node_id in self.node_action_state:
                        if self.node_action_state[node_id] == PLACED and self.infer_actions:
                            print node.name + " picked up"
                            situation = Situation()
                            situation.id = str(uuid.uuid4())
                            situation.type = ACTION
                            situation.description = node.name + " picked up"
                            situation.confidence = PICK_CONFIDENCE
                            situation.start.data = header.stamp
                            situation.end.data = header.stamp
                            situation.properties.append(Property("subject", node_id))
                            situation.properties.append(Property("action", "Place"))
                            changes.situations_to_update.append(situation)
                    self.node_action_state[node_id] = PICKEDUP
                    node.position.pose.position.x = self.perceived_position[node_id][0]
                    node.position.pose.position.y = self.perceived_position[node_id][1]
                    node.position.pose.position.z = self.perceived_position[node_id][2]
                    node.position.pose.orientation.x = self.perceived_orientation[node_id][0]
                    node.position.pose.orientation.y = self.perceived_orientation[node_id][1]
                    node.position.pose.orientation.z = self.perceived_orientation[node_id][2]
                    node.position.pose.orientation.w = self.perceived_orientation[node_id][3]
                    node.velocity.twist.linear.x = self.perceived_linear_velocity[node_id][0]
                    node.velocity.twist.linear.y = self.perceived_linear_velocity[node_id][1]
                    node.velocity.twist.linear.z = self.perceived_linear_velocity[node_id][2]
                    node.velocity.twist.angular.x = self.perceived_angular_velocity[node_id][0]
                    node.velocity.twist.angular.y = self.perceived_angular_velocity[node_id][1]
                    node.velocity.twist.angular.z = self.perceived_angular_velocity[node_id][2]
                    self.previous_position[node_id] = self.perceived_position[node_id]
                    self.previous_orientation[node_id] = self.perceived_orientation[node_id]
                    self.updateBulletNode(world_name, node_id, self.perceived_position[node_id], self.perceived_orientation[node_id], self.perceived_linear_velocity[node_id], self.perceived_angular_velocity[node_id])
                    changes.nodes_to_update.append(node)
                else:
                    if node_id in self.node_action_state:
                        if self.node_action_state[node_id] == PICKEDUP and self.infer_actions:
                            print node.name + " placed"
                            situation = Situation()
                            situation.id = str(uuid.uuid4())
                            situation.type = ACTION
                            situation.description = node.name + " placed"
                            situation.confidence = PLACE_CONFIDENCE
                            situation.start.data = header.stamp
                            situation.end.data = header.stamp
                            situation.properties.append(Property("subject", node_id))
                            situation.properties.append(Property("action", "Pick"))
                            changes.situations_to_update.append(situation)
                        self.node_action_state[node_id] = PLACED
                    infered_position, infered_orientation = p.getBasePositionAndOrientation(self.bullet_node_id_map[node_id])
                    infered_linear_velocity, infered_angular_velocity = p.getBaseVelocity(self.bullet_node_id_map[node_id])
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
                    self.previous_position[node_id] = infered_position
                    self.previous_orientation[node_id] = infered_orientation
                    changes.nodes_to_update.append(node)
            else:
                if node_id in invalidations.node_ids_updated:
                    changes.nodes_to_update.append(node)

        now = rospy.Time.now()
        for node1_id in self.simulated_node_ids:
            node1 = self.worlds[world_name].scene.nodes[node1_id]
            if node1.type != MESH:
                continue
            for node2_id in self.simulated_node_ids:
                node2 = self.worlds[world_name].scene.nodes[node2_id]
                if node1.id == node2.id:
                    continue
                if node2.type != MESH:
                    continue
                bb1 = self.aabb(node1)
                bb2 = self.aabb(node2)
                if node1.id not in self.isIn:
                    self.isIn[node1.id] = {}
                if node1.id not in self.isOnTop:
                    self.isOnTop[node1.id] = {}
                if node2.id not in self.isContaining:
                    self.isContaining[node2.id] = {}
                if self.isin(bb1, bb2, node2.id in self.isIn[node1.id]):
                    if node2.id not in self.isIn[node1.id]:
                        sit = Situation()
                        sit.id = str(uuid.uuid4())
                        sit.type = FACT
                        sit.description = node1.name + " is in " + node2.name
                        sit.properties.append(Property("subject", node1.id))
                        sit.properties.append(Property("object", node2.id))
                        sit.properties.append(Property("predicate", "isIn"))
                        sit.confidence = IN_CONFIDENCE
                        sit.start.data = now
                        sit.end.data = rospy.Time(0)
                        self.isIn[node1.id][node2.id] = sit
                        self.isContaining[node2.id][node1.id] = sit
                        changes.situations_to_update.append(sit)
                else:
                    if node2.id in self.isIn[node1.id]:
                        self.isIn[node1.id][node2.id].end.data = now
                        sit = self.isIn[node1.id][node2.id]
                        changes.situations_to_update.append(sit)
                        del self.isIn[node1.id][node2.id]
                        del self.isContaining[node2.id][node1.id]

                if self.isontop(bb1, bb2, node2.id in self.isOnTop[node1.id]):
                    if node2.id not in self.isOnTop[node1.id]:
                        sit = Situation()
                        sit.id = str(uuid.uuid4())
                        sit.type = FACT
                        sit.description = node1.name + " is on " + node2.name
                        sit.properties.append(Property("subject", node1.id))
                        sit.properties.append(Property("object", node2.id))
                        sit.properties.append(Property("predicate", "isOnTop"))
                        sit.confidence = ONTOP_CONFIDENCE
                        sit.start.data = now
                        sit.end.data = rospy.Time(0)
                        self.isOnTop[node1.id][node2.id] = sit
                        changes.situations_to_update.append(sit)
                else:
                    if node2.id in self.isOnTop[node1.id]:
                        self.isOnTop[node1.id][node2.id].end.data = now
                        sit = self.isOnTop[node1.id][node2.id]
                        changes.situations_to_update.append(sit)
                        del self.isOnTop[node1.id][node2.id]

        end_reasoning_time = rospy.Time.now()
        #print "reasoning duration : "+str((end_reasoning_time - start_reasoning_time).to_sec())
        return changes

    def updateBulletNode(self, world_name, node_id, position, orientation, linear, angular):
        """
        """
        if self.worlds[world_name].scene.rootID not in self.bullet_node_id_map:
            self.bullet_node_id_map[self.worlds[world_name].scene.rootID] = p.loadURDF("plane.urdf")

        node = self.worlds[world_name].scene.nodes[node_id]
        if node_id not in self.bullet_node_id_map:
            try:
                self.bullet_node_id_map[node_id] = p.loadURDF(node.name+".urdf", position, orientation)
                rospy.loginfo("[%s::updateBulletNodeNodes] "+node.name+".urdf' loaded successfully", self.node_name)
                p.changeDynamics(self.bullet_node_id_map[node_id], -1, frictionAnchor=1, rollingFriction=1.0, spinningFriction=1.0, lateralFriction=1.0)
                self.simulated_node_ids.append(node_id)
                if node_id not in self.node_action_state:
                    self.node_action_state[node_id] = PLACED
            except Exception as e:
                self.bullet_node_id_map[node_id] = -1
                rospy.logwarn("[%s::updateBulletNodeNodes] "+str(e))
        if self.bullet_node_id_map[node_id] > 0:
            p.resetBaseVelocity(self.bullet_node_id_map[node_id], linear, angular)
            p.resetBasePositionAndOrientation(self.bullet_node_id_map[node_id], position, orientation)
        else:
            self.bullet_node_id_map[node_id] = -1

    def aabb(self, node):
        """
        Compute world aabb by transforming the corners of the aabb by the node pose
        """
        for property in node.properties:
            if property.name == "aabb":
                aabb = property.data.split(",")
                if len(aabb) == 3:

                    t = [node.position.pose.position.x, node.position.pose.position.y, node.position.pose.position.z]
                    q = [node.position.pose.orientation.x, node.position.pose.orientation.y, node.position.pose.orientation.z, node.position.pose.orientation.w]
                    trans = tf.transformations.translation_matrix(t)
                    rot = tf.transformations.quaternion_matrix(q)
                    transform = tf.transformations.concatenate_matrices(trans, rot)
                    v = []
                    v.append(tf.transformations.translation_from_matrix(np.dot(transform, tf.transformations.translation_matrix([ float(aabb[0])/2,  float(aabb[1])/2, float(aabb[2])/2]))))
                    v.append(tf.transformations.translation_from_matrix(np.dot(transform, tf.transformations.translation_matrix([-float(aabb[0])/2,  float(aabb[1])/2, float(aabb[2])/2]))))
                    v.append(tf.transformations.translation_from_matrix(np.dot(transform, tf.transformations.translation_matrix([ float(aabb[0])/2, -float(aabb[1])/2, float(aabb[2])/2]))))
                    v.append(tf.transformations.translation_from_matrix(np.dot(transform, tf.transformations.translation_matrix([-float(aabb[0])/2, -float(aabb[1])/2, float(aabb[2])/2]))))
                    v.append(tf.transformations.translation_from_matrix(np.dot(transform, tf.transformations.translation_matrix([ float(aabb[0])/2,  float(aabb[1])/2, -float(aabb[2])/2]))))
                    v.append(tf.transformations.translation_from_matrix(np.dot(transform, tf.transformations.translation_matrix([-float(aabb[0])/2,  float(aabb[1])/2, -float(aabb[2])/2]))))
                    v.append(tf.transformations.translation_from_matrix(np.dot(transform, tf.transformations.translation_matrix([ float(aabb[0])/2, -float(aabb[1])/2, -float(aabb[2])/2]))))
                    v.append(tf.transformations.translation_from_matrix(np.dot(transform, tf.transformations.translation_matrix([-float(aabb[0])/2, -float(aabb[1])/2, -float(aabb[2])/2]))))
                    bb_min = [1e10, 1e10, 1e10]
                    bb_max = [-1e10, -1e10, -1e10]
                    for vertex in v:
                        bb_min = np.minimum(bb_min, vertex)
                        bb_max = np.maximum(bb_max, vertex)
                    return bb_min, bb_max
        raise RuntimeError("aabb not present")

    def bb_footprint(self, bb):
        """
        Copied from severin lemaignan underworlds client example :
        see : https://github.com/severin-lemaignan/underworlds/blob/master/clients/spatial_relations.py
        """
        x1, y1, z1 = bb[0]
        x2, y2, z2 = bb[1]

        return (x1, y1), (x2, y2)

    def overlap(self, rect1, rect2, prev=False):
        """Overlapping rectangles overlap both horizontally & vertically
        Coped from severin lemaignan underworlds client example :
        see : https://github.com/severin-lemaignan/underworlds/blob/master/clients/spatial_relations.py
        """
        (l1, b1), (r1, t1) = rect1
        (l2, b2), (r2, t2) = rect2
        return self.range_overlap(l1, r1, l2, r2) and self.range_overlap(b1, t1, b2, t2)

    def range_overlap(self, a_min, a_max, b_min, b_max, prev=False):
        """Neither range is completely greater than the other

        http://codereview.stackexchange.com/questions/31352/overlapping-rectangles
        Modified from severin lemaignan underworlds client example :
        see : https://github.com/severin-lemaignan/underworlds/blob/master/clients/spatial_relations.py
        """
        if prev is False:
            return (a_min <= b_max - EPSILON) and (b_min <= a_max + EPSILON)
        else:
            return (a_min <= b_max + EPSILON) and (b_min <= a_max - EPSILON)

    def weakly_cont(self, rect1, rect2, prev=False):
        """Obj1 is weakly contained if the base of the object is surrounded
        by Obj2
        Modified from severin lemaignan underworlds client example :
        see : https://github.com/severin-lemaignan/underworlds/blob/master/clients/spatial_relations.py
        """
        (l1, b1), (r1, t1) = rect1
        (l2, b2), (r2, t2) = rect2
        if prev is False:
            return (l1 >= l2 - EPSILON) and (b1 >= b2 - EPSILON) and (r1 <= r2 + EPSILON) and (t1 <= t2 - EPSILON)
        else:
            return (l1 >= l2 - EPSILON) and (b1 >= b2 - EPSILON) and (r1 <= r2 - EPSILON) and (t1 <= t2 + EPSILON)

    def isabove(self, bb1, bb2, prev=False):
        """
        For obj 1 to be above obj 2:
        - the bottom of its bounding box must be higher that
          the top of obj 2's bounding box
        - the bounding box footprint of both objects must overlap
        Modified from severin lemaignan underworlds client example :
        see : https://github.com/severin-lemaignan/underworlds/blob/master/clients/spatial_relations.py
        """

        bb1_min, _ = bb1
        _, bb2_max = bb2

        x1, y1, z1 = bb1_min
        x2, y2, z2 = bb2_max

        if z1 < z2 - EPSILON:
            return False

        return self.overlap(self.bb_footprint(bb1), self.bb_footprint(bb2))

    def isin(self, bb1, bb2, prev=False):
        """ Returns True if bb1 is in bb2.

        To be 'in' bb1 is weakly contained by bb2 and the bottom of bb1 is lower
        than the top of bb2 and higher than the bottom of bb2.
        Modified from severin lemaignan underworlds client example :
        see : https://github.com/severin-lemaignan/underworlds/blob/master/clients/spatial_relations.py
        """
        bb1_min, _ = bb1
        bb2_min, bb2_max = bb2

        x1, y1, z1 = bb1_min
        x2, y2, z2 = bb2_max
        x3, y3, z3 = bb2_min

        if z1 > z2 - EPSILON:
            return False

        if z1 < z3 + EPSILON:
            return False

        return self.weakly_cont(self.bb_footprint(bb1), self.bb_footprint(bb2))

    def isontop(self, bb1, bb2, prev=False):
        """
        For obj 1 to be on top of obj 2:
         - obj1 must be above obj 2
         - the bottom of obj 1 must be close to the top of obj 2
        Modified from severin lemaignan underworlds client example :
        see : https://github.com/severin-lemaignan/underworlds/blob/master/clients/spatial_relations.py
        """
        bb1_min, _ = bb1
        _, bb2_max = bb2

        x1, y1, z1 = bb1_min
        x2, y2, z2 = bb2_max

        return z1 < z2 + EPSILON and self.isabove(bb1, bb2)


if __name__ == '__main__':
    rospy.init_node("physics_filter")
    physics_filter = PhysicsFilter()
    rospy.spin()
