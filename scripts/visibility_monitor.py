#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import math
import uuid
import numpy
import pybullet as p
import pybullet_data
from pyuwds.reconfigurable_client import ReconfigurableClient
from uwds_msgs.msg import Changes, Situation, Property
from pyuwds.types.nodes import MESH, CAMERA
from pyuwds.types.situations import FACT
from pyuwds.uwds import FILTER
import tf

class VisibilityMonitor(ReconfigurableClient):
    def __init__(self):
        """
        """
        self.ressource_folder = rospy.get_param("~ressource_folder")
        p.connect(p.DIRECT) # Initialize bullet non-graphical version
        p.setAdditionalSearchPath(self.ressource_folder)
        self.urdf_available = {}
        self.node_id_map = {}
        self.reverse_node_id_map = {}
        self.min_treshold = rospy.get_param("~min_treshold", 0.4)
        self.width = rospy.get_param("~width", 480/8)
        self.height = rospy.get_param("~height", 360/8)
        ReconfigurableClient.__init__(self, "visibility_monitor", FILTER)

    def onReconfigure(self, worlds):
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
        changes = self.monitor(world_name, header, invalidations)
        if len(changes.situations_to_update) > 0:
            self.sendWorldChanges(world_name+"_visibilities", header, changes)
            rospy.loginfo("[%s::onChanges] Changes send (%d situations) in world <%s>", self.node_name, len(changes.situations_to_update), "visibilities")

    def monitor(self, world_name, header, invalidations):
        """
        """
        changes = Changes()
        for node_id in invalidations.node_ids_updated:
            self.updateBulletNodes(world_name, node_id)

        evaluated = []
        not_evaluated = []
        for node in self.worlds()[world_name].scene().nodes():
            #if node_id in invalidations.node_ids_updated:
            if node.type == CAMERA:
                if node.id in invalidations.node_ids_updated:
                    visibilities = self.computeVisibilities(world_name, node.id)
                    situations = self.updateSituations(world_name, header, node.id, visibilities)
                    for situation in situations:
                        changes.situations_to_update.append(situation)
                    evaluated.append(node.id)

        for node in self.worlds()[world_name].scene().nodes():
            if node.type == CAMERA:
                if (rospy.Time() - node.last_observation.data).to_sec() > 1.0:
                    situations = self.updateSituations(world_name, header, node.id, {})
                    for situation in situations:
                        changes.situations_to_update.append(situation)
        return changes

    def computeVisibilities(self, world_name, camera_id):
        """
        """
        visibilities = {}
        mean_distances_from_center = {}
        nb_pixel = {}
        relative_sizes = {}
        if camera_id in self.worlds[world_name].scene.nodes:
            camera_node = self.worlds[world_name].scene.nodes[camera_id]
            position = [camera_node.position.pose.position.x, camera_node.position.pose.position.y, camera_node.position.pose.position.z]
            orientation = [camera_node.position.pose.orientation.x, camera_node.position.pose.orientation.y, camera_node.position.pose.orientation.z, camera_node.position.pose.orientation.w]
            euler = tf.transformations.euler_from_quaternion(orientation)
            view_matrix = p.computeViewMatrixFromYawPitchRoll(position, -0.5, math.degrees(euler[2]), math.degrees(euler[1]), math.degrees(euler[1]), 2)
            fov = float(self.worlds[world_name].scene.getNodeProperty(camera_id, "hfov"))
            clipnear = float(self.worlds[world_name].scene.getNodeProperty(camera_id, "clipnear"))
            clipfar = float(self.worlds[world_name].scene.getNodeProperty(camera_id, "clipfar"))
            aspect = float(self.worlds[world_name].scene.getNodeProperty(camera_id, "aspect"))
            proj_matrix = p.computeProjectionMatrixFOV(40.0, aspect, clipnear, clipfar)
            width, height, rgb, depth, seg = p.getCameraImage(self.width, self.height, viewMatrix=view_matrix, projectionMatrix=proj_matrix, flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
            max_nb_pixel = 0
            background_nb_pixel = 0
            max_visibility = 0
            r = min(width, height)
            for line in range(0, height):
                for col in range(0, width):
                    pixel = seg[line][col]
                    if (pixel >= 0):
                        bullet_id = pixel & ((1 << 24)-1)
                        if bullet_id in self.reverse_node_id_map:
                            uwds_id = self.reverse_node_id_map[bullet_id]
                            if uwds_id != self.worlds[world_name].scene.rootID:
                                if uwds_id not in mean_distances_from_center:
                                    mean_distances_from_center[uwds_id] = 0.0
                                line_dist = (line-(line/2.0))
                                col_dist = (col-(col/2.0))
                                #dist_from_center = math.sqrt(line_dist*line_dist+col_dist*col_dist)
                                dist_from_center = math.sqrt(col_dist*col_dist)
                                mean_distances_from_center[uwds_id] += dist_from_center
                                if uwds_id not in nb_pixel:
                                    nb_pixel[uwds_id] = 0
                                nb_pixel[uwds_id] += 1
                                if uwds_id not in visibilities:
                                    visibilities[uwds_id] = 0
                                visibilities[uwds_id] += 1 - min(1, dist_from_center/r)
                                if max_visibility < visibilities[uwds_id]:
                                    max_visibility = visibilities[uwds_id]
                    else:
                        background_nb_pixel += 1.0

            if len(mean_distances_from_center) > 0:
                print "camera <%s> :" % self.worlds[world_name].scene.nodes[camera_id].name
                for node_id, mean_dist in mean_distances_from_center.items():
                    mean_distances_from_center[node_id] = mean_dist / nb_pixel[node_id]
                    camera_node = self.worlds[world_name].scene.nodes[camera_id]
                    object_node = self.worlds[world_name].scene.nodes[node_id]
                    if mean_distances_from_center[node_id] < r:
                        visibilities[node_id] = 1 - mean_distances_from_center[node_id]/r
                    else:
                        visibilities[node_id] = 0
                    if visibilities[node_id] > self.min_treshold:
                        print " - see object <%s> with %5f confidence" % (object_node.name, visibilities[node_id])
                    else:
                        del visibilities[node_id]
        return visibilities

    def updateSituations(self, world_name, header, camera_id, visibilities):
        """
        """
        situations = []
        situations_to_update = []
        situations_ids_ended = []
        for situation in self.worlds[world_name].timeline.situations.values():
            if situation.end.data == rospy.Time(0):
                if self.worlds()[world_name].timeline().get_situation_property(situation.id, "predicate") == "isVisible":
                    subject_id = self.worlds()[world_name].timeline().get_situation_property(situation.id, "subject")
                    object_id = self.worlds()[world_name].timeline().get_situation_property(situation.id, "object")
                    if camera_id == subject_id:
                        if object_id in visibilities:
                            situation.confidence = visibilities[object_id]
                            print "update : %s with %f confidence" % (situation.description, visibilities[object_id])
                            del visibilities[object_id]
                            situations_to_update.append(situation)
                        else:
                            situation.end.data = header.stamp
                            print "end : %s" % situation.description
                            situations_ids_ended.append(situation.id)
                        situations.append(situation)


        for id_seen, visibility_score in visibilities.items():
            situation = Situation()
            situation.id = str(uuid.uuid4())
            situation.type = FACT
            situation.description = self.worlds[world_name].scene.nodes[id_seen].name + " is visible by " + self.worlds[world_name].scene.nodes[camera_id].name
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
            situation.start.data = header.stamp
            situation.end.data = rospy.Time(0)
            print "start : %s" % situation.description
            situations_to_update.append(situation)
            situations.append(situation)

        self.worlds()[world_name].timeline().update(situations_to_update)
        self.worlds()[world_name].timeline().remove(situations_ids_ended)
        return situations

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
    rospy.init_node("visibility_monitor")
    vm = VisibilityMonitor()
    rospy.spin()
