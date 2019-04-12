#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import math
import uuid
import numpy
from pyuwds.reconfigurable_client import ReconfigurableClient
from uwds_msgs.msg import Changes, Situation, Property
from pyuwds.types import FILTER, MESH, CAMERA, FACT
import tf

class BeliefsFilter(ReconfigurableClient):
    def __init__(self):
        """
        """
        self.beliefs_map = {}
        ReconfigurableClient.__init__(self, "beliefs_filter", FILTER)

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
        changes_dict = self.filter(world_name, header, invalidations)
        for subject_id, changes in changes_dict.items():
            if len(changes.nodes_to_update) > 0:
                subject_name = self.worlds[world_name].scene.nodes[subject_id].name
                for mesh_id in invalidations.mesh_ids_updated:
                    changes.meshes_to_update.append(self.meshes[mesh_id])
                self.sendWorldChanges(subject_name+"_beliefs", header, changes)
                if len(changes.nodes_to_update) > 1:
                    rospy.loginfo("[%s::onChanges] Changes send (%d nodes) in world <%s>", self.node_name, len(changes.nodes_to_update), subject_name+"_beliefs")

    def filter(self, world_name, header, invalidations):
        """
        """
        changes = {}
        beliefs_map = {}

        #print len(invalidations.situation_ids_updated)
        for situation in self.worlds[world_name].timeline.situations.values():
            if situation.end.data == rospy.Time(0):
                if self.ctx.worlds()[world_name].timeline().by_property(situation.id, "predicate") == "isVisible":
                    subject_id = self.ctx.worlds()[world_name].timeline().by_property(situation.id, "subject")
                    object_id = self.ctx.worlds()[world_name].timeline().by_property(situation.id, "object")
                    if subject_id not in beliefs_map:
                        beliefs_map[subject_id] = {}
                    beliefs_map[subject_id][object_id] = self.ctx.worlds()[world_name].timeline().situations()[situation.id].confidence

        for subject_id in beliefs_map.keys():
            if subject_id not in changes:
                changes[subject_id] = Changes()
            changes[subject_id].nodes_to_update.append(self.worlds[world_name].scene.nodes[subject_id])
            if len(beliefs_map[subject_id]) > 0:
                print "camera <%s> see :" % self.worlds[world_name].scene.nodes[subject_id].name
                for object_id in beliefs_map[subject_id].keys():
                    print " - see object <%s> with %5f confidence" % (self.worlds[world_name].scene.nodes[object_id].name, beliefs_map[subject_id][object_id])
                    if object_id in invalidations.node_ids_updated:
                        changes[subject_id].nodes_to_update.append(self.worlds[world_name].scene.nodes[object_id])

        return changes


if __name__ == '__main__':
    rospy.init_node("beliefs_filter")
    bf = BeliefsFilter()
    rospy.spin()
