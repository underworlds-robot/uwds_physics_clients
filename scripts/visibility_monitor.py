import rospy
import pybullet as p
import pybullet_data
from pyuwds.reconfigurable_client import ReconfigurableClient
from uwds_msgs.msg import Changes
from uwds_msgs.srv import *
from pyuwds import *
import tf

class VisibilityMonitor(ReconfigurableClient):
    def __init__(self, ressource_folder, simulation_step, global_frame_id, min_treshold, input_world, output_world):
        """
        """
        ReconfigurableClient.__init__(self, "visibility_monitor", pyuwds.FILTER)
        p.connect(p.DIRECT) # Initialize bullet non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setAdditionalSearchPath(ressource_folder)

        self.input_world = input_world
        self.output_world = output_world

        self.node_id_map = {}
        self.reverse_node_id_map = {}
        bullet_id = p.loadURDF("plane.urdf")
        self.node_id_map[input_world.scene.rootID] = bullet_id
        self.reverse_node_id_map[bullet_id] = input_world.scene.rootID
        self.current_situation_map = {}
        self.min_treshold = min_treshold

    def onReconfigure(self, worlds):
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
        changes = self.monitor(invalidations)
        self.sendWorldChanges(world_name+"_stable", header, changes)

    def monitor(self, input_invalidations):
        """
        """
        changes = Changes()
        self.updateOutputNodes(input_invalidations.nodes_ids_updated)
        self.stepSimulation()

        for node in input_world.scene.nodes.items():
            node_id = node.id
            if node_id in self.node_id_map:
                if self.node_id_map[node_id] > 0:
                    t, q = p.getBasePositionAndOrientation(self.node_id_map[node_id])
                    node.position.pose.position = t
                    node.position.pose.orientation = q
                    changes.nodes_to_update.push_back(node)
        return changes

    def computeVisibilities(self, camera_id):
        visibilities = {}
        if not p.isNumpyEnabled():
            rospy.logwarn("Numpy not enabled, copy of image buffer can be slow")
        if camera_id in input_world.scene.nodes:
            camera_node = input_world.scene.nodes[camera_id]
            position = [camera_node.position.pose.position.x, camera_node.position.pose.position.y, camera_node.position.pose.position.z]
            euler = tf.transformations.euler_from_quaternion(camera_node.position.pose.orientation)
            view_matrix = p.computeViewMatrixFromYawPitchRoll(position, 4, euler[0], euler[1], euler[2], 2)
            fov = float(input_world.scene.getNodeProperty(camera_id, "hfov"))
            clipnear = float(input_world.scene.getNodeProperty(camera_id, "clipnear"))
            clipfar = float(input_world.scene.getNodeProperty(camera_id, "clipfar"))
            aspect = float(input_world.scene.getNodeProperty(camera_id, "aspect"))
            proj_matrix = p.computeProjectionMatrixFOV(fov, aspect, clipnear, clipfar)
            img = p.getCameraImage(self.width, self.height, flagsb = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
            seg = img[4]
            cv_image = pyopencv.Mat(height, width)
            for pixel in seg:
                if (pixel >= 0):
                    bullet_id = pixel & ((1 << 24)-1)
                    uwds_id = self.reverse_node_id_map[bullet_id]
                    cv_image[width * height+1] = pixel
                    if uwds_id not in visibilities:
                        visibilities[uwds_id] = 1
                    else:
                        visibilities[uwds_id] += 1
                for visibility_score in visibilities.items():
                    visibility_score / self.width * self.height
        return visibilities, cv_image

    def computeSituations(self, stamp, camera_id, visibilities):
        situations = []
        for id_seen, visibility_score in visibilities:
            if camera_id+id_seen not in self.current_situation_map:
                situation = Situation()
                situation.id = str(uuid.uuid4())
                situation.description = input_world.scene.nodes[id_seen].name + "is visible by" input_world.scene.nodes[camera_id].name
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
            else:
                situation = self.current_situation_map[camera_id+entity_seen]
                situation.confidence = visibility_score
                if situation.confidence < self.min_treshold:
                    situation.end = stamp
                    del self.current_situation_map[camera_id+entity_seen]
            situations.append(situation)
        return situations

    def updateBulletNodes(self, nodes_ids):
        """ This function load the urdf corresponding to the uwds node and set it in the environment
        The urdf need to have the same name than the node name
        :return:
        """
        for node_id in nodes_ids:
            if node_id != input_world.scene().rootID():
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
    rospy.init_node("visibility_monitor")
    gf = GravityFilter()
    rospy.spin()
