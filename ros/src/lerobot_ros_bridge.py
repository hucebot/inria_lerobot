#!/usr/bin/env python3

from __future__ import annotations
import rospy
import cv2
import numpy as np
import torch
from pathlib import Path
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Bool
from termcolor import colored
from lerobot.common.policies.act.modeling_act import ACTPolicy
import tf2_ros
import message_filters


def load_policy(model_path: str, device: str = "cuda") -> ACTPolicy:
    policy = ACTPolicy.from_pretrained(Path(model_path))
    policy.eval()
    print(colored("Model loaded", "green"))
    print(colored("Policy Input Features", "blue"), policy.config.input_features)
    print(colored("Policy Output Features", "blue"), policy.config.output_features)
    return policy


class ActPolicyNode:
    def __init__(self):
        rospy.init_node("act_policy_node")

        # Parameters
        self.device = rospy.get_param("~device", "cuda")
        self.model_repo = rospy.get_param("~model_repo", "/ros_ws/src/inria_lerobot/model/last/pretrained_model")
        self.img_topic = rospy.get_param("~img_topic", "/camera/color/image_raw")
        self.control_topic = rospy.get_param("~active_policy_topic", "/streamdeck/control_policy")
        self.base_frame = rospy.get_param("~base_frame", "ci/world")
        self.target_frame = rospy.get_param("~target_frame", "ci/gripper_right_grasping_frame")

        # Load policy
        self.policy = load_policy(self.model_repo, self.device)

        # Publishers
        self.pose_pub = rospy.Publisher("/dxl_input/pos_right", PoseStamped, queue_size=1)
        self.grip_pub = rospy.Publisher("/dxl_input/gripper_right", PointStamped, queue_size=1)

        # Control toggle
        self.control_active = False
        rospy.Subscriber(self.control_topic, Bool, self._active_policy_cb, queue_size=1)

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Image bridge
        self.bridge = CvBridge()

        # Image subscription only
        img_sub = message_filters.Subscriber(self.img_topic, Image)
        ats = message_filters.ApproximateTimeSynchronizer([img_sub], queue_size=10, slop=0.1)
        ats.registerCallback(self._sync_cb)

        # Publishing rate
        self.rate = rospy.Rate(20)
        self.last_action: np.ndarray | None = None

        rospy.loginfo(colored("ActPolicyNode initialized using tf transform for observation", "green"))

    def _active_policy_cb(self, msg: Bool) -> None:
        self.control_active = msg.data
        color = "green" if self.control_active else "red"
        print(colored(f"Control active: {self.control_active}", color))

    def _sync_cb(self, img_msg: Image) -> None:
        if not self.control_active:
            return

        # Convert image
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Image conversion failed: {e}")
            return
        img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (224, 224))
        img_tensor = (
            torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0).float() / 255.0
        ).to(self.device)

        # Use TF to get pose of target_frame in base_frame
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.target_frame,
                img_msg.header.stamp,
                rospy.Duration(0.1)
            )
            pos = tf_msg.transform.translation
            quat = tf_msg.transform.rotation
            pose_vec = torch.tensor([
                pos.x, pos.y, pos.z,
                quat.x, quat.y, quat.z, quat.w
            ], dtype=torch.float32).unsqueeze(0).to(self.device)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"TF transform failed: {e}")
            return

        # Build observation and get action
        obs = {
            "observation.images.cam_head_color": img_tensor,
            "observation.state": pose_vec,
        }
        with torch.no_grad():
            action = self.policy.select_action(obs)
        self.last_action = action.squeeze(0).cpu().numpy()

    def _publish_action(self, action_np: np.ndarray) -> None:
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = action_np[:3].tolist()
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = action_np[3:7].tolist()
        self.pose_pub.publish(pose)

        grip = PointStamped()
        grip.header.stamp = rospy.Time.now()
        grip.header.frame_id = self.base_frame
        grip.point.x = action_np[7] if action_np.size >= 8 else 0.0
        self.grip_pub.publish(grip)

        print(colored(f"Published action: {action_np}", "green" if self.control_active else "red", attrs=["bold"]))

    def run(self) -> None:
        rospy.loginfo(colored("Entering spin loop", "blue"))
        while not rospy.is_shutdown():
            if self.control_active and self.last_action is not None:
                self._publish_action(self.last_action)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        node = ActPolicyNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
