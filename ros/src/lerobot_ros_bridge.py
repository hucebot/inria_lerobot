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
        self.model_repo = rospy.get_param(
            "~model_repo",
            "/ros_ws/src/inria_lerobot/model/last/pretrained_model",
        )
        # Dual-camera topics
        self.img_topic_head = rospy.get_param("~img_topic_head", "/camera/head/color/image_raw")
        self.img_topic_side = rospy.get_param("~img_topic_side", "/camera/side/color/image_raw")
        self.control_topic = rospy.get_param("~active_policy_topic", "/streamdeck/control_policy")
        self.base_frame = rospy.get_param("~base_frame", "ci/world")
        self.target_frame = rospy.get_param("~target_frame", "ci/gripper_right_grasping_frame")

        # Load policy
        self.policy = load_policy(self.model_repo, self.device)

        # Publishers
        self.pose_pub = rospy.Publisher(
            "/dxl_input/pos_right", PoseStamped, queue_size=1
        )
        self.grip_pub = rospy.Publisher(
            "/dxl_input/gripper_right", PointStamped, queue_size=1
        )

        # Control toggle
        self.control_active = False
        rospy.Subscriber(
            self.control_topic, Bool, self._active_policy_cb, queue_size=1
        )

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Image bridge
        self.bridge = CvBridge()

        # Subscribe to both cameras and synchronize
        head_sub = message_filters.Subscriber(self.img_topic_head, Image)
        side_sub = message_filters.Subscriber(self.img_topic_side, Image)
        ats = message_filters.ApproximateTimeSynchronizer(
            [head_sub, side_sub], queue_size=10, slop=0.1
        )
        ats.registerCallback(self._sync_cb)

        # Publishing rate
        self.rate = rospy.Rate(5)
        self.last_action: np.ndarray | None = None

        rospy.loginfo(
            colored(
                "ActPolicyNode initialized with dual-camera input and TF-based observation",
                "green",
            )
        )

    def _active_policy_cb(self, msg: Bool) -> None:
        self.control_active = msg.data
        color = "green" if self.control_active else "red"
        print(colored(f"Control active: {self.control_active}", color))

    def _sync_cb(self, head_img_msg: Image, side_img_msg: Image) -> None:
        if not self.control_active:
            return

        # Convert and preprocess head image
        try:
            cv_img_head = self.bridge.imgmsg_to_cv2(head_img_msg, desired_encoding="bgr8")
            head_rgb = cv2.cvtColor(cv_img_head, cv2.COLOR_BGR2RGB)
            head_resized = cv2.resize(head_rgb, (224, 224))
            head_tensor = (
                torch.from_numpy(head_resized)
                .permute(2, 0, 1)
                .unsqueeze(0)
                .float()
                / 255.0
            ).to(self.device)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Head image failed: {e}")
            return

        # Convert and preprocess side image
        try:
            cv_img_side = self.bridge.imgmsg_to_cv2(side_img_msg, desired_encoding="bgr8")
            side_rgb = cv2.cvtColor(cv_img_side, cv2.COLOR_BGR2RGB)
            side_resized = cv2.resize(side_rgb, (224, 224))
            side_tensor = (
                torch.from_numpy(side_resized)
                .permute(2, 0, 1)
                .unsqueeze(0)
                .float()
                / 255.0
            ).to(self.device)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Side image failed: {e}")
            return

        # Get TF pose
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.target_frame,
                head_img_msg.header.stamp,
                rospy.Duration(0.1),
            )
            t = tf_msg.transform.translation
            q = tf_msg.transform.rotation
            pose_vec = torch.tensor([
                t.x, t.y, t.z,
                q.x, q.y, q.z, q.w
            ], dtype=torch.float32).unsqueeze(0).to(self.device)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"TF failed: {e}")
            return

        # Build observation and infer action
        obs = {
            "observation.images.cam_head_color": head_tensor,
            "observation.images.cam_side_color": side_tensor,
            "observation.state": pose_vec,
        }
        with torch.no_grad():
            action = self.policy.select_action(obs)
        self.last_action = action.squeeze(0).cpu().numpy()

    def _publish_action(self, action_np: np.ndarray) -> None:
        # Publish pose
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = action_np[:3].tolist()
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = action_np[3:7].tolist()
        self.pose_pub.publish(pose)

        # Publish gripper
        grip = PointStamped()
        grip.header.stamp = rospy.Time.now()
        grip.header.frame_id = self.base_frame
        grip.point.x = action_np[7] if action_np.size >= 8 else 0.0
        self.grip_pub.publish(grip)

        print(
            colored(f"Published action: {action_np}", "green" if self.control_active else "red", attrs=["bold"])
        )

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
