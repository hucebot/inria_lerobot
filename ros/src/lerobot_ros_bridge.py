#!/usr/bin/env python3

from __future__ import annotations
import time
from typing import Optional

import cv2
import numpy as np
import rospy
import torch
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Bool


def load_policy(model_repo: str, device: str = "cuda"):
    """Load a LeRobot policy from HF Hub or local path."""
    from lerobot.models import load_policy_from_hub

    policy = load_policy_from_hub(model_repo, device=device)
    policy.eval()
    return policy

class ActPolicyNode:
    def __init__(self):
        rospy.init_node("act_policy_node")

        # ---------------- Params ----------------
        self.device: str = rospy.get_param("~device", "cuda")
        self.model_repo: str = rospy.get_param("~model_repo", "lerobot/act_aloha_sim_transfer_cube_human")
        self.output_mode: str = rospy.get_param("~output_mode", "joint_state")
        self.dry_run: bool = rospy.get_param("~dry_run", False)
        self.control_active: bool = rospy.get_param("~control_active", True)
        self.img_topic: str = rospy.get_param("~img_topic", "/camera/color/image_raw")
        self.js_topic: str = rospy.get_param("~js_topic", "/joint_states")
        self.rate_hz: int = rospy.get_param("~rate_hz", 5)
        self.active_policy_topic: str = rospy.get_param("~active_policy_topic", "/streamdeck/control_policy")

        rospy.loginfo(f"[act_policy_node] Loading policy '{self.model_repo}' on {self.device} …")
        self.policy = load_policy(self.model_repo, self.device)

        # ---------------- State ------------------
        self.bridge = CvBridge()
        self.latest_img: Optional[np.ndarray] = None
        self.latest_js: Optional[JointState] = None

        # ---------------- Subs -------------------
        rospy.Subscriber(self.img_topic, Image, self._img_cb, queue_size=1, buff_size=2**24)
        rospy.Subscriber(self.js_topic, JointState, self._js_cb, queue_size=1)
        rospy.Subscriber(self.active_policy_topic, Bool, self._active_policy_cb, queue_size=1)

        # ---------------- Pubs -------------------
        if not self.dry_run:
            if self.output_mode == "joint_state":
                self.cmd_pub = rospy.Publisher(
                    "/so100/joint_group_position_controller/command", JointState, queue_size=1
                )
            elif self.output_mode == "dxl":
                self.pose_pub = rospy.Publisher("/dxl_input/pos_right", PoseStamped, queue_size=1)
                self.grip_pub = rospy.Publisher("/dxl_input/gripper_right", Float64, queue_size=1)
            else:
                raise ValueError("output_mode must be 'joint_state' or 'dxl'")

    # ---------------- Callbacks -----------------
    def _img_cb(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_img = img
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Image conversion failed: {e}")

    def _js_cb(self, msg: JointState):
        self.latest_js = msg

    def _active_policy_cb(self, msg: Bool):
        self.control_active = msg.data

    # ---------------- Loop ----------------------
    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if not self.control_active:
                rate.sleep()
                continue
            
            if self.latest_img is None or self.latest_js is None:
                rate.sleep()
                continue

            obs = {
                "rgb": torch.from_numpy(cv2.cvtColor(self.latest_img, cv2.COLOR_BGR2RGB))
                         .permute(2, 0, 1).unsqueeze(0).float() / 255.0,
                "state": torch.tensor(self.latest_js.position, dtype=torch.float32).unsqueeze(0),
            }
            obs = {k: v.to(self.device) for k, v in obs.items()}

            with torch.no_grad():
                action = self.policy(obs)

            action_np = action.squeeze(0).cpu().numpy()

            # ---------------------------------------------------------
            if self.dry_run:
                rospy.loginfo_throttle(1.0, f"Pred action: {action_np}")
            else:
                if self.output_mode == "joint_state":
                    self._publish_joint_state(action_np)
                else:
                    self._publish_dxl(action_np)

            rate.sleep()

    # ---------------- Utils ---------------------
    def _publish_joint_state(self, action_np):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = self.latest_js.name
        js.position = action_np.tolist()
        self.cmd_pub.publish(js)

    def _publish_dxl(self, action_np):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = action_np[:3].tolist()
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = action_np[3:7].tolist()
        grip = Float64()
        grip.data = float(action_np[7]) if action_np.size >= 8 else 0.0
        self.pose_pub.publish(pose)
        self.grip_pub.publish(grip)

# ---------------------------------------------------------------------
if __name__ == "__main__":
    try:
        node = ActPolicyNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
