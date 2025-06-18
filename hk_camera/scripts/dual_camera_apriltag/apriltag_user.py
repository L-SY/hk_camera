#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import json
import os

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class JsonPolygonCropNode:
    def __init__(self):
        rospy.init_node("json_polygon_crop_node")

        # —— 参数 ——
        self.image_topic    = rospy.get_param("~image_topic",       "/stitched_image")
        self.json_path      = rospy.get_param("~polygon_json_path", "apriltag_polygon.json")
        self.output_topic   = rospy.get_param("~output_topic",     "/panel_detector/foam_board/image_raw")

        self.bridge = CvBridge()
        self.pub    = rospy.Publisher(self.output_topic, Image, queue_size=1)

        # 缓存 JSON 文件修改时间及顶点
        self._last_mtime = None
        self._points     = None

        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)

        rospy.loginfo(f"[json_crop] Subscribed to '{self.image_topic}'")
        rospy.loginfo(f"[json_crop] JSON path: {self.json_path}")
        rospy.loginfo(f"[json_crop] Publishing to '{self.output_topic}'")
        rospy.spin()


    def _load_json(self):
        """按需从 JSON 加载多边形顶点缓存"""
        try:
            mtime = os.path.getmtime(self.json_path)
        except OSError:
            rospy.logwarn_throttle(10, f"[json_crop] JSON 文件不存在：{self.json_path}")
            return False

        if self._last_mtime == mtime and self._points is not None:
            return True

        try:
            with open(self.json_path, "r") as f:
                data = json.load(f)
            pts = data.get("points", [])
            if not isinstance(pts, list) or len(pts) < 3:
                raise ValueError("points 列表格式错误或点数不足")
            # 转成 N×1×2 并缓存
            arr = np.array(pts, dtype=np.int32).reshape(-1,1,2)
            self._points = arr
            self._last_mtime = mtime
            rospy.loginfo_throttle(5, f"[json_crop] Loaded {len(arr)} points")
            return True

        except Exception as e:
            rospy.logwarn_throttle(10, f"[json_crop] 读取 JSON 失败: {e}")
            return False


    def _order_points(self, pts):
        """
        将任意 4×2 的点集按 tl,tr,br,bl 顺序返回
        参考：https://www.pyimagesearch.com/2014/08/25/4-point-opencv-getperspective-transform-example/
        """
        rect = np.zeros((4,2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]  # top-left
        rect[2] = pts[np.argmax(s)]  # bottom-right

        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]  # top-right
        rect[3] = pts[np.argmax(diff)]  # bottom-left

        return rect


    def image_callback(self, img_msg: Image):
        # 1) 加载或更新顶点
        if not self._load_json():
            return

        # 2) 解码图像
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"[json_crop] 解码图像失败: {e}")
            return

        # 3) 准备源点：如果正好 4 点用它，否则用 minAreaRect
        pts = self._points.reshape(-1,2).astype("float32")
        if pts.shape[0] == 4:
            src_pts = pts
        else:
            rect = cv2.minAreaRect(pts)
            box  = cv2.boxPoints(rect)
            src_pts = np.array(box, dtype="float32")

        # 4) 按 tl,tr,br,bl 排序，并计算目标尺寸
        ordered = self._order_points(src_pts)
        (tl, tr, br, bl) = ordered

        widthA  = np.linalg.norm(br - bl)
        widthB  = np.linalg.norm(tr - tl)
        maxWidth  = int(max(widthA, widthB))

        heightA = np.linalg.norm(tr - br)
        heightB = np.linalg.norm(tl - bl)
        maxHeight = int(max(heightA, heightB))

        dst = np.array([
            [0, 0],
            [maxWidth-1, 0],
            [maxWidth-1, maxHeight-1],
            [0, maxHeight-1]
        ], dtype="float32")

        # 5) 透视变换 & 裁切
        M = cv2.getPerspectiveTransform(ordered, dst)
        warped = cv2.warpPerspective(img, M, (maxWidth, maxHeight))

        # 6) 发布结果
        out_msg = self.bridge.cv2_to_imgmsg(warped, encoding="bgr8")
        out_msg.header = img_msg.header
        self.pub.publish(out_msg)


if __name__ == "__main__":
    try:
        JsonPolygonCropNode()
    except rospy.ROSInterruptException:
        pass
