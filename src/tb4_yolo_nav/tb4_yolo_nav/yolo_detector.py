"""Infer YOLO on live RGB camera messages converted with cv_bridge.

Only YOLO outputs become bounding boxes/classes/confidence; scene names and model
poses are never read. A bounded queue drops old frames so navigation stays live.
"""

import hashlib, json, os, queue, threading, time
from datetime import datetime, timezone
from pathlib import Path
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import Image
from std_msgs.msg import String


class YoloDetector(Node):
    def __init__(self):
        super().__init__(
            "yolo_detector", parameter_overrides=[Parameter("use_sim_time", value=True)]
        )
        root = Path(os.environ.get("TB4_WORKSPACE", "."))
        defaults = {
            "image_topic": "/oakd/rgb/preview/image_raw",
            "weights": str(root / "weights/yolov8n.pt"),
            "confidence": 0.35,
            "image_size": 320,
            "max_fps": 1.0,
            "display": True,
            "classes": [0, 10],
            "output_directory": str(root / "evidence/yolo"),
            "cpu_threads": 2,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.param = lambda k: self.get_parameter(k).value
        if not 0 < self.param("confidence") <= 1 or self.param("max_fps") <= 0:
            raise ValueError("Invalid inference parameters")
        self.output = Path(self.param("output_directory"))
        self.output.mkdir(parents=True, exist_ok=True)
        self.weights = Path(self.param("weights")).expanduser().resolve()
        if not self.weights.is_file():
            raise FileNotFoundError(
                "Run scripts/setup_yolo.sh first: " + str(self.weights)
            )
        import torch
        from ultralytics import YOLO

        torch.set_num_threads(self.param("cpu_threads"))
        cv2.setNumThreads(1)
        self.model = YOLO(str(self.weights))
        self.bridge = CvBridge()
        self.frames = queue.Queue(maxsize=1)
        self.stopping = threading.Event()
        self.gui_lock = threading.Lock()
        self.gui_frame = None
        self.gui_record = None
        self.latest_mission = {"state": "WAIT_MISSION", "completed": 0, "required": 3}
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, "/mission/status", self.on_mission, status_qos)
        self.count = 0
        self.received = 0
        self.dropped = 0
        self.errors = 0
        self.saved_classes = set()
        self.last_save = 0
        self.last_log = 0
        self.frame_ids = set()
        self.first_stamp = None
        self.last_stamp = None
        self.annotated = self.create_publisher(
            Image, "/yolo/annotated_image", qos_profile_sensor_data
        )
        self.detections = self.create_publisher(String, "/yolo/detections", 10)
        self.create_subscription(
            Image, self.param("image_topic"), self.on_image, qos_profile_sensor_data
        )
        self.stream = (self.output / "detections.jsonl").open("a", buffering=1)
        self.thread = threading.Thread(target=self.worker, daemon=True)
        self.thread.start()
        if self.param("display"):
            cv2.namedWindow("TurtleBot 4 - YOLO camera", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("TurtleBot 4 - YOLO camera", 600, 850)
            self.gui_timer = self.create_timer(
                0.1, self.update_gui, clock=Clock(clock_type=ClockType.STEADY_TIME)
            )
        self.get_logger().info(
            f'YOLO CPU ready; input={self.param("image_topic")} weights={self.weights.name}'
        )

    def on_image(self, msg):
        self.received += 1
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8").copy()
        except Exception as e:
            self.errors += 1
            self.get_logger().error("cv_bridge: " + str(e))
            return
        try:
            self.frames.put_nowait((image, msg.header))
        except queue.Full:
            try:
                self.frames.get_nowait()
                self.dropped += 1
            except queue.Empty:
                pass
            try:
                self.frames.put_nowait((image, msg.header))
            except queue.Full:
                self.dropped += 1

    def worker(self):
        while not self.stopping.is_set():
            try:
                image, header = self.frames.get(timeout=0.2)
            except queue.Empty:
                continue
            began = time.monotonic()
            try:
                result = self.model.predict(
                    source=image,
                    imgsz=self.param("image_size"),
                    conf=self.param("confidence"),
                    classes=self.param("classes"),
                    device="cpu",
                    verbose=False,
                )[0]
                # Ultralytics selects its own thread count on the first call; restore the configured bound.
                import torch

                torch.set_num_threads(self.param("cpu_threads"))
                detections = []
                if result.boxes is not None:
                    for box in result.boxes:
                        index = int(box.cls.item())
                        detections.append(
                            {
                                "class_id": index,
                                "class_name": result.names[index],
                                "confidence": float(box.conf.item()),
                                "xyxy": [float(x) for x in box.xyxy[0].tolist()],
                            }
                        )
                annotated = result.plot()
                out = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                out.header = header
                self.annotated.publish(out)
                stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
                record = {
                    "utc": datetime.now(timezone.utc).isoformat(),
                    "image_stamp": stamp,
                    "frame_id": header.frame_id,
                    "source_topic": self.param("image_topic"),
                    "source_shape": list(image.shape),
                    "source_sha256": hashlib.sha256(image.tobytes()).hexdigest(),
                    "inference_wall_ms": round((time.monotonic() - began) * 1000, 2),
                    "detections": detections,
                }
                msg = String()
                msg.data = json.dumps(record)
                self.detections.publish(msg)
                self.stream.write(msg.data + "\n")
                self.count += 1
                self.frame_ids.add(header.frame_id)
                if self.first_stamp is None:
                    self.first_stamp = stamp
                self.last_stamp = stamp
                names = {d["class_name"] for d in detections}
                if detections and (
                    names - self.saved_classes or time.monotonic() - self.last_save > 20
                ):
                    stem = f"frame_{self.count:06d}"
                    cv2.imwrite(str(self.output / (stem + "_raw.png")), image)
                    cv2.imwrite(str(self.output / (stem + "_annotated.png")), annotated)
                    (self.output / (stem + ".json")).write_text(
                        json.dumps(record, indent=2) + "\n"
                    )
                    self.saved_classes.update(names)
                    self.last_save = time.monotonic()
                if time.monotonic() - self.last_log > 2:
                    labels = (
                        ", ".join(
                            f'{d["class_name"]} {d["confidence"]:.2f}'
                            for d in detections
                        )
                        or "No detections"
                    )
                    self.get_logger().info(
                        f'camera t={stamp:.2f}s: {labels}; inference={record["inference_wall_ms"]:.0f}ms'
                    )
                    self.last_log = time.monotonic()
                    self.write_summary()
                with self.gui_lock:
                    self.gui_frame = annotated.copy()
                    self.gui_record = record
            except Exception as e:
                self.errors += 1
                self.get_logger().error("Inference/publish failed: " + str(e))
            self.stopping.wait(
                max(0, 1.0 / self.param("max_fps") - (time.monotonic() - began))
            )

    def on_mission(self, msg):
        try:
            self.latest_mission = json.loads(msg.data)
        except (ValueError, TypeError):
            pass

    def update_gui(self):
        import numpy as np

        with self.gui_lock:
            frame = self.gui_frame
            record = self.gui_record
        if frame is not None:
            canvas = np.full((850, 600, 3), 22, dtype=np.uint8)
            canvas[:450] = cv2.resize(frame, (600, 450), interpolation=cv2.INTER_LINEAR)
            mission = self.latest_mission
            lines = [
                ("LIVE CAMERA / YOLOv8n", (120, 220, 250)),
                ("AUTONOMOUS NAVIGATION", (250, 220, 120)),
                (
                    "State: " + str(mission.get("state", "WAIT_MISSION")),
                    (220, 240, 220),
                ),
                (
                    f"Completed: {mission.get('completed',0)} / {mission.get('required',3)} goals",
                    (220, 240, 220),
                ),
            ]
            target = mission.get("target")
            if target:
                lines.append(
                    (
                        f"Target: ({target['x']:.1f}, {target['y']:.1f}), yaw {target['yaw']:.2f}",
                        (220, 240, 220),
                    )
                )
            feedback = mission.get("feedback") or {}
            if "distance_remaining" in feedback:
                lines.append(
                    (
                        f"Distance remaining: {feedback['distance_remaining']:.2f} m",
                        (220, 240, 220),
                    )
                )
            labels = (
                ", ".join(
                    d["class_name"] + f" {d['confidence']:.2f}"
                    for d in (record or {}).get("detections", [])
                )
                or "No detections"
            )
            lines.extend(
                [
                    ("Detected: " + labels, (180, 240, 180)),
                    (
                        f"Camera time: {(record or {}).get('image_stamp',0):.2f} s",
                        (180, 190, 200),
                    ),
                    ("Goals are sent by the ROS node.", (180, 190, 200)),
                ]
            )
            for i, (text, color) in enumerate(lines):
                cv2.putText(
                    canvas,
                    text,
                    (18, 480 + i * 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    1,
                    cv2.LINE_AA,
                )
            cv2.imshow("TurtleBot 4 - YOLO camera", canvas)
        cv2.waitKey(1)

    def write_summary(self):
        data = {
            "received_images": self.received,
            "inferred_images": self.count,
            "dropped_old_images": self.dropped,
            "errors": self.errors,
            "classes_detected": sorted(self.saved_classes),
            "input_topic": self.param("image_topic"),
            "frame_ids": sorted(self.frame_ids),
            "first_image_stamp": self.first_stamp,
            "last_image_stamp": self.last_stamp,
            "weights": self.weights.name,
            "weights_sha256": hashlib.sha256(self.weights.read_bytes()).hexdigest(),
            "backend": "Ultralytics YOLOv8n / PyTorch CPU",
            "confidence_threshold": self.param("confidence"),
        }
        tmp = self.output / "summary.json.tmp"
        tmp.write_text(json.dumps(data, indent=2) + "\n")
        tmp.replace(self.output / "summary.json")

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=20)
        self.write_summary()
        self.stream.close()
        if self.param("display"):
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
