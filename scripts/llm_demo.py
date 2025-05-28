import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import numpy as np
import time
import os
import base64
import math
from math import cos, sin

from openai import AzureOpenAI
from PIL import Image as PILImage
from cv_bridge import CvBridge

# ========== Azure OpenAI Settings ==========
AZURE_OPENAI_ENDPOINT = "https://gpt-4-turbo-with-vision-kubo.openai.azure.com/"
AZURE_OPENAI_MODEL_NAME = "gpt-4o"
AZURE_OPENAI_DEPLOYMENT = "gpt-4o-image-to-text"
AZURE_OPENAI_KEY = os.getenv("OPENAI_API_KEY")
AZURE_OPENAI_API_VERSION = "2024-12-01-preview"

client = AzureOpenAI(
    api_version=AZURE_OPENAI_API_VERSION,
    azure_endpoint=AZURE_OPENAI_ENDPOINT,
    api_key=AZURE_OPENAI_KEY,
)

def compress_image(input_path, output_path, max_size=(800, 800), quality=80):
    with PILImage.open(input_path) as img:
        img.thumbnail(max_size)
        img.save(output_path, format="JPEG", quality=quality)

def llm_cost_estimate(image_msg, prompt):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
    temp_path = "/tmp/current_cam.jpg"
    PILImage.fromarray(cv_image).save(temp_path)
    compressed_path = "/tmp/current_cam_compressed.jpg"
    compress_image(temp_path, compressed_path)
    with open(compressed_path, "rb") as f:
        b64_image = base64.b64encode(f.read()).decode("utf-8")
    try:
        # 1st: Get visual description in English
        description_prompt = "Describe in English what you see in this image."
        response = client.chat.completions.create(
            model=AZURE_OPENAI_DEPLOYMENT,
            messages=[
                {
                    "role": "system",
                    "content": "You are a helpful assistant.",
                },
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": description_prompt},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{b64_image}"
                            }
                        }
                    ]
                }
            ],
            max_tokens=64,
            temperature=0.0,
            top_p=1.0
        )
        desc = response.choices[0].message.content.strip()
        print(f"[LLM visual description] {desc}")

        # 2nd: Judge safety based on description
        judge_prompt = (
            f"Based on the following situation description, judge if it is safe to proceed. "
            f"Reply only with 'safe' or 'unsafe'.\n\nDescription: {desc}"
        )
        judge_response = client.chat.completions.create(
            model=AZURE_OPENAI_DEPLOYMENT,
            messages=[
                {
                    "role": "system",
                    "content": "You are a helpful assistant.",
                },
                {
                    "role": "user",
                    "content": judge_prompt,
                }
            ],
            max_tokens=8,
            temperature=0.0,
            top_p=1.0
        )
        judge = judge_response.choices[0].message.content.strip().lower()
        print(f"[LLM judge] {judge}")

        if "safe" in judge and not "unsafe" in judge:
            return 1
        else:
            return 100
    except Exception as e:
        print(f"LLM error: {e}")
        return 100

def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

def yaw_to_quaternion(yaw_deg):
    # ROS uses xyzw order
    q = quaternion_from_euler(0, 0, np.deg2rad(yaw_deg))
    return q  # [x, y, z, w]

class AStarLLMNode(Node):
    def __init__(self, image_saver):
        super().__init__('astar_llm_node')
        self.teleport_pub = self.create_publisher(PoseStamped, '/OmniLRS/Robots/Teleport', 10)
        self.spawn_pub = self.create_publisher(PoseStamped, '/OmniLRS/Robots/Spawn', 10)
        self.image_sub = self.create_subscription(
            Image, '/left_rgb/rgb', self.camera_cb, 10)
        self.bridge = CvBridge()
        self.current_pose = (5.0, 5.0)
        self.current_yaw = 0.0  # degrees
        self.goal_pose = (1.0, 1.0)
        self.visited = set()
        self.images = {}
        self.last_pose = None
        self.move_wait_time = 10.0  # seconds for waiting after teleport for image to arrive
        self.timer = self.create_timer(1.0, self.astar_step)
        self.teleport_robot(*self.current_pose, self.current_yaw)
        self.get_logger().info('AStar LLM Node started.')
        self.image_ready = False
        self.latest_image = None
        self.image_saver = image_saver  # ImageSaverインスタンス参照

    def teleport_robot(self, x, y, yaw_deg):
        msg = PoseStamped()
        msg.header.frame_id = '/jackal'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 1.0
        q = yaw_to_quaternion(yaw_deg)
        msg.pose.orientation.x = float(q[0])
        msg.pose.orientation.y = float(q[1])
        msg.pose.orientation.z = float(q[2])
        msg.pose.orientation.w = float(q[3])
        self.teleport_pub.publish(msg)
        self.get_logger().info(f"Teleported robot to: ({x}, {y}), yaw={yaw_deg}")

    def spawn_robot(self, x, y, yaw_deg):
        msg = PoseStamped()
        msg.header.frame_id = 'jackal:/workspace/omnilrs/assets/USD_Assets/robots/ros2_jackal_PhysX_vlp16.usd_stereo.usd'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 1.0
        q = yaw_to_quaternion(yaw_deg)
        msg.pose.orientation.x = float(q[0])
        msg.pose.orientation.y = float(q[1])
        msg.pose.orientation.z = float(q[2])
        msg.pose.orientation.w = float(q[3])
        self.spawn_pub.publish(msg)
        self.get_logger().info(f"Spawned robot at: ({x}, {y}), yaw={yaw_deg}")

    def camera_cb(self, msg):
        self.latest_image = msg
        self.image_ready = True

    def get_image_for_orientation(self, x, y, yaw_deg):
        self.teleport_robot(x, y, yaw_deg)
        teleport_time = time.time()
        wait_time = self.move_wait_time
        time.sleep(wait_time)  # テレポート後に一定時間待つ
        # ImageSaverのバッファからteleport_time+wait_time以降の画像を取得
        target_time = teleport_time + wait_time
        waited = 0
        found = False
        ts = None
        msg = None
        while waited < wait_time * 5:
            ts, msg = self.image_saver.get_image_after(target_time)
            if msg is not None:
                found = True
                break
            time.sleep(0.2)
            waited += 0.2
        if found:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            save_path = f"/workspace/omnilrs/tmp/left_rgb_{x:.2f}_{y:.2f}_{yaw_deg:.0f}_{int(ts)}.jpg"
            pil_image = PILImage.fromarray(cv_image)
            pil_image.save(save_path)
            self.get_logger().info(f'Saved image: {save_path}')
            return msg
        else:
            self.get_logger().warning("No image received after teleport/orient!")
            return None

    def astar_step(self):
        current = self.current_pose
        if current == self.goal_pose:
            self.get_logger().info('Goal reached!')
            self.destroy_node()
            return

        # 4 directions: [yaw: 0, 90, 180, 270]
        directions = [
            (0.1, 0,   0),   # east (yaw 0)
            (0, 0.1,  90),   # north (yaw 90)
            (-0.1, 0, 180),  # west (yaw 180)
            (0, -0.1, 270),  # south (yaw 270)
        ]
        move_names = ['east', 'north', 'west', 'south']
        neighbors = [
            (current[0] + dx, current[1] + dy, yaw)
            for dx, dy, yaw in directions
        ]
        costs = []
        self.get_logger().info(f"[A*] Current pose: {self.current_pose}, yaw: {self.current_yaw}")
        for i, (nx, ny, yaw) in enumerate(neighbors):
            self.get_logger().info(f"[A*] Checking direction {move_names[i]}: ({nx}, {ny}), yaw={yaw}")
            if (nx, ny) in self.visited:
                self.get_logger().info(f"[A*] Already visited: ({nx}, {ny})")
                costs.append((float('inf'), (nx, ny, yaw)))
                continue
            prompt = f"You are controlling a moon rover. Analyze and describe the situation in the image, and determine if it is safe to proceed {move_names[i]}. Respond only with a single word: 'safe' or 'unsafe'."
            image = self.get_image_for_orientation(current[0], current[1], yaw)
            if image is None:
                self.get_logger().warning(f"[A*] No image for direction {move_names[i]} ({nx}, {ny}, yaw={yaw})")
                costs.append((float('inf'), (nx, ny, yaw)))
                continue
            self.get_logger().info(f"[A*] Got image for direction {move_names[i]} ({nx}, {ny}, yaw={yaw})")
            cost = llm_cost_estimate(image, prompt)
            self.get_logger().info(f"[A*] LLM cost for direction {move_names[i]} (yaw={yaw}): {cost}")
            costs.append((cost, (nx, ny, yaw)))

        # Choose min cost, not visited
        min_cost, (next_x, next_y, next_yaw) = min(costs, key=lambda x: x[0])
        self.get_logger().info(f"[A*] All costs: {costs}")
        if min_cost == float('inf'):
            self.get_logger().info("[A*] No available moves! Stopping.")
            self.destroy_node()
            return

        self.get_logger().info(f"[A*] Next move: ({next_x}, {next_y}) yaw={next_yaw} with cost {min_cost}")
        self.teleport_robot(next_x, next_y, next_yaw)
        self.get_logger().info(f"[A*] Teleported to: ({next_x}, {next_y}), yaw={next_yaw}")
        self.current_pose = (next_x, next_y)
        self.current_yaw = next_yaw
        self.visited.add((next_x, next_y))
        self.latest_image = None
        # LLM出力後すぐに次のテレポート（タイマーで自動呼び出しされるため、ここでreturnしない）
        self.get_logger().info(f"[A*] Step complete. Will check again on next timer tick.")

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/left_rgb/rgb',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.count = 0
        self.image_buffer = []  # (timestamp, msg)

    def listener_callback(self, msg):
        now = time.time()
        self.image_buffer.append((now, msg))
        # バッファが大きくなりすぎないように最新20件だけ保持
        if len(self.image_buffer) > 20:
            self.image_buffer = self.image_buffer[-20:]
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        filename = f'/workspace/omnilrs/tmp/left_rgb_{self.count:04d}.jpg'
        #pil_image = PILImage.fromarray(cv_image)
        #pil_image.save(filename)
        #self.get_logger().info(f'Saved image: {filename}')
        self.count += 1

    def get_image_after(self, target_time):
        # target_time以降で最初の画像を返す
        for ts, msg in self.image_buffer:
            if ts >= target_time:
                return ts, msg
        return None, None

def main(args=None):
    rclpy.init(args=args)
    image_saver_node = ImageSaver()
    astar_node = AStarLLMNode(image_saver_node)
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(astar_node)
    executor.add_node(image_saver_node)
    try:
        executor.spin()
    finally:
        astar_node.destroy_node()
        image_saver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()