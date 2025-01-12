import os
import numpy as np
# ros2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, String
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from openai import OpenAI


class ChatGPT(Node):
    # ノード名
    SELFNODE = "chatgpt"

    def __init__(self):
        # ノードの初期化
        super().__init__(self.SELFNODE)
        self.get_logger().info("%s initializing..." % (self.SELFNODE))
        # OpenAI
        self.client = OpenAI()
        # ros2 init
        self.text_pub_ = self.create_publisher(
            String, 'chatgpt/output_text', 1)
        self.text_sub_ = self.create_subscription(
            String, 'chatgpt/input_text', self.text_callback, qos_profile=ReliabilityPolicy.RELIABLE)

    def gpt_response(self, text):
        completion = self.client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": "You are a helpful assistant."},
                {
                    "role": "user",
                    "content": text
                }
            ]
        )
        return completion.choices[0].message

    def __del__(self):
        self.get_logger().info("%s done." % self.SELFNODE)

    def param(self, name, value):
        self.declare_parameter(name, value)
        return self.get_parameter(name).get_parameter_value()

    def text_callback(self, msg):
        print("<< ", msg.data)
        response = self.gpt_response(msg.data)
        print(">> \n\r", response.content)
        self.text_pub_.publish(String(data=response.content))
        pass


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ChatGPT()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了処理
        rclpy.shutdown()


if __name__ == '__main__':
    main()
