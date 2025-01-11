from typing import Tuple

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from susumu_ros2_interfaces.msg import LED

from susumu_blinkstick_ros2.animation_manager import AnimationManager
from susumu_blinkstick_ros2.animations import (
    SolidAnimation,
    BlinkAnimation,
    TwoColorFadeAnimation,
    LeftToRightAnimation,
    RightToLeftAnimation,
    WaveAnimation,
    InwardAnimation,
    OutwardAnimation, StopAnimation
)


class BlinkstickNode(Node):
    """
    ROS2ノードとして起動し、
    /led トピック (LED型) を購読してアニメーションを追加。
    """

    def __init__(self):
        super().__init__('blinkstick_node')

        self.declare_parameter('led_count', 8)
        led_count_val = self.get_parameter('led_count').value

        self.declare_parameter('brightness', 0.5)
        self.brightness = self.get_parameter('brightness').value

        self.get_logger().info(f"BlinkstickNode: led_count parameter = {led_count_val}")
        self.get_logger().info(f"BlinkstickNode: brightness parameter = {self.brightness}")

        self.add_on_set_parameters_callback(self.parameter_callback)

        # 4) AnimationManager を初期化
        self.manager = AnimationManager(led_count=led_count_val, frame_interval=0.05)

        self.subscription = self.create_subscription(
            LED,  # 生成されたmsg型
            '/led',
            self.led_callback,
            10
        )
        self.get_logger().info("BlinkstickNode started, subscribing to /led")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'brightness' and param.type_ == rclpy.parameter.Parameter.Type.DOUBLE:
                new_brightness = param.value
                if 0.0 <= new_brightness <= 1.0:
                    self.brightness = new_brightness
                    self.get_logger().info(f"Brightness updated to: {self.brightness}")
                else:
                    self.get_logger().warn("Brightness must be between 0.0 and 1.0")
                    return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def led_callback(self, msg: LED):
        """
        /ledトピックからのメッセージを受け取り、アニメーションを追加。
        """
        self.get_logger().info(f"Received: pattern={msg.pattern}, priority={msg.priority}")

        pat = msg.pattern.upper()
        prio = msg.priority
        dur = msg.duration
        c1_str = msg.color1.strip().lower()
        c2_str = msg.color2.strip().lower()
        dec = msg.decay_rate
        spd = msg.speed

        # brightness はメッセージからではなく、ノードのパラメータから取得
        br = self.brightness

        # 文字列からRGBタプルへの変換
        try:
            c1 = self.parse_color(c1_str)
            c2 = self.parse_color(c2_str)
        except ValueError as e:
            self.get_logger().error(f"Color parsing error: {e}")
            self.get_logger().warn("Falling back to default colors.")
            c1 = (255, 255, 255)
            c2 = (255, 255, 255)

        # パターンに応じて生成
        if pat == "STOP":
            anim = StopAnimation(prio, dur, c1, br, dec, spd)
        elif pat == "BLINK":
            anim = BlinkAnimation(prio, dur, c1, br, dec, spd)
        elif pat == "SOLID":
            anim = SolidAnimation(prio, dur, c1, br, dec, spd)
        elif pat == "TWO_COLOR_FADE":
            anim = TwoColorFadeAnimation(prio, dur, c1, c2, br, dec, spd)
        elif pat == "LEFT_TO_RIGHT":
            anim = LeftToRightAnimation(prio, dur, c1, br, dec, spd)
        elif pat == "RIGHT_TO_LEFT":
            anim = RightToLeftAnimation(prio, dur, c1, br, dec, spd)
        elif pat == "WAVE":
            anim = WaveAnimation(prio, dur, c1, br, dec, spd)
        elif pat == "INWARD":
            anim = InwardAnimation(prio, dur, c1, br, dec, spd)
        elif pat == "OUTWARD":
            anim = OutwardAnimation(prio, dur, c1, br, dec, spd)
        else:
            self.get_logger().warn(f"Unknown pattern: {pat}, fallback to SOLID")
            anim = SolidAnimation(prio, dur, c1, br, dec, spd)

        self.manager.add_animation(anim)

    def parse_color(self, color_str: str) -> Tuple[int, int, int]:
        """
        色の文字列をRGBタプルに変換する。
        サポート:
            - 基本的な色名 (red, green, blue, etc.)
            - 16進数RGBコード (FF0000, #FF0000, etc.)
        """
        # 基本色名のマッピング
        color_names = {
            'black': (0, 0, 0),
            'white': (255, 255, 255),
            'red': (255, 0, 0),
            'green': (0, 255, 0),
            'blue': (0, 0, 255),
            'yellow': (255, 255, 0),
            'cyan': (0, 255, 255),
            'magenta': (255, 0, 255),
            'orange': (255, 165, 0),
            'purple': (128, 0, 128),
            'pink': (255, 192, 203),
            'gray': (128, 128, 128),
            'grey': (128, 128, 128),
            # 必要に応じて追加
        }

        if color_str in color_names:
            return color_names[color_str]
        else:
            # 16進数RGBコードの解析
            hex_str = color_str.lstrip('#')
            if len(hex_str) != 6:
                raise ValueError(f"Invalid color format: {color_str}")
            try:
                r = int(hex_str[0:2], 16)
                g = int(hex_str[2:4], 16)
                b = int(hex_str[4:6], 16)
                return (r, g, b)
            except ValueError:
                raise ValueError(f"Invalid hex color code: {color_str}")

    def destroy_node(self) -> bool:
        self.get_logger().info("Shutting down BlinkstickNode")
        self.manager.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BlinkstickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
