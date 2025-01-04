import time
import threading
import queue
from typing import Optional, List, Tuple
from blinkstick import blinkstick

from susumu_blinkstick_ros2.animations import BaseAnimation


class LEDController:
    """
    BlinkStickによるLED制御。
    """
    def __init__(self, led_count: int = 8) -> None:
        self.led_count: int = led_count
        self.colors: List[Tuple[int, int, int]] = [(0, 0, 0)] * led_count
        self.bstick = blinkstick.find_first()
        if not self.bstick:
            raise Exception("BlinkStick not found")

    def apply_trail(self, decay_rate: float) -> None:
        new_colors = []
        for (r, g, b) in self.colors:
            nr = int(r * decay_rate)
            ng = int(g * decay_rate)
            nb = int(b * decay_rate)
            new_colors.append((nr, ng, nb))
        self.colors = new_colors

    def set_led_color(self, index: int, color: Tuple[int, int, int]) -> None:
        if 0 <= index < self.led_count:
            self.colors[index] = color

    def set_all_color(self, color: Tuple[int, int, int]) -> None:
        self.colors = [color] * self.led_count

    def update_hardware(self) -> None:
        for i, (r, g, b) in enumerate(self.colors):
            self.bstick.set_color(channel=0, index=i, red=r, green=g, blue=b)

    def turn_off(self) -> None:
        self.colors = [(0, 0, 0)] * self.led_count
        self.update_hardware()


class AnimationManager:
    """
    フレームループで current_animation を実行。
    新たなアニメを add するときに優先度が高ければ乗り換え。
    """
    def __init__(self, led_count: int = 8, frame_interval: float = 0.05) -> None:
        self.led_controller = LEDController(led_count)
        self.animation_queue = queue.Queue()
        self.frame_interval = frame_interval
        self.running = True
        self.current_animation: Optional[BaseAnimation] = None
        self.start_time: float = 0.0

        self.thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.thread.start()

    def add_animation(self, anim: BaseAnimation) -> None:
        self.animation_queue.put(anim)

    def stop(self) -> None:
        self.running = False
        self.animation_queue.put(None)
        self.thread.join()
        self.led_controller.turn_off()

    def _worker_loop(self) -> None:
        last_time = time.time()

        while self.running:
            try:
                new_anim = self.animation_queue.get(timeout=0.05)
                if new_anim is None:
                    continue
                # 優先度チェック
                if (self.current_animation is None
                    or new_anim.priority > self.current_animation.priority):
                    self._start_new_anim(new_anim)
            except queue.Empty:
                pass

            now = time.time()
            dt = now - last_time
            last_time = now

            if self.current_animation:
                elapsed = now - self.start_time
                if self.current_animation.is_finished(elapsed):
                    self.current_animation = None
                    self.led_controller.turn_off()
                else:
                    self.current_animation.draw_frame(dt, self.led_controller, elapsed)
                    self.led_controller.update_hardware()

            time.sleep(self.frame_interval)

    def _start_new_anim(self, anim: BaseAnimation) -> None:
        self.current_animation = anim
        self.start_time = time.time()
        self.led_controller.turn_off()
