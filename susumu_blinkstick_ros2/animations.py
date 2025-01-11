from typing import Tuple


class BaseAnimation:
    def __init__(self, priority: int, duration: float, brightness: float, decay_rate: float, speed: float):
        self.priority = priority
        self.duration = duration
        self.brightness = brightness
        self.decay_rate = decay_rate
        self.speed = speed
        self._old_pos = 0.0

    def draw_frame(self, dt: float, led_controller: "LEDController", elapsed: float) -> None:
        # 残像
        led_controller.apply_trail(self.decay_rate)
        # パターン別描画
        self._draw_pattern(dt, led_controller, elapsed)

    def _draw_pattern(self, dt: float, led_controller: "LEDController", elapsed: float) -> None:
        raise NotImplementedError()

    def is_finished(self, elapsed: float) -> bool:
        return elapsed > self.duration

    def _apply_brightness(self, color: Tuple[int, int, int]) -> Tuple[int, int, int]:
        r = int(color[0] * self.brightness)
        g = int(color[1] * self.brightness)
        b = int(color[2] * self.brightness)
        return (r, g, b)


class StopAnimation(BaseAnimation):
    def __init__(self, priority: int, duration: float, color: Tuple[int, int, int], brightness: float, decay_rate: float, speed: float):
        super().__init__(priority, duration, brightness, decay_rate, speed)

    def _draw_pattern(self, dt, led_controller, elapsed) -> None:
        pass

class SolidAnimation(BaseAnimation):
    def __init__(self, priority: int, duration: float, color: Tuple[int, int, int], brightness: float, decay_rate: float, speed: float):
        super().__init__(priority, duration, brightness, decay_rate, speed)
        self.color = color

    def _draw_pattern(self, dt, led_controller, elapsed) -> None:
        c = self._apply_brightness(self.color)
        led_controller.set_all_color(c)


class BlinkAnimation(BaseAnimation):
    def __init__(self, priority: int, duration: float, color: Tuple[int, int, int], brightness: float, decay_rate: float, speed: float):
        super().__init__(priority, duration, brightness, decay_rate, speed)
        self.color = color

    def _draw_pattern(self, dt, led_controller, elapsed) -> None:
        pos_int = int(self._old_pos)
        if (pos_int % 2) == 0:
            c = self._apply_brightness(self.color)
            led_controller.set_all_color(c)
        else:
            led_controller.set_all_color((0, 0, 0))
        self._old_pos += self.speed * dt


class TwoColorFadeAnimation(BaseAnimation):
    def __init__(self, priority: int, duration: float, color1: Tuple[int, int, int], color2: Tuple[int, int, int], brightness: float, decay_rate: float, speed: float):
        super().__init__(priority, duration, brightness, decay_rate, speed)
        self.color1 = color1
        self.color2 = color2

    def _draw_pattern(self, dt, led_controller, elapsed) -> None:
        ratio = min(elapsed / self.duration, 1.0)
        r = int((1 - ratio) * self.color1[0] + ratio * self.color2[0])
        g = int((1 - ratio) * self.color1[1] + ratio * self.color2[1])
        b = int((1 - ratio) * self.color1[2] + ratio * self.color2[2])
        c = self._apply_brightness((r, g, b))
        led_controller.set_all_color(c)


class LeftToRightAnimation(BaseAnimation):
    def __init__(self, priority: int, duration: float, color: Tuple[int, int, int], brightness: float, decay_rate: float, speed: float):
        super().__init__(priority, duration, brightness, decay_rate, speed)
        self.color = color
        self._pos = 0.0

    def _draw_pattern(self, dt, led_controller, elapsed) -> None:
        old_pos = self._old_pos
        new_pos = old_pos + self.speed * dt
        step_count = int(abs(new_pos - old_pos) * 10) + 1
        c = self._apply_brightness(self.color)

        for step in range(step_count + 1):
            t = step / step_count
            mid = old_pos + (new_pos - old_pos) * t
            idx = int(mid) % led_controller.led_count
            led_controller.set_led_color(idx, c)

        self._old_pos = new_pos


class RightToLeftAnimation(BaseAnimation):
    def __init__(self, priority: int, duration: float, color: Tuple[int, int, int], brightness: float, decay_rate: float, speed: float):
        super().__init__(priority, duration, brightness, decay_rate, speed)
        self.color = color

    def _draw_pattern(self, dt, led_controller, elapsed) -> None:
        old_pos = self._old_pos
        new_pos = old_pos + self.speed * dt
        step_count = int(abs(new_pos - old_pos) * 10) + 1
        c = self._apply_brightness(self.color)

        for step in range(step_count + 1):
            t = step / step_count
            mid = old_pos + (new_pos - old_pos) * t
            idx = int(mid) % led_controller.led_count
            idx = led_controller.led_count - 1 - idx
            led_controller.set_led_color(idx, c)

        self._old_pos = new_pos


class WaveAnimation(BaseAnimation):
    def __init__(self, priority: int, duration: float, color: Tuple[int, int, int], brightness: float, decay_rate: float, speed: float):
        super().__init__(priority, duration, brightness, decay_rate, speed)
        self.color = color

    def _draw_pattern(self, dt, led_controller, elapsed) -> None:
        old_pos = self._old_pos
        new_pos = old_pos + self.speed * dt
        step_count = int(abs(new_pos - old_pos) * 10) + 1
        c = self._apply_brightness(self.color)
        n = led_controller.led_count
        cycle = 2 * (n - 1)

        def triwave(x: float) -> int:
            t = x % cycle
            if t < (n - 1):
                return int(t)
            else:
                return int(cycle - t)

        for step in range(step_count + 1):
            ratio = step / step_count
            mid = old_pos + (new_pos - old_pos) * ratio
            idx = triwave(mid)
            led_controller.set_led_color(idx, c)

        self._old_pos = new_pos


class InwardAnimation(BaseAnimation):
    def __init__(self, priority: int, duration: float, color: Tuple[int, int, int], brightness: float, decay_rate: float, speed: float):
        super().__init__(priority, duration, brightness, decay_rate, speed)
        self.color = color

    def _draw_pattern(self, dt, led_controller, elapsed) -> None:
        old_pos = self._old_pos
        new_pos = old_pos + self.speed * dt
        step_count = int(abs(new_pos - old_pos) * 10) + 1
        c = self._apply_brightness(self.color)
        n = led_controller.led_count
        half = n // 2

        for step in range(step_count + 1):
            ratio = step / step_count
            mid = old_pos + (new_pos - old_pos) * ratio
            idx = int(mid) % (half + 1)
            left = idx
            right = n - 1 - idx
            led_controller.set_led_color(left, c)
            led_controller.set_led_color(right, c)

        self._old_pos = new_pos


class OutwardAnimation(BaseAnimation):
    def __init__(self, priority: int, duration: float, color: Tuple[int, int, int], brightness: float, decay_rate: float, speed: float):
        super().__init__(priority, duration, brightness, decay_rate, speed)
        self.color = color

    def _draw_pattern(self, dt, led_controller, elapsed) -> None:
        old_pos = self._old_pos
        new_pos = old_pos + self.speed * dt
        step_count = int(abs(new_pos - old_pos) * 10) + 1
        c = self._apply_brightness(self.color)
        n = led_controller.led_count
        half = n // 2
        center_left = (n - 1) // 2
        center_right = n // 2

        for step in range(step_count + 1):
            ratio = step / step_count
            mid = old_pos + (new_pos - old_pos) * ratio
            idx = int(mid) % (half + 1)
            left = center_left - idx
            right = center_right + idx
            if 0 <= left < n:
                led_controller.set_led_color(left, c)
            if 0 <= right < n:
                led_controller.set_led_color(right, c)

        self._old_pos = new_pos

