"""LVGL setup follows the board's ybMain/main.py and 02.lvgl_button.py."""

import image
import lvgl as lv
import uctypes
from machine import TOUCH
from media.display import Display


class TouchMenu:
    def __init__(self, send_key):
        self.menu_revision = 0
        self.last_lines = None
        self.last_status = None
        lv.init()
        self.font = lv.font_load("A:/sdcard/balance_ui_28.fnt")
        if self.font is None:
            raise RuntimeError("Missing balance_ui_28.fnt font file")
        self.display = lv.disp_create(640, 480)
        # LVGL must render 32-bit pixels into the BGRA8888 image buffers.
        self.display.set_color_format(lv.COLOR_FORMAT.ARGB8888)
        self.buffers = [image.Image(640, 480, image.BGRA8888) for _ in range(2)]
        for buffer in self.buffers:
            buffer.clear()
        # Keep the bytearray wrappers alive while LVGL holds their raw pointers.
        self.draw_buffers = [buffer.bytearray() for buffer in self.buffers]
        self.front_buffer = self.buffers[0]
        self.display.set_draw_buffers(
            self.draw_buffers[0], self.draw_buffers[1],
            self.buffers[0].size(), lv.DISP_RENDER_MODE.FULL,
        )
        self.display.set_flush_cb(self._flush)
        self.touch = TOUCH(0)
        self.input = lv.indev_create()
        self.input.set_type(lv.INDEV_TYPE.POINTER)
        self.input.set_read_cb(self._read_touch)

        screen = lv.scr_act()
        screen.set_style_bg_color(lv.color_hex(0x0D111A), 0)
        screen.set_style_bg_opa(255, 0)
        screen.set_style_text_font(self.font, 0)
        screen.set_style_text_color(lv.color_hex(0xFFFFFF), 0)
        screen.set_scroll_dir(lv.DIR.NONE)

        panel = lv.obj(screen)
        panel.set_pos(16, 16)
        panel.set_size(608, 334)
        panel.set_style_bg_color(lv.color_hex(0x171D29), 0)
        panel.set_style_bg_opa(255, 0)
        panel.set_style_border_width(0, 0)
        panel.set_style_pad_all(16, 0)
        panel.set_style_text_color(lv.color_hex(0xFFFFFF), 0)
        panel.set_style_text_font(self.font, 0)
        panel.set_scroll_dir(lv.DIR.NONE)
        self.status = lv.label(panel)
        self.status.set_pos(0, 0)
        self.status.set_width(570)
        self.lines = []
        for row in range(4):
            label = lv.label(panel)
            label.set_pos(12, 100 + row * 45)
            label.set_width(540)
            self.lines.append(label)

        for index, text in enumerate(("上移", "下移", "确认", "返回")):
            button = lv.btn(screen)
            button.set_pos(16 + index * 156, 382)
            button.set_size(140, 76)
            button.set_style_bg_color(lv.color_hex(0x1C4D99), 0)
            label = lv.label(button)
            label.set_style_text_font(self.font, 0)
            label.set_style_text_color(lv.color_hex(0xFFFFFF), 0)
            label.set_text(text)
            label.center()
            button.add_event(lambda event, index=index: send_key(index), lv.EVENT.CLICKED, None)

    def _flush(self, driver, area, color):
        try:
            address = uctypes.addressof(color.__dereference__())
            for buffer in self.buffers:
                if buffer.virtaddr() == address:
                    self.front_buffer = buffer
                    Display.show_image(buffer, layer=Display.LAYER_OSD0)
                    break
            else:
                raise RuntimeError("LVGL returned an unknown draw buffer")
        finally:
            driver.flush_ready()

    def _read_touch(self, driver, data):
        x, y = 0, 0
        state = lv.INDEV_STATE.RELEASED
        points = self.touch.read(1)
        if points:
            point = points[0]
            x, y = point.x, point.y
            if point.event in (TOUCH.EVENT_DOWN, TOUCH.EVENT_MOVE):
                state = lv.INDEV_STATE.PRESSED
        data.point = lv.point_t({"x": x, "y": y})
        data.state = state

    def update(self, lines, observation_status, connected):
        observation_status = {
            "STOPPED - LOST": "视觉观测已停止",
            "CALIBRATION / DETECTOR PENDING": "待接入识别与标定",
            "NO VALID BALL - LOST": "未检测到有效小球",
        }.get(observation_status, observation_status)
        status = ("主控菜单已同步" if connected else "未收到主控回包") + "\n" + observation_status
        if status != self.last_status:
            self.status.set_text(status)
            self.last_status = status
        if lines != self.last_lines:
            for label, text in zip(self.lines, lines):
                label.set_text(text)
            self.last_lines = list(lines)
        lv.timer_handler_run_in_period(1)

    def close(self):
        lv.deinit()
