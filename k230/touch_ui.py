"""LVGL setup follows the board's ybMain/main.py and 02.lvgl_button.py."""

import image
import lvgl as lv
import uctypes
from machine import TOUCH
from media.display import Display

PREVIEW_WIDTH = 448
PREVIEW_HEIGHT = 336
ALIGN_LINE_Y = 150  # Preview Y=0..335, increases downward.


class TouchMenu:
    def __init__(self, send_key):
        if not isinstance(ALIGN_LINE_Y, int) or not 0 <= ALIGN_LINE_Y < PREVIEW_HEIGHT:
            raise ValueError("ALIGN_LINE_Y must be an integer from 0 to 335")
        self.menu_revision = 0
        self.last_lines = None
        self.last_status = None
        self.jog_ready = False
        lv.init()
        self.font = lv.font_load("A:/sdcard/balance_ui_20.fnt")
        if self.font is None:
            raise RuntimeError("Missing balance_ui_20.fnt font file")
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

        self.status = lv.label(screen)
        self.status.set_pos(8, 4)
        self.status.set_size(624, 52)

        # Own the pixels so the camera may reuse its capture buffer after each frame.
        self.preview_data = bytearray(PREVIEW_WIDTH * PREVIEW_HEIGHT * 2)
        self.preview_descriptor = lv.img_dsc_t({
            "header": {"always_zero": 0, "cf": lv.COLOR_FORMAT.RGB565,
                       "w": PREVIEW_WIDTH, "h": PREVIEW_HEIGHT},
            "data_size": len(self.preview_data), "data": self.preview_data,
        })
        self.preview = lv.img(screen)
        self.preview.set_pos(8, 64)
        self.preview.set_src(self.preview_descriptor)

        panel = lv.obj(screen)
        panel.set_pos(464, 64)
        panel.set_size(168, 336)
        panel.set_style_bg_color(lv.color_hex(0x171D29), 0)
        panel.set_style_bg_opa(255, 0)
        panel.set_style_border_width(0, 0)
        panel.set_style_pad_all(4, 0)
        panel.set_style_text_color(lv.color_hex(0xFFFFFF), 0)
        panel.set_style_text_font(self.font, 0)
        panel.set_scroll_dir(lv.DIR.NONE)
        self.lines = []
        for row in range(4):
            label = lv.label(panel)
            label.set_pos(0, row * 44)
            label.set_size(156, 44)
            self.lines.append(label)

        self.help_label = lv.label(panel)
        self.help_label.set_pos(0, 184)
        self.help_label.set_width(156)
        self.legacy_help = "上移：测试值+1\n下移：测试值-1\n确认：启停识别\n返回：清测试值\n前进/后退：1步"
        self.last_help = self.legacy_help
        self.help_label.set_text(self.last_help)

        self.jog_buttons = []
        for index, text in enumerate(("上移 +1", "下移 -1", "确认", "返回", "前进", "后退")):
            jog = index >= 4
            button = lv.btn(screen)
            button.set_pos(8 + index * 105, 416)
            button.set_size(98, 56)
            button.set_style_bg_color(lv.color_hex(0x3D4655 if jog else 0x1C4D99), 0)
            if jog:
                self.jog_buttons.append(button)
            label = lv.label(button)
            label.set_style_text_font(self.font, 0)
            label.set_style_text_color(lv.color_hex(0xFFFFFF), 0)
            label.set_text(text)
            label.center()
            def on_click(event, index=index, jog=jog):
                if not jog or self.jog_ready:
                    send_key(index)

            button.add_event(on_click, lv.EVENT.CLICKED, None)

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

    def update(self, lines, observation_status, connected, frame, box):
        frame.draw_line(0, ALIGN_LINE_Y, PREVIEW_WIDTH - 1, ALIGN_LINE_Y,
                        color=(255, 0, 0), thickness=2)
        if box is not None:
            x, y, width, height = box
            x1 = max(0, min(PREVIEW_WIDTH - 1, round(x * PREVIEW_WIDTH / 640)))
            y1 = max(0, min(PREVIEW_HEIGHT - 1, round(y * PREVIEW_HEIGHT / 480)))
            x2 = max(x1 + 1, min(PREVIEW_WIDTH, round((x + width) * PREVIEW_WIDTH / 640)))
            y2 = max(y1 + 1, min(PREVIEW_HEIGHT, round((y + height) * PREVIEW_HEIGHT / 480)))
            frame.draw_rectangle(x1, y1, x2 - x1, y2 - y1, color=(0, 255, 0), thickness=2)
            frame.draw_cross((x1 + x2) // 2, (y1 + y2) // 2, color=(255, 255, 0), size=8)
        pixels = frame.bytearray()
        if len(pixels) != len(self.preview_data):
            raise ValueError("Preview frame size must be 448x336 RGB565")
        self.preview_data[:] = pixels
        self.preview.invalidate()
        observation_status = {
            "STOPPED - LOST": "视觉观测已停止",
            "CALIBRATION / DETECTOR PENDING": "待接入识别与标定",
            "NO VALID BALL - LOST": "未检测到有效小球",
        }.get(observation_status, observation_status)
        status = ("主控菜单已同步" if connected else "未收到主控回包") + " | " + observation_status.replace("\n", " | ")
        if status != self.last_status:
            self.status.set_text(status)
            self.last_status = status
        if lines[3].startswith("STEP:"):
            step_status = lines[3][5:]
            motor_cycle = lines[1].startswith("MOTOR:")
            balance_mode = lines[1].startswith("BAL:")
            setting_level = lines[1] == "BAL: SET LEVEL"
            jog_ready = connected and (setting_level or not (motor_cycle or balance_mode)) and not step_status.endswith(" ERR")
            lines = list(lines)
            startup = lines[1].startswith("CAL:") or lines[1].startswith("HOME:")
            if startup and step_status.endswith(" ERR"):
                step_status = step_status[:-4]
            lines[3] = "步数 " + (step_status if connected else "--")
            if setting_level:
                help_text = "UP/DOWN:25\nOK:SAVE LEVEL\nBACK:STOP"
            elif balance_mode:
                if lines[1] == "BAL: STARTUP UP" or lines[1].startswith("BAL: RETURN "):
                    help_text = "BACK:STOP"
                else:
                    help_text = ("OK/BACK:STOP" if lines[1] in ("BAL: ACTIVE", "BAL: HOLD")
                                 else "UP:+  DOWN:-\nOK:START")
            elif motor_cycle:
                help_text = "返回：STOP" if lines[1] in ("MOTOR:FWD", "MOTOR:REV") else ""
            elif startup:
                help_text = {
                    "CAL: SET LOW": "上移：SAVE LOW",
                    "CAL: SET LEVEL": "下移：SAVE LEVEL",
                    "HOME: CHECK LOW": "返回：STOP",
                    "HOME: SWEEP UP": "返回：STOP",
                    "HOME: RETURN LEVEL": "返回：STOP",
                    "HOME: LEVELING": "返回：STOP",
                    "HOME: READY": "确认：启停识别\n前进/后退：1步",
                    "HOME: MANUAL": "确认：启停识别\n前进/后退：1步",
                }.get(lines[1], "")
            else:
                help_text = self.legacy_help
        else:
            jog_ready = False
            help_text = self.legacy_help
        if help_text != self.last_help:
            self.help_label.set_text(help_text)
            self.last_help = help_text
        if jog_ready != self.jog_ready:
            for button in self.jog_buttons:
                button.set_style_bg_color(lv.color_hex(0x166453 if jog_ready else 0x3D4655), 0)
            self.jog_ready = jog_ready
        if lines != self.last_lines:
            for label, text in zip(self.lines, lines):
                label.set_text(text)
            self.last_lines = list(lines)
        lv.timer_handler_run_in_period(1)

    def close(self):
        lv.deinit()
