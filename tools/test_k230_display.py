"""Regression checks for the LVGL draw format, double buffering and touch callbacks."""

from array import array
import importlib.util
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


class Buffer:
    def __init__(self, width, height, pixel_format):
        self.data = bytearray(width * height * (2 if pixel_format == 9 else 4))
        self.draw_line = Mock()
        self.draw_rectangle = Mock()
        self.draw_cross = Mock()
        self.draw_string_advanced = Mock()

    def clear(self):
        pass

    def bytearray(self):
        return self.data

    def size(self):
        return len(self.data)

    def virtaddr(self):
        return id(self.data)


class DisplayTests(unittest.TestCase):
    def setUp(self):
        self.driver = Mock()
        self.screen = Mock()
        self.buttons = []

        def button(parent):
            item = Mock()
            self.buttons.append(item)
            return item

        self.lv = SimpleNamespace(
            init=Mock(), deinit=Mock(), disp_create=Mock(return_value=self.driver),
            COLOR_FORMAT=SimpleNamespace(ARGB8888=17, RGB565=9),
            DISP_RENDER_MODE=SimpleNamespace(FULL=1),
            INDEV_TYPE=SimpleNamespace(POINTER=1),
            INDEV_STATE=SimpleNamespace(RELEASED=0, PRESSED=1),
            DIR=SimpleNamespace(NONE=0), EVENT=SimpleNamespace(CLICKED=7),
            font_load=Mock(return_value=object()), scr_act=Mock(return_value=self.screen),
            indev_create=Mock(return_value=Mock()), color_hex=lambda value: value,
            obj=Mock(), label=Mock(), btn=button, point_t=lambda point: point,
            img=Mock(), img_dsc_t=lambda descriptor: descriptor,
            timer_handler_run_in_period=Mock(),
        )
        self.touch = Mock()
        touch_factory = Mock(return_value=self.touch)
        touch_factory.EVENT_DOWN = 2
        touch_factory.EVENT_MOVE = 3
        self.display = SimpleNamespace(show_image=Mock(), LAYER_OSD0=4)

        def frombuffer(data, *, dtype):
            if dtype == 2:
                return WordPixels(data)
            self.assertEqual(dtype, 1)
            return SimpleNamespace(reshape=lambda shape: SimpleNamespace(data=data, shape=shape))

        class WordPixels:
            def __init__(self, data):
                self.words = array("H")
                self.words.frombytes(data)

            def __getitem__(self, key):
                return SimpleNamespace(copy=lambda: self.words[key])

        def flip(pixels, code):
            self.assertEqual(pixels.shape, (480, 640, 4))
            self.assertEqual(code, -1)
            words = array("I")
            words.frombytes(pixels.data)
            words.reverse()
            return words

        modules = {
            "image": SimpleNamespace(Image=Buffer, BGRA8888=17, RGB565=9),
            "lvgl": self.lv, "uctypes": SimpleNamespace(addressof=id),
            "machine": SimpleNamespace(TOUCH=touch_factory),
            "media": SimpleNamespace(__path__=[]),
            "media.display": SimpleNamespace(Display=self.display),
            "cv2": SimpleNamespace(flip=flip),
            "ulab": SimpleNamespace(numpy=SimpleNamespace(frombuffer=frombuffer, uint8=1, uint16=2)),
        }
        path = Path(__file__).resolve().parents[1] / "k230" / "touch_ui.py"
        spec = importlib.util.spec_from_file_location("display_under_test", path)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules), patch.object(
                sys, "path", [str(path.parent)] + sys.path):
            spec.loader.exec_module(module)
            self.sent = []
            self.ui = module.TouchMenu(self.sent.append)
        self.module = module

    def test_startup_accepts_status_only_ui(self):
        path = Path(self.module.__file__).with_name("main.py")
        spec = importlib.util.spec_from_file_location("k230_startup_test", path)
        app = importlib.util.module_from_spec(spec)
        uart = Mock()
        uart.return_value.write.side_effect = len
        status = Mock()
        modules = {
            "machine": SimpleNamespace(FPIOA=Mock(), UART=uart),
            "media": SimpleNamespace(__path__=[]),
            "media.sensor": SimpleNamespace(Sensor=Mock(), CAM_CHN_ID_0=0, CAM_CHN_ID_2=2),
            "media.display": SimpleNamespace(Display=Mock()),
            "media.media": SimpleNamespace(MediaManager=Mock()),
            "touch_ui": SimpleNamespace(StatusDisplay=status, PREVIEW_WIDTH=448, PREVIEW_HEIGHT=336),
            "steel_ball_detector": SimpleNamespace(MODEL_PATH="model.kmodel", SteelBallDetector=Mock()),
        }
        with patch.dict(sys.modules, modules), patch.object(
                sys, "path", [str(path.parent)] + sys.path), patch.object(
                os, "exitpoint", create=True), patch.object(
                os, "EXITPOINT_ENABLE", 1, create=True), patch.object(
                time, "ticks_ms", side_effect=KeyboardInterrupt, create=True):
            spec.loader.exec_module(app)
            app.run()
        status.assert_called_once_with()
        status.return_value.close.assert_called_once_with()

    def test_draw_format_buffers_and_actual_flushed_pointer(self):
        self.driver.set_color_format.assert_called_once_with(17)
        calls = [call[0] for call in self.driver.method_calls]
        self.assertLess(calls.index("set_color_format"), calls.index("set_draw_buffers"))
        first, second, size, mode = self.driver.set_draw_buffers.call_args.args
        self.assertIsNot(first, second)
        self.assertEqual(len(first), 640 * 480 * 4)
        self.assertEqual(len(second), size)
        self.lv.font_load.assert_called_once_with("A:/sdcard/balance_ui_20.fnt")
        self.screen.set_style_text_font.assert_called_once_with(self.ui.font, 0)
        self.screen.set_style_bg_opa.assert_called_once_with(255, 0)
        for index, buffer in enumerate(self.ui.buffers):
            width, height = self.module.DISPLAY_WIDTH, self.module.DISPLAY_HEIGHT
            marks = ((0, 0), (width - 1, 0), (0, height - 1), (width - 1, height - 1), (17, 211))
            for n, (x, y) in enumerate(marks):
                offset = (y * width + x) * 4
                buffer.data[offset:offset + 4] = bytes((n + 1 + 10 * index, 20 + n, 90, 255))
            original = bytes(buffer.data)
            self.ui._flush(self.driver, None, SimpleNamespace(__dereference__=lambda: buffer.data))
            output = self.display.show_image.call_args.args[0]
            self.assertIs(output, self.ui.rotated_buffer)
            self.assertEqual(self.display.show_image.call_args.kwargs, {"layer": 4})
            self.assertIs(self.ui.front_buffer, buffer)
            self.assertEqual(buffer.data, original)
            for x, y in marks:
                offset = (y * width + x) * 4
                rotated = ((height - 1 - y) * width + width - 1 - x) * 4
                self.assertEqual(output.data[rotated:rotated + 4], original[offset:offset + 4])
        with self.assertRaises(RuntimeError):
            self.ui._flush(self.driver, None, SimpleNamespace(__dereference__=lambda: b"unknown"))
        self.assertEqual(self.driver.flush_ready.call_count, 3)

    def test_normal_orientation_uses_the_original_buffer_and_touch(self):
        with patch.object(self.module, "ROTATE_DISPLAY_180", False):
            ui = self.module.TouchMenu(self.sent.append)
            buffer = ui.buffers[0]
            ui._flush(self.driver, None, SimpleNamespace(__dereference__=lambda: buffer.data))
            self.display.show_image.assert_called_once_with(buffer, layer=4)
            data = SimpleNamespace()
            self.touch.read.return_value = [SimpleNamespace(x=40, y=410, event=2)]
            ui._read_touch(None, data)
            self.assertEqual(data.point, {"x": 40, "y": 410})

    def test_button_bindings_and_touch_release(self):
        self.assertEqual(len(self.buttons), 6)
        for button in self.buttons:
            callback, event, _ = button.add_event.call_args.args
            self.assertEqual(event, self.lv.EVENT.CLICKED)
            callback(None)
        self.assertEqual(self.sent, [0, 1, 2, 3])
        data = SimpleNamespace()
        self.touch.read.return_value = [SimpleNamespace(x=40, y=410, event=2)]
        self.ui._read_touch(None, data)
        self.assertEqual((data.point, data.state), ({"x": 599, "y": 69}, 1))
        self.touch.read.return_value = []
        self.ui._read_touch(None, data)
        self.assertEqual(data.state, 0)

    def test_jog_uses_live_f407_count_and_ignores_legacy_or_stale_menu(self):
        forward, back = [button.add_event.call_args.args[0] for button in self.buttons[-2:]]
        frame = Mock()
        frame.bytearray.return_value = bytes(448 * 336 * 2)
        for row, connected in (("RUN:ON E:0 C:0", True), ("STEP:+0", False),
                               ("STEP:+0 ERR", True)):
            self.ui.update(["", "", "", row], "", connected, frame, None)
            forward(None)
            back(None)
        self.assertEqual(self.sent, [])
        self.ui.update(["", "", "", "STEP:+0"], "", True, frame, None)
        forward(None)
        back(None)
        self.assertEqual(self.sent, [4, 5])
        self.ui.lines[3].set_text.assert_called_with("步数 +0")
        self.ui.update(["", "", "", "STEP:-1"], "", True, frame, None)
        self.ui.lines[3].set_text.assert_called_with("步数 -1")
        self.ui.update(["", "", "", "STEP:-1"], "", False, frame, None)
        self.ui.lines[3].set_text.assert_called_with("步数 --")
        forward(None)
        self.assertEqual(self.sent, [4, 5])

    def test_startup_calibration_and_jog_lock(self):
        class FirmwareString(str):
            def startswith(self, prefix):
                if isinstance(prefix, tuple):
                    raise TypeError("tuple prefix is unsupported")
                return super().startswith(prefix)

        frame = Mock()
        frame.bytearray.return_value = bytes(448 * 336 * 2)
        up = self.buttons[0].add_event.call_args.args[0]
        forward = self.buttons[-2].add_event.call_args.args[0]
        self.ui.update(["RX:OK", FirmwareString("CAL: SET LOW"), "UP: SAVE LOW", "STEP:0/640 ERR"],
                       "", True, frame, None)
        self.ui.help_label.set_text.assert_any_call("上移：SAVE LOW")
        self.ui.lines[3].set_text.assert_called_with("步数 0/640")
        forward(None)
        up(None)
        self.assertEqual(self.sent, [0])
        self.ui.update(["RX:OK", FirmwareString("HOME: SWEEP UP"), "ANGLE:+1.5 deg", "STEP:320/640 ERR"],
                       "", True, frame, None)
        self.ui.help_label.set_text.assert_any_call("返回：STOP")
        forward(None)
        self.assertEqual(self.sent, [0])
        self.ui.update(["RX:OK", FirmwareString("HOME: READY"), "ANGLE:-0.05 deg", "STEP:240/640"],
                       "", True, frame, None)
        forward(None)
        self.assertEqual(self.sent, [0, 4])

    def test_first_task_menu_blocks_jog_and_shows_start(self):
        frame = Mock()
        frame.bytearray.return_value = bytes(448 * 336 * 2)
        self.ui.update(["F407 RX:OK", "T1: WAIT BALL", "A:+0.1 deg", "STEP:240/640"],
                       "X:+0 mm", True, frame, None)
        self.assertEqual(self.ui.last_help, "OK:START\nBACK:STOP")
        self.buttons[-2].add_event.call_args.args[0](None)
        self.assertEqual(self.sent, [])
        self.ui.update(["F407 RX:OK", "T1: HOLD", "A:+0.1 deg", "STEP:240/640"],
                       "X:+0 mm", True, frame, None)
        self.assertEqual(self.ui.last_help, "BACK:STOP")

    def test_preview_pixels_overlay_scaling_and_no_ball_refresh(self):
        source = Buffer(448, 336, 9)
        source.data[:4] = b"\x12\x34\x56\x78"
        source.data[-4:] = b"\x89\xab\xcd\xef"
        self.ui.update(["a", "b", "c", "d"], "YOLO: NO BALL\nCALIBRATION PENDING", True,
                       source, (100, 200, 40, 20))
        frame = self.ui.camera_preview
        y = 335 - self.module.ALIGN_LINE_Y
        frame.draw_line.assert_any_call(0, y, 447, y, color=(255, 0, 0), thickness=2)
        frame.draw_rectangle.assert_called_once_with(350, 182, 28, 14, color=(0, 255, 0), thickness=2)
        frame.draw_cross.assert_called_once_with(363, 188, color=(255, 255, 0), size=8)
        self.assertEqual(len(self.module.PIPE_TICK_X), 21)
        self.assertEqual(self.module.PIPE_TICK_MM, 10)
        self.assertTrue(all(a < b for a, b in zip(self.module.PIPE_TICK_X, self.module.PIPE_TICK_X[1:])))
        self.assertEqual(frame.draw_line.call_count, 22)
        labels = [call.args[3] for call in frame.draw_string_advanced.call_args_list]
        self.assertEqual(labels, [str(v) if v <= 0 else "+%d" % v for v in range(10, -11, -1)])
        for x, tick, label in zip(self.module.PIPE_TICK_X, frame.draw_line.call_args_list[1:],
                                  frame.draw_string_advanced.call_args_list):
            self.assertEqual(tick.args[0], 447 - round(x))
            self.assertEqual(tick.args[0], label.args[0] + 10)
        self.assertEqual(self.ui.preview_data[:4], b"\xcd\xef\x89\xab")
        self.assertEqual(self.ui.preview_data[-4:], b"\x56\x78\x12\x34")
        source.draw_line.assert_not_called()
        self.assertEqual(self.ui.preview_descriptor["header"]["cf"], 9)
        self.ui.preview.set_src.assert_called_once_with(self.ui.preview_descriptor)
        self.ui.preview.invalidate.assert_called_once_with()
        frame.draw_line.reset_mock()
        frame.draw_rectangle.reset_mock()
        frame.draw_cross.reset_mock()
        self.ui.update(["a", "b", "c", "d"], "YOLO: NO BALL", False, source, None)
        frame.draw_line.assert_any_call(0, y, 447, y, color=(255, 0, 0), thickness=2)
        frame.draw_rectangle.assert_not_called()
        frame.draw_cross.assert_not_called()
        self.assertEqual(self.ui.preview.invalidate.call_count, 2)
        source.data = b"bad size"
        with self.assertRaises(ValueError):
            self.ui.update(["a", "b", "c", "d"], "", False, source, None)

    def test_alignment_line_position_and_bounds(self):
        frame = Mock()
        frame.bytearray.return_value = bytes(448 * 336 * 2)
        with patch.object(self.module, "ALIGN_LINE_Y", 100):
            self.ui.update(["", "", "", ""], "STOPPED - LOST", False, frame, None)
        self.ui.camera_preview.draw_line.assert_any_call(0, 235, 447, 235, color=(255, 0, 0), thickness=2)
        for y in (-1, 336, 1.5):
            with self.subTest(y=y), patch.object(self.module, "ALIGN_LINE_Y", y):
                with self.assertRaises(ValueError):
                    self.module.TouchMenu(self.sent.append)


if __name__ == "__main__":
    unittest.main()
