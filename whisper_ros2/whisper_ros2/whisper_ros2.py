import os
import numpy as np
from multiprocessing import Process
# ros2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, String, Bool
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from pydub import AudioSegment, playback
from ctypes import *
from contextlib import contextmanager
import pyaudio
from io import BytesIO
import numpy as np
import soundfile as sf
import speech_recognition as sr
import whisper

ERROR_HANDLER_FUNC = CFUNCTYPE(
    None, c_char_p, c_int, c_char_p, c_int, c_char_p)


def py_error_handler(filename, line, function, err, fmt):
    pass


c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)


@contextmanager
def noalsaerr():
    asound = cdll.LoadLibrary('libasound.so')
    asound.snd_lib_error_set_handler(c_error_handler)
    yield
    asound.snd_lib_error_set_handler(None)


class WhisperNode(Node):
    # ノード名
    SELFNODE = "chatgpt"

    def __init__(self):
        # ノードの初期化
        super().__init__(self.SELFNODE)
        self.get_logger().info("%s initializing..." % (self.SELFNODE))
        with noalsaerr():
            p = pyaudio.PyAudio()
        self.model_name = self.param("model_name", "medium").string_value
        timer_period = self.param("timer_period", 0.01).double_value
        # whisper
        self.model = whisper.load_model(self.model_name)
        self.recognizer = sr.Recognizer()
        # ros2 init
        self.text_pub_ = self.create_publisher(String, 'chatgpt/input_text', 1)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.speak_sub_ = self.create_subscription(
            Bool, 'voicevox_ros2/speak', self.speak_callback, 1)
        self.speak = True
        self.log_out = False

    def __del__(self):
        self.get_logger().info("%s done." % self.SELFNODE)

    def get_text(self):
        self.get_logger().info("Listening ...")
        with sr.Microphone(sample_rate=16_000) as source:
            self.get_logger().info("Talk to me. ")
            self.recognizer.adjust_for_ambient_noise(source)
            audio = self.recognizer.listen(source, timeout=1000.0)
        try:
            self.get_logger().info("Voice processing ...")
            # 「音声データをWhisperの入力形式に変換」参照
            wav_bytes = audio.get_wav_data()
            wav_stream = BytesIO(wav_bytes)
            audio_array, sampling_rate = sf.read(wav_stream)
            audio_fp32 = audio_array.astype(np.float32)
            result = self.model.transcribe(audio_fp32, language="ja", fp16=False)
            return result["text"]
            # print(result["text"])
        except sr.UnknownValueError:
            self.get_logger().info("Sorry, I could not understand what you said.")
        return None

    def param(self, name, value):
        self.declare_parameter(name, value)
        return self.get_parameter(name).get_parameter_value()

    def speak_callback(self, msg):
        self.speak = msg.data

    def listen_task(self):
        if self.speak:
            if not self.log_out:
                self.get_logger().info("Listening stop")
                self.log_out = True
            return
        text = self.get_text()
        if text is None or text == "":
            return
        self.get_logger().info("You said: %s" % text)
        self.text_pub_.publish(String(data=text))
        self.speak = True
        self.log_out = False

    def timer_callback(self):
        self.listen_task()
        pass


def main(args=None):
    try:
        rclpy.init(args=args)
        node = WhisperNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了処理
        rclpy.shutdown()


if __name__ == '__main__':
    main()
