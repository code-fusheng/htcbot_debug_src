#!/usr/bin/env python2
#-*-coding:utf-8-*-

import rospy
import subprocess
from htcbot_msgs.msg import VideoStreamControl

class App:
    def __init__(self):
        rospy.init_node("video_streamer_node")
        self.input_url  = "rtsp://192.168.1.150:8557/h264"
        self.output_url = "rtsp://47.111.158.6:8554/live1"
        self.default_cmd =  ['ffmpeg', '-rtsp_transport', 'tcp', '-i', self.input_url , '-vcodec', 'libx264', '-an', '-f', 'flv', self.output_url]
        self.process = None

    def start_stream(self, cmd=None):
        if self.process != None:
            rospy.logwarn("[video streamer] Video streamer is running, ignore new SEND command")
            return

        final_cmd = ['ffmpeg', '-rtsp_transport', 'tcp', '-i', self.input_url , '-vcodec', 'libx264', '-an', '-f', 'flv', self.output_url]
        if cmd != None:
            final_cmd = cmd
        rospy.loginfo("[video_streamer] Start process to send video stream from {} to {} with command {}".format(self.input_url, self.output_url, final_cmd))
        self.process = subprocess.Popen(final_cmd)

    def stop_stream(self):
        rospy.loginfo("[video_streamer] Stop video streamer")
        if self.process != None:
            self.process.terminate()
            self.process.wait()
            self.process = None

    def run(self):
        self.sub_control = rospy.Subscriber("video_streamer_control", VideoStreamControl, self._cb_control)

        rospy.spin()

    def _cb_control(self, msg):
        if msg.source_url:
            self.input_url = msg.source_url

        if msg.target_url:
            self.output_url = msg.target_url

        if msg.command == msg.COMMAND_SEND:
            if len(msg.cmd) < 1:
                self.start_stream(None)
            else:
                self.stop_stream(msg.cmd)
        elif msg.command == msg.COMMAND_STOP:
            self.stop_stream()
        else:
            rospy.loginfo("[video_streamer] Invalid command: {}".format(msg.command))

    def shutdown(self):
        self.stop_stream()


if __name__ == "__main__":
    app = App()
    try:
        app.run()
    except Exception as e:
        print(e)
    finally:
        app.shutdown()
