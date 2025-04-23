#!/usr/bin/env python

import rospy
import pyaudio
import numpy as np
from std_msgs.msg import Int16MultiArray

# Initialize ROS node
rospy.init_node('audio_streamer')
audio_hz = rospy.get_param("audio_samplerate")
buffer_size = rospy.get_param("buffer")
device_index = rospy.get_param("device_index")
topic_name = rospy.get_param("audio_stream_topic")

# Publisher for audio frames
audio_pub = rospy.Publisher(topic_name, Int16MultiArray, queue_size=10)

# Function to record and publish audio
def audio_stream(device_index = 7, sample_rate=16000):
    p = pyaudio.PyAudio()
    
    # Open the stream for audio capture
    stream = p.open(format=pyaudio.paInt16,
                    channels=1,
                    rate=sample_rate,
                    input=True,
                    frames_per_buffer=buffer_size,
                    input_device_index=device_index)

    rospy.loginfo("Recording...")

    # Capture audio and publish frames
    while not rospy.is_shutdown():
        data = stream.read(buffer_size)
        audio_msg = Int16MultiArray()
        audio_msg.data = np.frombuffer(data, dtype=np.int16).tolist()
        audio_pub.publish(audio_msg)  # Publish audio data to topic
        rospy.loginfo("Publishing audio frame...")

    rospy.loginfo("Recording finished.")
    stream.stop_stream()
    stream.close()
    p.terminate()

if __name__ == '__main__':
    try:
        # Start audio streaming
        audio_stream(device_index, audio_hz)
    except rospy.ROSInterruptException:
        pass
