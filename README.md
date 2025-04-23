# tiago_audio_streamer
The present ROS1 package implements continuous audio streaming from TIAGo's microphones.

It sends audio frames using std_msgs.msg's Int16MultiArray, which is defined as follows: 

\# Please look at the MultiArrayLayout message definition for
\# documentation on all multiarrays.

MultiArrayLayout  layout        \# specification of data layout
int16[]           data          \# array of data


In order to be able to receive the data from this topic and convert it to actual audio data, 
you need to add the following code to your node:


<pre>``` python  
audio_sub = rospy.Subscriber(name of the audio topic, default is /audio_frames, Int16MultiArray, self.audio_callback) ```</pre>


You can store the received audio frames using the following code: 

from queue import Queue

audio_buffer = Queue

def audio_callback(msg: Int16MultiArray):
    try:
        audio_data = np.array(msg.data, dtype=np.int16)
        audio_buffer.put(audio_data)        
    
    except Exception as e:
        rospy.logerr(f"Failed to store audio data due to: {e}")

And you can convert stored frames into an audio file through: 

import wave
import time

def save_audio(self):
    if self.audio_buffer.empty():
        rospy.logwarn("No audio data to save.")
        return False

    # Combine all audio frames
    frames = list(self.audio_buffer.queue)
    audio = np.concatenate(frames)

    # Convert to bytes
    audio_bytes = audio.tobytes()

    # Filename with timestamp
    filename = f"/tmp/recorded_audio_{int(time.time())}.wav" #change to fit your own needs.

    try:
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(1)  # Mono
            wf.setsampwidth(2)  # 2 bytes for int16
            wf.setframerate(<sample_rate>) #set the samplerate to your robot's samplerate
            wf.writeframes(audio_bytes) 

        rospy.loginfo(f"Audio saved to {filename}")
        self.audio_buffer = Queue()  # Clear buffer after saving
        return True

    except Exception as e:
        rospy.logerr(f"Failed to save audio: {e}")
        return False


