# tiago_audio_streamer
The present ROS1 package implements continuous audio streaming from TIAGo's microphones.

It sends audio frames using std_msgs.msg's Int16MultiArray, which is defined as follows: 

\# Please look at the MultiArrayLayout message definition for
\# documentation on all multiarrays.

MultiArrayLayout  layout        \# specification of data layout
int16[]           data          \# array of data


In order to be able to receive the data from this topic and convert it to actual audio data, 
you need to add the following code to your node:


```python  
audio_sub = rospy.Subscriber(<name of the audio topic, default is /audio_frames>, Int16MultiArray, audio_callback) 
```


You can store the received audio frames using the following code: 

```python
from queue import Queue

audio_buffer = Queue

def audio_callback(msg: Int16MultiArray):
    try:
        audio_data = np.array(msg.data, dtype=np.int16)
        audio_buffer.put(audio_data)        
    
    except Exception as e:
        rospy.logerr(f"Failed to store audio data due to: {e}")
```

And you can convert stored frames into an audio file through: 

```python
import wave
import time

def save_audio(self):
    if audio_buffer.empty():
        rospy.logwarn("No audio data to save.")
        return False

    # Combine all audio frames
    frames = list(audio_buffer.queue)
    audio = np.concatenate(frames)

    # Convert to bytes
    audio_bytes = audio.tobytes()

    # Filename with timestamp
    filename = f"/tmp/recorded_audio_{int(time.time())}.wav" #change how the file is named to fit your needs.

    try:
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(1)  # Mono
            wf.setsampwidth(2)  # 2 bytes for int16, since we are using Int16MultiArray to send data.
            wf.setframerate(<sample_rate>) #set the samplerate to your robot's samplerate
            wf.writeframes(audio_bytes) 

        rospy.loginfo(f"Audio saved to {filename}")
        audio_buffer = Queue()  # Clear buffer after saving, comment if you want to keep the buffer
        return True

    except Exception as e:
        rospy.logerr(f"Failed to save audio: {e}")
        return False
```

