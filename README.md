# tiago_audio_streamer

The present ROS1 package implements continuous audio streaming from TIAGo's microphones.


## How to install it in TIAGo? 

You need to deploy it in TIAGo. You can clone the repository into your robot after installing git or copying it from your computer. The important thing is to have the code inside the /home/pal/test_ws/src folder. The code currently works for ROS Noetic, but there is not apparent reason why it should not work for ROS Melodic. Let me knoe if it fails for Melodic and I will update the code. 

You should build it using:

```bash

cd /home/pal/test_ws

catkin build tiago_audio_streamer

```

If no errors happened during the building process, test it with: 

```bash

source devel/setup.bash

roslaunch tiago_audio_streamer audio_stream.launch

```

You need to check whether or not any audio is getting streamed. 
In the computer connected to the robot, use the following command:

```bash

rostopic echo <your_topic_name, default is /audio_frames>

```

Clap or talk and check if the number are changing. 

If they do not change, you need to change, check the microphone configurations
inside TIAGo using command: 

```bash

alsamixer

```

And increase the volume of the microphones of the robot in all available devices. 
Check again the if the audio frames are changing when you make sounds. If not, 
you are probably connected to the wrong microphone/audio device and you need to 
change the parameter in the launch file.  

After everything works, you need to deploy your package. The process how to deploy
packages is described in detail in TIAGo's documentation. 

For Kraków's TIAGo, the procedure is described in: https://docs.pal-robotics.com/sdk/23.12/development/deploy-code.html


From Prague's TIAGo++, the procedure is described in: TODO - find the documentation for older TIAGo's system. 


## How is audio streamed? 

It sends audio frames using std_msgs.msg's Int16MultiArray, which is defined as follows: 

```
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
int16[]           data          # array of data
```

## How to use the streamed audio

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

def save_audio(audio_buffer: Queue, filename: str):
    if audio_buffer.empty():
        rospy.logwarn("No audio data to save.")
        return False

    # Combine all audio frames
    frames = list(audio_buffer.queue)
    audio = np.concatenate(frames)

    # Convert to bytes
    audio_bytes = audio.tobytes()

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

If you want to perform an action with the audio frames directly from the buffer without needing to save them, you can modify the code below: 

```python 

def audio   _part_function(audio_buffer: Queue, sample_rate: int, frame_duration: float):
    frame_duration # Frame duration in milliseconds, determines the duration of the audio sample you want to work on.   
    sample_rate # Audio sample rate.
    frame_size = int(sample_rate * frame_duration / 1000)  # Frame size in number of samples.
    frames = list(audio_buffer.queue) # Copy the Queue contents into a list. 
    audio = np.concatenate(frames) # Joins all the frames into one audio.
    frames = [audio[i:i + frame_size] for i in range(0, len(audio), frame_size)] # Separates into frames of your desired duration.
    for frame in frames:
        if len(frame) == frame_size:
            frame_as_bytes = frame.tobytes()
            # Perform some action with the frame, such as voice acitivity detection.
    return True # Add return False somewhere when your function fails for whatever reason. 

def full_audio_function(audio_buffer: Queue): 
    audio = np.concatenate(frames) # Joins all the frames into one audio.
    audio_as_bytes = audio.tobytes()
    # Perform some action with the full audio, such as automatic speech recognition.
    return True # Add return False somewhere when your function fails for whatever reason. 

```