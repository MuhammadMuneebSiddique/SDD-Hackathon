---
sidebar_position: 2
title: 'Voice Command Processing'
---

# Voice Command Processing

This section covers the implementation of voice command processing for the capstone project, enabling the humanoid robot to understand and respond to natural language commands.

## Learning Objectives

After completing this section, you will be able to:
- Implement voice command recognition using Whisper
- Process natural language commands with LLMs
- Integrate voice commands with robot action planning
- Handle various voice command scenarios and edge cases

## Voice Command Architecture

The voice command system consists of several key components:

1. **Audio Input**: Capturing voice commands from the environment
2. **Speech Recognition**: Converting speech to text using Whisper
3. **Language Understanding**: Interpreting commands with LLMs
4. **Action Planning**: Converting commands to executable robot actions
5. **Execution**: Performing the planned actions

## Implementation

### Audio Capture and Preprocessing

```python
import pyaudio
import numpy as np
import threading
import queue
import time

class AudioCapture:
    def __init__(self, sample_rate=16000, chunk_size=1024):
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.audio_queue = queue.Queue()
        self.is_recording = False

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Configure audio stream
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=sample_rate,
            input=True,
            frames_per_buffer=chunk_size
        )

    def start_capture(self):
        """Start capturing audio in a separate thread"""
        self.is_recording = True
        self.capture_thread = threading.Thread(target=self._capture_audio)
        self.capture_thread.daemon = True
        self.capture_thread.start()

    def stop_capture(self):
        """Stop capturing audio"""
        self.is_recording = False
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join()
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()

    def _capture_audio(self):
        """Internal method to capture audio continuously"""
        while self.is_recording:
            try:
                # Read audio data from stream
                audio_data = self.stream.read(self.chunk_size, exception_on_overflow=False)
                audio_array = np.frombuffer(audio_data, dtype=np.int16)

                # Normalize to float32 in range [-1, 1]
                audio_float = audio_array.astype(np.float32) / 32768.0

                # Add to queue
                self.audio_queue.put(audio_float)
            except Exception as e:
                print(f"Audio capture error: {e}")
                time.sleep(0.01)  # Small delay to prevent busy waiting

    def get_audio_chunk(self):
        """Get the next audio chunk from the queue"""
        try:
            return self.audio_queue.get_nowait()
        except queue.Empty:
            return None
```

### Speech Recognition with Whisper

```python
import whisper
import numpy as np
from typing import Optional

class WhisperRecognizer:
    def __init__(self, model_size="small"):
        self.model = whisper.load_model(model_size)
        self.sample_rate = 16000

    def transcribe_audio(self, audio_chunk: np.ndarray) -> Optional[str]:
        """Transcribe audio chunk to text"""
        try:
            # Ensure audio is at correct sample rate and format
            if len(audio_chunk) > 0:
                # Transcribe the audio
                result = self.model.transcribe(
                    audio_chunk,
                    language="en",
                    fp16=False  # Use fp32 for better compatibility
                )

                return result["text"].strip()
        except Exception as e:
            print(f"Transcription error: {e}")
            return None

    def transcribe_with_timestamps(self, audio_chunk: np.ndarray):
        """Transcribe audio with timestamps for better processing"""
        try:
            result = self.model.transcribe(
                audio_chunk,
                language="en",
                word_timestamps=True
            )
            return result
        except Exception as e:
            print(f"Timestamped transcription error: {e}")
            return None
```

### Natural Language Command Processing

```python
import openai
import json
from typing import Dict, Any

class CommandProcessor:
    def __init__(self, openai_api_key: str):
        self.client = openai.OpenAI(api_key=openai_api_key)

    def process_command(self, command_text: str) -> Dict[str, Any]:
        """Process natural language command and return structured action"""
        prompt = f"""
        You are a command interpreter for a humanoid robot. Parse the following command and return a structured JSON object.

        Command: "{command_text}"

        The command should be interpreted to perform one of these actions:
        - Navigation: Moving to a location
        - Object manipulation: Picking up, placing, or interacting with objects
        - Interaction: Speaking, gesturing, or responding to humans
        - Complex tasks: Multi-step actions combining navigation and manipulation

        Return a JSON object with the following structure:
        {{
            "action_type": "navigation|manipulation|interaction|complex",
            "target_object": "name of target object if applicable",
            "target_location": "name of target location if applicable",
            "action_sequence": [
                {{
                    "action": "specific action",
                    "parameters": {{"param_name": "param_value"}}
                }}
            ],
            "priority": "high|medium|low",
            "estimated_duration": "seconds"
        }}

        If the command is unclear or invalid, return:
        {{
            "action_type": "invalid",
            "error": "description of why command is invalid",
            "suggestions": ["possible alternative commands"]
        }}
        """

        try:
            response = self.client.chat.completions.create(
                model="gpt-4-turbo",
                messages=[
                    {
                        "role": "system",
                        "content": "You are a command interpreter for a humanoid robot. Always return valid JSON."
                    },
                    {
                        "role": "user",
                        "content": prompt
                    }
                ],
                temperature=0.1,
                response_format={"type": "json_object"}
            )

            return json.loads(response.choices[0].message.content)

        except Exception as e:
            print(f"Command processing error: {e}")
            return {
                "action_type": "invalid",
                "error": f"Processing error: {str(e)}",
                "suggestions": ["Please repeat the command", "Try a simpler command"]
            }
```

### Voice Command Integration

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import AudioData
import threading
import time

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Initialize components
        self.audio_capture = AudioCapture()
        self.whisper_recognizer = WhisperRecognizer()
        self.command_processor = CommandProcessor(openai_api_key="your-api-key")

        # Publishers and subscribers
        self.command_pub = self.create_publisher(String, '/processed_command', 10)
        self.status_pub = self.create_publisher(String, '/voice_status', 10)
        self.listening_pub = self.create_publisher(Bool, '/is_listening', 10)

        # Parameters
        self.declare_parameter('activation_phrase', 'Hey Robot')
        self.declare_parameter('silence_threshold', 0.5)
        self.declare_parameter('command_timeout', 5.0)

        self.activation_phrase = self.get_parameter('activation_phrase').value
        self.silence_threshold = self.get_parameter('silence_threshold').value
        self.command_timeout = self.get_parameter('command_timeout').value

        # State variables
        self.is_listening = False
        self.command_buffer = []
        self.listening_timer = None

        # Start audio capture
        self.audio_capture.start_capture()

        # Start processing thread
        self.processing_thread = threading.Thread(target=self.process_audio_continuously)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.get_logger().info("Voice Command Node initialized")

    def process_audio_continuously(self):
        """Continuously process audio for voice commands"""
        accumulated_audio = np.array([])
        last_activity_time = time.time()
        activation_detected = False

        while rclpy.ok():
            try:
                # Get audio chunk
                audio_chunk = self.audio_capture.get_audio_chunk()

                if audio_chunk is not None:
                    # Check for activation phrase
                    if not activation_detected:
                        # Simple energy-based detection for activation phrase
                        energy = np.mean(np.abs(audio_chunk))

                        if energy > self.silence_threshold:
                            # Accumulate audio for potential activation phrase
                            accumulated_audio = np.concatenate([accumulated_audio, audio_chunk])

                            if len(accumulated_audio) > self.sample_rate * 2:  # 2 seconds of audio
                                # Transcribe to check for activation phrase
                                activation_text = self.whisper_recognizer.transcribe_audio(accumulated_audio)

                                if activation_text and self.activation_phrase.lower() in activation_text.lower():
                                    activation_detected = True
                                    self.start_listening_for_command()
                                    accumulated_audio = np.array([])
                                else:
                                    # Keep some audio for continuity
                                    accumulated_audio = accumulated_audio[-int(self.sample_rate * 0.5):]  # Keep last 0.5 seconds
                    else:
                        # Collect command audio
                        accumulated_audio = np.concatenate([accumulated_audio, audio_chunk])
                        last_activity_time = time.time()

                        # Check if we have enough audio for transcription
                        if len(accumulated_audio) > self.sample_rate * 1:  # At least 1 second
                            # Check for silence to determine end of command
                            recent_energy = np.mean(np.abs(accumulated_audio[-int(self.sample_rate * 0.5):]))

                            if recent_energy < self.silence_threshold * 0.5:  # Low energy indicates pause
                                if len(accumulated_audio) > self.sample_rate * 0.5:  # At least 0.5 seconds of command
                                    self.process_command_audio(accumulated_audio)
                                    activation_detected = False
                                    accumulated_audio = np.array([])
                                    self.stop_listening()
                                else:
                                    # Not enough audio, continue collecting
                                    pass
                            else:
                                # Still active, continue collecting (but limit buffer size)
                                if len(accumulated_audio) > self.sample_rate * 10:  # 10 seconds max
                                    self.process_command_audio(accumulated_audio)
                                    activation_detected = False
                                    accumulated_audio = np.array([])
                                    self.stop_listening()

                # Check for timeout
                if activation_detected and (time.time() - last_activity_time) > self.command_timeout:
                    if len(accumulated_audio) > self.sample_rate * 0.5:  # At least 0.5 seconds
                        self.process_command_audio(accumulated_audio)

                    activation_detected = False
                    accumulated_audio = np.array([])
                    self.stop_listening()

                time.sleep(0.01)  # Small delay to prevent busy waiting

            except Exception as e:
                self.get_logger().error(f"Audio processing error: {e}")
                time.sleep(0.1)

    def start_listening_for_command(self):
        """Start listening for a command after activation phrase"""
        self.is_listening = True
        listening_msg = Bool()
        listening_msg.data = True
        self.listening_pub.publish(listening_msg)

        status_msg = String()
        status_msg.data = "Listening for command"
        self.status_pub.publish(status_msg)

    def stop_listening(self):
        """Stop listening after command"""
        self.is_listening = False
        listening_msg = Bool()
        listening_msg.data = False
        self.listening_pub.publish(listening_msg)

        status_msg = String()
        status_msg.data = "Stopped listening"
        self.status_pub.publish(status_msg)

    def process_command_audio(self, audio_data):
        """Process collected command audio"""
        try:
            # Transcribe the command
            command_text = self.whisper_recognizer.transcribe_audio(audio_data)

            if command_text and command_text.strip():
                self.get_logger().info(f"Heard command: {command_text}")

                # Process the command with LLM
                command_structure = self.command_processor.process_command(command_text)

                # Publish the processed command
                command_msg = String()
                command_msg.data = json.dumps(command_structure)
                self.command_pub.publish(command_msg)

                self.get_logger().info(f"Processed command: {command_structure.get('action_type', 'unknown')}")
            else:
                self.get_logger().info("No command detected in audio")

        except Exception as e:
            self.get_logger().error(f"Command processing error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Voice Command Node...")
    finally:
        node.audio_capture.stop_capture()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices

1. **Audio Quality**: Ensure high-quality audio input for accurate recognition
2. **Activation Phrases**: Use distinctive activation phrases to avoid false triggers
3. **Error Handling**: Implement robust error handling for recognition failures
4. **Privacy**: Consider privacy implications of voice data processing
5. **Testing**: Test with various accents, speaking styles, and environmental conditions

## Integration with Robot Systems

The voice command system integrates with the overall robot architecture by:
- Publishing processed commands to the action planning system
- Providing real-time feedback through status messages
- Supporting multimodal interaction when combined with vision
- Enabling natural human-robot interaction

This voice command processing system forms the foundation for natural interaction with the humanoid robot in the capstone project, allowing users to communicate with the robot using natural language commands.