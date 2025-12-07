---
sidebar_position: 2
title: 'Whisper Integration for Voice Commands'
---

# Whisper Integration for Voice Commands

This section covers the integration of OpenAI's Whisper speech recognition model for processing voice commands in robotics applications, enabling natural human-robot interaction through spoken language.

## Learning Objectives

After completing this section, you will be able to:
- Install and configure Whisper for robotics applications
- Process real-time audio input for voice command recognition
- Integrate Whisper with ROS 2 for seamless voice processing
- Handle various audio input sources and formats
- Implement robust voice command interpretation pipelines

## Introduction to Whisper for Robotics

Whisper is OpenAI's automatic speech recognition (ASR) system trained on a large dataset of diverse audio. For robotics applications, Whisper provides several advantages:

- **Multilingual Support**: Supports multiple languages for international applications
- **Robustness**: Performs well in various acoustic environments
- **Real-time Capability**: Can be optimized for real-time processing
- **Open Source**: Available for commercial use under MIT license
- **Customizable**: Can be fine-tuned for specific domains or vocabularies

### Whisper Model Variants

| Model | Size | Required VRAM | Relative Speed | English-only | Multilingual |
|-------|------|---------------|----------------|--------------|--------------|
| tiny  | 75 MB | ~1 GB | ~32x | ✓ | ✓ |
| base  | 145 MB | ~1 GB | ~16x | ✓ | ✓ |
| small | 445 MB | ~2 GB | ~6x | ✓ | ✓ |
| medium | 830 MB | ~5 GB | ~2x | ✓ | ✓ |
| large | 1.5 GB | ~10 GB | 1x | ✗ | ✓ |

For robotics applications, `small` or `medium` models typically provide the best balance of accuracy and resource usage.

## Installation and Setup

### Installing Whisper

```bash
# Install Whisper and related dependencies
pip install openai-whisper
pip install torch torchvision torchaudio  # For PyTorch support
pip install sounddevice pyaudio  # For audio input/output
pip install numpy scipy  # For signal processing
```

### Installing Additional Dependencies for Robotics

```bash
# ROS 2 dependencies
pip install ros2

# Audio processing
pip install pydub librosa

# For GPU acceleration (if available)
pip install onnxruntime-gpu  # For faster inference
```

### Basic Whisper Usage

```python
import whisper

# Load model (first time will download the model)
model = whisper.load_model("small")

# Transcribe audio file
result = model.transcribe("audio_file.wav")
print(result["text"])
```

## Real-time Audio Processing

### Audio Input Configuration

```python
import pyaudio
import numpy as np
import threading
import queue
import time

class AudioInputHandler:
    def __init__(self, sample_rate=16000, chunk_size=1024):
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.audio_queue = queue.Queue()
        self.is_recording = False
        self.audio = pyaudio.PyAudio()

        # Configure audio stream
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=sample_rate,
            input=True,
            frames_per_buffer=chunk_size
        )

    def start_recording(self):
        """Start recording audio in a separate thread"""
        self.is_recording = True
        self.recording_thread = threading.Thread(target=self._record_audio)
        self.recording_thread.start()

    def stop_recording(self):
        """Stop recording audio"""
        self.is_recording = False
        if hasattr(self, 'recording_thread'):
            self.recording_thread.join()
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()

    def _record_audio(self):
        """Internal method to record audio in a loop"""
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
                print(f"Audio recording error: {e}")
                time.sleep(0.01)  # Small delay to prevent busy waiting

    def get_audio_chunk(self):
        """Get the next audio chunk from the queue"""
        try:
            return self.audio_queue.get_nowait()
        except queue.Empty:
            return None
```

### Voice Activity Detection

```python
import webrtcvad
import collections

class VoiceActivityDetector:
    def __init__(self, sample_rate=16000, frame_duration_ms=30):
        self.vad = webrtcvad.Vad(2)  # Aggressiveness mode 2 (0-3)
        self.sample_rate = sample_rate
        self.frame_duration_ms = frame_duration_ms
        self.frame_size = int(sample_rate * frame_duration_ms / 1000)

        # Ring buffer to store recent audio chunks
        self.audio_buffer = collections.deque(maxlen=30)  # 30 chunks = ~900ms
        self.is_speaking = False
        self.speech_start_time = None
        self.silence_duration = 0

    def is_voice_present(self, audio_chunk):
        """Detect if voice is present in the audio chunk"""
        # Convert float32 audio to 16-bit PCM
        audio_16bit = (audio_chunk * 32767).astype(np.int16)

        # Pad if necessary to match frame size
        if len(audio_16bit) < self.frame_size:
            audio_16bit = np.pad(audio_16bit, (0, self.frame_size - len(audio_16bit)), 'constant')

        # Check voice activity in frames
        frames = self._frame_generator(self.frame_duration_ms, audio_16bit, self.sample_rate)
        voice_detected = False

        for frame in frames:
            if self.vad.is_speech(frame, self.sample_rate):
                voice_detected = True
                break

        return voice_detected

    def _frame_generator(self, frame_duration_ms, audio, sample_rate):
        """Generate audio frames from PCM audio data"""
        n = int(sample_rate * (frame_duration_ms / 1000.0) * 2)
        offset = 0
        while offset + n < len(audio):
            yield audio[offset:offset + n].tobytes()
            offset += n
```

## Whisper Integration with ROS 2

### ROS 2 Node for Voice Recognition

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
from builtin_interfaces.msg import Time
import whisper
import numpy as np
import threading
import queue

class WhisperROSNode(Node):
    def __init__(self):
        super().__init__('whisper_ros_node')

        # Initialize Whisper model
        self.get_logger().info("Loading Whisper model...")
        self.model = whisper.load_model("small")
        self.get_logger().info("Whisper model loaded successfully")

        # Publishers and subscribers
        self.voice_cmd_pub = self.create_publisher(String, '/voice_command', 10)
        self.audio_sub = self.create_subscription(
            AudioData, '/audio_input', self.audio_callback, 10
        )

        # Parameters
        self.declare_parameter('language', 'en')
        self.declare_parameter('energy_threshold', 400)
        self.declare_parameter('timeout', 5.0)
        self.declare_parameter('phrase_time_limit', 10.0)

        self.language = self.get_parameter('language').value
        self.energy_threshold = self.get_parameter('energy_threshold').value
        self.timeout = self.get_parameter('timeout').value
        self.phrase_time_limit = self.get_parameter('phrase_time_limit').value

        # Audio processing variables
        self.audio_queue = queue.Queue()
        self.is_listening = False

        # Start audio processing thread
        self.audio_thread = threading.Thread(target=self.process_audio)
        self.audio_thread.daemon = True
        self.audio_thread.start()

        self.get_logger().info("Whisper ROS node initialized")

    def audio_callback(self, msg):
        """Callback for audio input"""
        # Convert AudioData to numpy array
        audio_array = np.frombuffer(msg.data, dtype=np.int8).astype(np.float32) / 128.0
        self.audio_queue.put(audio_array)

    def process_audio(self):
        """Process audio chunks and perform speech recognition"""
        accumulated_audio = np.array([])
        last_audio_time = self.get_clock().now()

        while rclpy.ok():
            try:
                # Get audio chunk from queue
                if not self.audio_queue.empty():
                    audio_chunk = self.audio_queue.get_nowait()
                    accumulated_audio = np.concatenate([accumulated_audio, audio_chunk])
                    last_audio_time = self.get_clock().now()
                else:
                    # Check if we have accumulated enough audio or timeout occurred
                    current_time = self.get_clock().now()
                    time_since_last_audio = (current_time.nanoseconds - last_audio_time.nanoseconds) / 1e9

                    if len(accumulated_audio) > 0 and time_since_last_audio > 0.5:  # 500ms silence
                        # Process accumulated audio
                        if len(accumulated_audio) > 16000 * 0.5:  # At least 0.5 seconds of audio
                            self.recognize_speech(accumulated_audio)
                        accumulated_audio = np.array([])
                    elif time_since_last_audio > self.timeout:
                        # Reset if timeout exceeded
                        accumulated_audio = np.array([])
                        time.sleep(0.01)  # Small delay to prevent busy waiting
                    else:
                        time.sleep(0.01)  # Small delay to prevent busy waiting

            except queue.Empty:
                time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f"Audio processing error: {e}")
                time.sleep(0.01)

    def recognize_speech(self, audio_data):
        """Perform speech recognition on audio data"""
        try:
            # Convert to 16kHz if needed
            if len(audio_data) > 0:
                # Convert to format expected by Whisper (float32, -1 to 1 range)
                audio_16khz = self.resample_audio(audio_data, 16000)

                # Perform transcription
                result = self.model.transcribe(
                    audio_16khz,
                    language=self.language,
                    fp16=False  # Use fp32 for better compatibility
                )

                # Publish recognized text
                if result['text'].strip():
                    cmd_msg = String()
                    cmd_msg.data = result['text'].strip()
                    self.voice_cmd_pub.publish(cmd_msg)
                    self.get_logger().info(f"Recognized: {cmd_msg.data}")

        except Exception as e:
            self.get_logger().error(f"Speech recognition error: {e}")

    def resample_audio(self, audio_data, target_sr=16000):
        """Resample audio to target sample rate (simplified version)"""
        # In practice, you might want to use librosa or scipy for proper resampling
        # This is a simplified version that assumes 16kHz input
        return audio_data

def main(args=None):
    rclpy.init(args=args)
    node = WhisperROSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Whisper ROS node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Audio Input Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import AudioData
import pyaudio
import numpy as np
import threading

class AudioInputNode(Node):
    def __init__(self):
        super().__init__('audio_input_node')

        # Publisher for audio data
        self.audio_pub = self.create_publisher(AudioData, '/audio_input', 10)

        # Audio configuration
        self.sample_rate = 16000
        self.chunk_size = 1024
        self.channels = 1

        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()

        # Open audio stream
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )

        # Start audio capture thread
        self.is_capturing = True
        self.capture_thread = threading.Thread(target=self.capture_audio)
        self.capture_thread.daemon = True
        self.capture_thread.start()

        self.get_logger().info("Audio input node initialized")

    def capture_audio(self):
        """Capture audio and publish to ROS topic"""
        while self.is_capturing:
            try:
                # Read audio data
                audio_data = self.stream.read(self.chunk_size, exception_on_overflow=False)

                # Create AudioData message
                audio_msg = AudioData()
                audio_msg.data = audio_data

                # Publish audio data
                self.audio_pub.publish(audio_msg)

            except Exception as e:
                self.get_logger().error(f"Audio capture error: {e}")
                break

    def destroy_node(self):
        """Clean up audio resources"""
        self.is_capturing = False
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join()
        if hasattr(self, 'stream'):
            self.stream.stop_stream()
            self.stream.close()
        if hasattr(self, 'audio'):
            self.audio.terminate()
        super().destroy_node()
```

## Advanced Whisper Features for Robotics

### Keyword Spotting

```python
import whisper
import numpy as np
from transformers import pipeline

class KeywordSpotter:
    def __init__(self, keywords, model_size="small"):
        self.whisper_model = whisper.load_model(model_size)
        self.keywords = [kw.lower() for kw in keywords]
        self.activation_threshold = 0.3  # Confidence threshold for keyword activation

    def detect_keywords(self, audio_data):
        """Detect if any keywords are present in the audio"""
        try:
            # Transcribe the audio
            result = self.whisper_model.transcribe(audio_data, fp16=False)
            text = result['text'].lower()

            detected_keywords = []
            for keyword in self.keywords:
                if keyword in text:
                    detected_keywords.append(keyword)

            return detected_keywords, text

        except Exception as e:
            print(f"Keyword detection error: {e}")
            return [], ""

    def is_wake_word_present(self, audio_data, wake_words):
        """Check if wake word is present in audio"""
        detected_keywords, text = self.detect_keywords(audio_data)

        for wake_word in wake_words:
            if wake_word.lower() in detected_keywords:
                return True, text

        return False, text
```

### Multilingual Voice Commands

```python
import whisper
import langdetect

class MultilingualVoiceProcessor:
    def __init__(self):
        # Load multiple models for different languages or use multilingual model
        self.models = {
            'en': whisper.load_model("small.en"),
            'es': whisper.load_model("small"),  # Multilingual model
            'fr': whisper.load_model("small"),
            'de': whisper.load_model("small"),
            'ja': whisper.load_model("small")
        }

        # Fallback multilingual model
        self.multilingual_model = whisper.load_model("medium")

    def detect_language(self, audio_data):
        """Detect the language of the spoken audio"""
        try:
            # Transcribe with multilingual model first
            result = self.multilingual_model.transcribe(audio_data, fp16=False)

            # Use langdetect to identify language from text
            detected_lang = langdetect.detect(result['text'])
            return detected_lang

        except:
            # If detection fails, return default
            return 'en'

    def transcribe_multilingual(self, audio_data):
        """Transcribe audio in the detected language"""
        detected_lang = self.detect_language(audio_data)

        # Map detected language to available models
        lang_code = detected_lang.split('-')[0] if '-' in detected_lang else detected_lang

        if lang_code in self.models:
            model = self.models[lang_code]
            result = model.transcribe(audio_data, language=lang_code, fp16=False)
        else:
            # Use multilingual model with detected language
            result = self.multilingual_model.transcribe(
                audio_data,
                language=detected_lang if detected_lang in ['en', 'zh', 'de', 'es', 'ru', 'ko', 'fr', 'ja', 'pt', 'tr', 'pl', 'ca', 'nl', 'ar', 'sv', 'it', 'id', 'hi', 'fi', 'vi', 'he', 'uk', 'el', 'ms', 'cs', 'ro', 'da', 'hu', 'ta', 'no', 'th', 'ur', 'hr', 'bg', 'lt', 'la', 'mi', 'ml', 'cy', 'sk', 'te', 'fa', 'lv', 'bn', 'sr', 'az', 'sl', 'kn', 'et', 'mk', 'br', 'eu', 'is', 'hy', 'ne', 'mn', 'bs', 'kk', 'sq', 'sw', 'gl', 'mr', 'pa', 'si', 'km', 'sn', 'yo', 'so', 'af', 'oc', 'ka', 'be', 'tg', 'sd', 'gu', 'am', 'yi', 'lo', 'uz', 'fo', 'ht', 'ps', 'tk', 'nn', 'mt', 'sa', 'lb', 'my', 'bo', 'tl', 'mg', 'as', 'tt', 'haw', 'ln', 'ha', 'ba', 'jw', 'su'],
                fp16=False
            )

        return result['text'], detected_lang
```

## Performance Optimization

### GPU Acceleration

```python
import whisper
import torch

class OptimizedWhisperProcessor:
    def __init__(self, model_size="small"):
        # Check if CUDA is available
        if torch.cuda.is_available():
            self.device = "cuda"
            self.get_logger().info("Using GPU for Whisper processing")
        else:
            self.device = "cpu"
            self.get_logger().info("Using CPU for Whisper processing")

        # Load model to specified device
        self.model = whisper.load_model(model_size).to(self.device)

        # Set compute type based on device
        if self.device == "cuda":
            self.model = self.model.half()  # Use half precision for GPU

    def transcribe_with_gpu(self, audio_data):
        """Transcribe audio using GPU acceleration"""
        try:
            # Move audio data to GPU if available
            if self.device == "cuda":
                # Convert to tensor and move to GPU
                audio_tensor = torch.from_numpy(audio_data).to(self.device)
            else:
                audio_tensor = torch.from_numpy(audio_data)

            # Perform transcription
            result = self.model.transcribe(audio_tensor.cpu().numpy() if self.device == "cuda" else audio_tensor.numpy())

            return result['text']

        except Exception as e:
            print(f"GPU transcription error: {e}")
            # Fallback to CPU
            result = self.model.transcribe(audio_data)
            return result['text']
```

### Audio Preprocessing for Better Recognition

```python
import numpy as np
from scipy import signal
import librosa

class AudioPreprocessor:
    def __init__(self):
        self.sample_rate = 16000
        self.noise_threshold = 0.01  # Threshold for noise detection

    def preprocess_audio(self, audio_data, target_sr=16000):
        """Preprocess audio for better Whisper recognition"""
        # Convert to float32 if needed
        if audio_data.dtype != np.float32:
            audio_data = audio_data.astype(np.float32)

        # Normalize audio
        audio_data = audio_data / np.max(np.abs(audio_data)) if np.max(np.abs(audio_data)) != 0 else audio_data

        # Apply noise reduction (simplified version)
        audio_data = self.reduce_noise_simple(audio_data)

        # Apply pre-emphasis filter to boost high frequencies
        audio_data = self.pre_emphasis_filter(audio_data)

        # Resample if needed (simplified - in practice use librosa)
        if target_sr != 16000:  # Assuming input is at 16kHz
            audio_data = librosa.resample(audio_data, orig_sr=16000, target_sr=target_sr)

        return audio_data

    def reduce_noise_simple(self, audio_data):
        """Simple noise reduction by spectral gating"""
        # Calculate the mean of the absolute values to estimate noise level
        noise_level = np.mean(np.abs(audio_data)) * 0.1  # 10% of average amplitude

        # Apply simple noise gating
        mask = np.abs(audio_data) > noise_level
        return audio_data * mask.astype(np.float32)

    def pre_emphasis_filter(self, audio_data, coeff=0.97):
        """Apply pre-emphasis filter to boost high frequencies"""
        return np.append(audio_data[0], audio_data[1:] - coeff * audio_data[:-1])
```

## Lab Activity: Voice Command System

Create a complete voice command processing system with:
1. Real-time audio capture and processing
2. Whisper-based speech recognition
3. ROS 2 integration for robot command execution
4. Keyword spotting for wake word activation
5. Multilingual support

### Steps:
1. Set up audio input system with proper configuration
2. Install and configure Whisper model
3. Create ROS 2 nodes for audio processing and speech recognition
4. Implement keyword spotting for robot activation
5. Add multilingual support for international commands
6. Test system with various voice commands

### Expected Outcome:
- Functional voice command recognition system
- Real-time audio processing with Whisper
- ROS 2 integration for robot control
- Wake word activation working
- Multilingual command support
- Lab activity completed successfully

## Best Practices

1. **Audio Quality**: Ensure high-quality audio input for best recognition results
2. **Model Selection**: Choose appropriate Whisper model size for your hardware constraints
3. **Real-time Processing**: Optimize for real-time performance with proper buffering
4. **Error Handling**: Implement robust error handling for audio and recognition failures
5. **Privacy**: Consider privacy implications of voice data processing
6. **Testing**: Test with various accents, speaking styles, and environmental conditions

## Checklist

- [ ] Whisper installation and configuration completed
- [ ] Real-time audio processing implemented
- [ ] ROS 2 integration established
- [ ] Keyword spotting functionality working
- [ ] Multilingual support added
- [ ] Performance optimization applied
- [ ] Lab activity completed successfully

## Next Steps

In the next section, we'll explore using Large Language Models (LLMs) for reasoning and generating ROS 2 actions based on voice commands.