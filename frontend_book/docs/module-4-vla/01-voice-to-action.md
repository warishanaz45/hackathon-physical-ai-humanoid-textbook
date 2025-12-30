---
sidebar_position: 1
title: "Voice-to-Action with Speech Recognition"
description: "Implement voice control for humanoid robots using OpenAI Whisper and ROS 2"
keywords: [voice, speech, whisper, openai, ros2, robotics, vla, humanoid]
---

# Voice-to-Action with Speech Recognition

## What You'll Learn

- Understand the Vision-Language-Action (VLA) pipeline architecture
- Set up audio capture with voice activity detection (VAD)
- Install and configure OpenAI Whisper for speech-to-text
- Implement real-time speech transcription
- Publish voice commands to ROS 2 topics
- Handle speech recognition errors gracefully

:::info Prerequisites
This chapter builds on concepts from **Modules 1-3**:
- ROS 2 topics and publishers (Module 1)
- Simulation environment setup (Module 2)
- Navigation stack from Nav2 (Module 3)

**Hardware Required**: USB microphone, internet connection (for initial setup)
:::

## Introduction to Voice-Controlled Robotics

Voice control represents the most natural interface for human-robot interaction. Instead of programming specific movements or using joysticks, users can simply speak commands like *"Go to the kitchen"* or *"Pick up the red block"*.

The **Vision-Language-Action (VLA)** pipeline connects three key components:

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│   Voice      │────▶│   Language   │────▶│   Action     │
│   (Whisper)  │     │   (LLM)      │     │   (ROS 2)    │
└──────────────┘     └──────────────┘     └──────────────┘
     Audio              Text Plan           Robot Motion
```

In this chapter, we focus on the first component: capturing voice and converting it to text that downstream systems can process.

### Why Voice for Humanoids?

Humanoid robots are designed to operate in human environments. Voice commands allow:

- **Natural interaction** without specialized controllers
- **Hands-free operation** while the robot performs tasks
- **Accessibility** for users with mobility limitations
- **Multi-step instructions** in natural language

## Speech Recognition Fundamentals

Speech-to-text (STT) systems convert audio waveforms into text. Modern approaches use deep learning models trained on thousands of hours of speech data.

### How Speech Recognition Works

1. **Audio Capture**: Microphone records sound waves as digital samples
2. **Preprocessing**: Noise reduction, normalization, feature extraction
3. **Model Inference**: Neural network predicts text from audio features
4. **Post-processing**: Language model refines transcription

### Cloud vs Local Processing

| Approach | Pros | Cons |
|----------|------|------|
| **Cloud APIs** (Google, Azure) | High accuracy, no GPU needed | Latency, internet required, costs |
| **Local Models** (Whisper) | Offline capable, free, private | GPU recommended for speed |

For robotics, we use **OpenAI Whisper** locally because:
- No internet dependency during operation
- Lower latency for real-time commands
- Privacy (audio stays on device)
- Free and open-source

## Setting Up Audio Capture

Before we can transcribe speech, we need to capture audio from a microphone.

### Hardware Requirements

- **USB Microphone**: Any standard USB mic works
- **Recommended**: Directional microphone for noisy environments
- **Sample Rate**: 16kHz (optimal for Whisper)

### Installing PyAudio

PyAudio provides Python bindings for audio I/O:

```bash title="Install PyAudio on Ubuntu"
sudo apt-get install python3-pyaudio portaudio19-dev
pip install pyaudio webrtcvad
```

### Voice Activity Detection (VAD)

We don't want to transcribe silence or background noise. Voice Activity Detection (VAD) identifies when someone is speaking.

**webrtcvad** is a lightweight VAD library that filters audio:

```python title="audio_capture.py"
import pyaudio
import webrtcvad
import collections
import numpy as np

class AudioCapture:
    """Captures audio with voice activity detection."""

    def __init__(self, sample_rate=16000, frame_duration_ms=30):
        self.sample_rate = sample_rate
        self.frame_duration_ms = frame_duration_ms
        self.frame_size = int(sample_rate * frame_duration_ms / 1000)

        # VAD aggressiveness: 0-3 (3 = most aggressive filtering)
        self.vad = webrtcvad.Vad(2)

        # Audio stream
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=sample_rate,
            input=True,
            frames_per_buffer=self.frame_size
        )

    def capture_utterance(self, max_silence_ms=1000):
        """Capture audio until speech ends."""
        frames = []
        silence_frames = 0
        max_silence_frames = max_silence_ms // self.frame_duration_ms
        speaking = False

        print("Listening...")

        while True:
            frame = self.stream.read(self.frame_size)
            is_speech = self.vad.is_speech(frame, self.sample_rate)

            if is_speech:
                if not speaking:
                    print("Speech detected...")
                speaking = True
                silence_frames = 0
                frames.append(frame)
            elif speaking:
                frames.append(frame)
                silence_frames += 1
                if silence_frames > max_silence_frames:
                    break

        audio_data = b''.join(frames)
        duration = len(frames) * self.frame_duration_ms / 1000
        print(f"Audio captured ({duration:.1f} seconds)")

        return np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

    def close(self):
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()

# Usage
if __name__ == "__main__":
    capture = AudioCapture()
    try:
        audio = capture.capture_utterance()
        print(f"Captured {len(audio)} samples")
    finally:
        capture.close()
```

**Expected Output**:
```
Listening...
Speech detected...
Audio captured (2.3 seconds)
Captured 36800 samples
```

## Implementing Whisper Transcription

OpenAI Whisper is a state-of-the-art speech recognition model trained on 680,000 hours of multilingual data.

### Installing Whisper

```bash title="Install Whisper"
pip install openai-whisper
```

### Model Selection

Whisper offers multiple model sizes:

| Model | Parameters | Speed | Accuracy | VRAM |
|-------|------------|-------|----------|------|
| tiny | 39M | Real-time | ~80% | 1GB |
| base | 74M | Fast | ~85% | 1GB |
| small | 244M | Moderate | ~90% | 2GB |
| medium | 769M | Slow | ~93% | 5GB |
| large-v3 | 1.5B | Very slow | ~95% | 10GB |

For robotics commands, **small** or **base** models provide good accuracy with reasonable speed.

### ROS 2 Whisper Node

Now let's wrap Whisper in a ROS 2 node that publishes transcriptions:

```python title="whisper_node.py"
#!/usr/bin/env python3
"""ROS 2 node for speech-to-text using Whisper."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import numpy as np
import pyaudio
import webrtcvad

class WhisperNode(Node):
    """Transcribes speech and publishes to /voice_command topic."""

    def __init__(self):
        super().__init__('whisper_node')

        # Parameters
        self.declare_parameter('model_size', 'base')
        self.declare_parameter('sample_rate', 16000)

        model_size = self.get_parameter('model_size').value
        self.sample_rate = self.get_parameter('sample_rate').value

        # Load Whisper model
        self.get_logger().info(f'Loading Whisper model: {model_size}')
        self.model = whisper.load_model(model_size)
        self.get_logger().info('Whisper model loaded')

        # Publisher
        self.publisher = self.create_publisher(String, '/voice_command', 10)

        # Audio setup
        self.vad = webrtcvad.Vad(2)
        self.frame_duration_ms = 30
        self.frame_size = int(self.sample_rate * self.frame_duration_ms / 1000)

        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.frame_size
        )

        # Timer for continuous listening
        self.timer = self.create_timer(0.1, self.listen_callback)
        self.is_listening = True

        self.get_logger().info('Whisper node ready. Listening for commands...')

    def capture_audio(self):
        """Capture audio until speech ends."""
        frames = []
        silence_frames = 0
        max_silence_frames = 1000 // self.frame_duration_ms
        speaking = False

        while True:
            frame = self.stream.read(self.frame_size, exception_on_overflow=False)
            is_speech = self.vad.is_speech(frame, self.sample_rate)

            if is_speech:
                speaking = True
                silence_frames = 0
                frames.append(frame)
            elif speaking:
                frames.append(frame)
                silence_frames += 1
                if silence_frames > max_silence_frames:
                    break

            if not speaking and len(frames) == 0:
                return None

        audio_data = b''.join(frames)
        return np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

    def listen_callback(self):
        """Timer callback to check for speech."""
        if not self.is_listening:
            return

        # Check if speech is starting
        frame = self.stream.read(self.frame_size, exception_on_overflow=False)
        if self.vad.is_speech(frame, self.sample_rate):
            self.get_logger().info('Speech detected, capturing...')

            # Capture full utterance
            audio = self.capture_audio()

            if audio is not None and len(audio) > 0:
                # Transcribe
                result = self.model.transcribe(audio, language='en')
                text = result['text'].strip()

                if text:
                    self.get_logger().info(f"Transcribed: '{text}'")

                    # Publish
                    msg = String()
                    msg.data = text
                    self.publisher.publish(msg)

    def destroy_node(self):
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WhisperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected Output**:
```
[INFO] [whisper_node]: Loading Whisper model: base
[INFO] [whisper_node]: Whisper model loaded
[INFO] [whisper_node]: Whisper node ready. Listening for commands...
[INFO] [whisper_node]: Speech detected, capturing...
[INFO] [whisper_node]: Transcribed: 'go to the kitchen'
```

## Publishing to ROS 2

The node above publishes to `/voice_command`. Let's create a complete example that includes confidence tracking:

```python title="voice_command_publisher.py"
#!/usr/bin/env python3
"""Complete voice command capture and publish node."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from builtin_interfaces.msg import Time
import whisper
import numpy as np
import pyaudio
import webrtcvad
import json

class VoiceCommandPublisher(Node):
    """Captures voice, transcribes with Whisper, publishes to ROS 2."""

    def __init__(self):
        super().__init__('voice_command_publisher')

        # Parameters
        self.declare_parameter('model_size', 'base')
        self.declare_parameter('confidence_threshold', 0.7)

        model_size = self.get_parameter('model_size').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value

        # Load model
        self.get_logger().info(f'Loading Whisper {model_size} model...')
        self.model = whisper.load_model(model_size)

        # Publishers
        self.cmd_publisher = self.create_publisher(String, '/voice_command', 10)
        self.raw_publisher = self.create_publisher(String, '/voice_command_raw', 10)

        # Audio setup
        self.sample_rate = 16000
        self.vad = webrtcvad.Vad(2)
        self.frame_ms = 30
        self.frame_size = int(self.sample_rate * self.frame_ms / 1000)

        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.frame_size
        )

        # Start listening loop
        self.timer = self.create_timer(0.05, self.process_audio)
        self.get_logger().info('Voice command publisher ready')

    def capture_speech(self):
        """Capture audio while speech is detected."""
        frames = []
        silence_count = 0
        max_silence = 33  # ~1 second at 30ms frames

        while True:
            frame = self.stream.read(self.frame_size, exception_on_overflow=False)
            is_speech = self.vad.is_speech(frame, self.sample_rate)

            frames.append(frame)

            if is_speech:
                silence_count = 0
            else:
                silence_count += 1
                if silence_count > max_silence and len(frames) > 10:
                    break

        audio = b''.join(frames)
        return np.frombuffer(audio, dtype=np.int16).astype(np.float32) / 32768.0

    def process_audio(self):
        """Check for speech and process."""
        try:
            frame = self.stream.read(self.frame_size, exception_on_overflow=False)
        except IOError:
            return

        if self.vad.is_speech(frame, self.sample_rate):
            self.get_logger().info('Capturing speech...')

            audio = self.capture_speech()

            # Transcribe with Whisper
            result = self.model.transcribe(
                audio,
                language='en',
                task='transcribe'
            )

            text = result['text'].strip().lower()

            # Calculate rough confidence from no_speech_prob
            segments = result.get('segments', [])
            if segments:
                avg_no_speech = np.mean([s.get('no_speech_prob', 0) for s in segments])
                confidence = 1.0 - avg_no_speech
            else:
                confidence = 0.5

            # Publish raw result
            raw_msg = String()
            raw_msg.data = json.dumps({
                'text': text,
                'confidence': confidence,
                'language': result.get('language', 'en')
            })
            self.raw_publisher.publish(raw_msg)

            # Publish command if confident enough
            if confidence >= self.confidence_threshold and text:
                self.get_logger().info(
                    f"Published: '{text}' (confidence: {confidence:.2f})"
                )
                msg = String()
                msg.data = text
                self.cmd_publisher.publish(msg)
            else:
                self.get_logger().warn(
                    f"Low confidence ({confidence:.2f}): '{text}'"
                )

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected Output**:
```
[INFO] [voice_command_publisher]: Loading Whisper base model...
[INFO] [voice_command_publisher]: Voice command publisher ready
[INFO] [voice_command_publisher]: Capturing speech...
[INFO] [voice_command_publisher]: Published: 'pick up the red block' (confidence: 0.94)
```

### Testing the Publisher

In a separate terminal, subscribe to the topic:

```bash
ros2 topic echo /voice_command
```

You should see messages appear when you speak commands.

## Key Takeaways

1. **VLA Pipeline**: Voice → Language (LLM) → Action forms the complete voice control system

2. **Voice Activity Detection**: Use VAD to filter silence and capture only speech segments

3. **Whisper Models**: Choose model size based on accuracy vs. speed tradeoff:
   - `tiny`/`base`: Fast, good for short commands
   - `small`: Best balance for robotics
   - `medium`/`large`: Maximum accuracy, slower

4. **ROS 2 Integration**: Publish transcriptions to topics for downstream processing

5. **Confidence Filtering**: Track transcription confidence to avoid acting on unclear speech

6. **Error Handling**: Handle audio overflow, unclear speech, and microphone issues gracefully

---

**Next Chapter**: [LLM-Based Cognitive Planning](./02-llm-cognitive-planning.md) - Convert voice commands into structured robot action plans using Large Language Models.
