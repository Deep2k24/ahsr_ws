#!/usr/bin/env python3
"""Listen with Vosk. Wake up, buffer the complete sentence, and converse continuously."""

import json
import os
import re
import subprocess
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    import pyaudio
except ImportError as e:
    pyaudio = None
    _PYAUDIO_IMPORT_ERROR = e

try:
    import vosk
except ImportError as e:
    vosk = None
    _VOSK_IMPORT_ERROR = e

try:
    from groq import Groq
except ImportError as e:
    Groq = None
    _GROQ_IMPORT_ERROR = e

# ==========================================
# 🛑 PASTE YOUR GROQ API KEY HERE 🛑
GROQ_API_KEY = "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"
# ==========================================

SYSTEM_PROMPT = (
    "You are Techno, an advanced Autonomous Hospital Service Robot (AHSR) designed by CS Brushtech and engineered by Deep, Atharva, Alok. "
    "Your core mission is to assist nurses and doctors, transport medical supplies, and politely guide patients through the facility. "
    "You are currently operating in the physical world on a wheeled mobile base. "
    "TONE: Friendly, highly professional, reassuring, and concise. "
    "STRICT CONSTRAINTS FOR YOUR OUTPUT: "
    "1. You are speaking through a Text-to-Speech voice engine. You MUST NOT use any emojis, asterisks, hash symbols, or markdown formatting whatsoever. "
    "2. Keep EVERY answer strictly under 3 short sentences. Speak conversationally. "
    "3. Never provide medical advice or diagnoses; always politely refer medical questions to human doctors or nurses. "
    "4. If asked about your status, you are fully operational and ready to navigate."
)

def _normalize_phrase(text: str) -> str:
    text = text.lower()
    text = re.sub(r'[^a-z0-9\s]', ' ', text)
    return ' '.join(text.split())

def _resample_pcm16_mono(data: bytes, orig_rate: int, target_rate: int) -> bytes:
    if orig_rate == target_rate:
        return data
    y = np.frombuffer(data, dtype=np.int16).astype(np.float32)
    if len(y) < 2:
        return data
    n_out = max(2, int(len(y) * target_rate / orig_rate))
    x_old = np.linspace(0.0, 1.0, len(y), endpoint=False)
    x_new = np.linspace(0.0, 1.0, n_out, endpoint=False)
    y_out = np.interp(x_new, x_old, y).astype(np.int16)
    return y_out.tobytes()

class TechnoVoiceNode(Node):
    def __init__(self) -> None:
        super().__init__('techno_voice_node')

        if pyaudio is None:
            raise RuntimeError('PyAudio is required: sudo apt install python3-pyaudio')
        if vosk is None:
            raise RuntimeError('Vosk is required: pip3 install vosk')
        if Groq is None:
            raise RuntimeError('Groq is required: pip3 install groq')

        self.llm_client = Groq(api_key=GROQ_API_KEY)

        self.declare_parameter('model_path', '')
        self.declare_parameter('vosk_sample_rate', 16000)
        self.declare_parameter('audio_device_index', -1)
        self.declare_parameter('speak_cooldown_sec', 3.0) 
        self.declare_parameter('tts_executable', 'espeak-ng')
        self.declare_parameter('tts_speed', 160)

        self._target_rate = int(self.get_parameter('vosk_sample_rate').value)
        self._cooldown = float(self.get_parameter('speak_cooldown_sec').value)
        self._tts_executable = self.get_parameter('tts_executable').get_parameter_value().string_value
        self._tts_speed = int(self.get_parameter('tts_speed').value)
        dev = self.get_parameter('audio_device_index').value
        self._device_index = None if int(dev) < 0 else int(dev)

        model_path = self._resolve_model_path(
            self.get_parameter('model_path').get_parameter_value().string_value.strip()
        )
        self.get_logger().info(f'Loading Vosk model from: {model_path}')
        self._model = vosk.Model(model_path)
        self._recognizer = vosk.KaldiRecognizer(self._model, self._target_rate)

        self._last_speak_time = 0.0
        self._state_lock = threading.Lock()
        self._speaking = False

        self._pa: pyaudio.PyAudio | None = None
        self._stream = None
        self._input_rate = self._target_rate
        self._audio_thread: threading.Thread | None = None
        self._shutdown = threading.Event()

        self._open_audio_stream()
        self._audio_thread = threading.Thread(target=self._audio_loop, daemon=True)
        self._audio_thread.start()

        self.get_logger().info('🤖 AI Techno voice ready! Say "Hi" or "Techno" to wake him up.')

    def _resolve_model_path(self, declared: str) -> str:
        candidates = []
        if declared:
            candidates.append(os.path.expanduser(declared))
        candidates.extend([
            os.path.expanduser('~/vosk-model-small-en-in-0.4'),
            os.path.expanduser('~/vosk-model-small-en-us-0.15'),
            '/opt/vosk-models/vosk-model-small-en-us-0.15',
        ])
        for path in candidates:
            if path and os.path.isdir(path):
                return path
        raise RuntimeError('Vosk model not found. Check path or download it.')

    def _open_audio_stream(self) -> None:
        self._pa = pyaudio.PyAudio()
        fmt = pyaudio.paInt16
        channels = 1
        chunk = 4096

        def try_open(rate: int):
            return self._pa.open(
                format=fmt, channels=channels, rate=rate, input=True,
                input_device_index=self._device_index, frames_per_buffer=chunk,
            )

        try:
            self._stream = try_open(self._target_rate)
            self._input_rate = self._target_rate
        except Exception as e:
            self.get_logger().warn(f'Could not open mic at {self._target_rate} Hz ({e!r}); trying default.')
            info = self._pa.get_default_input_device_info()
            self._input_rate = int(info['defaultSampleRate'])
            self._stream = try_open(self._input_rate)

    def _audio_loop(self) -> None:
        assert self._stream is not None
        
        wake_words = ['hi', 'high', 'hey', 'hello', 'techno', 'tech no', 'tekno']
        sleep_words = ['goodbye', 'bye', 'go to sleep', 'stop listening']

        self._is_awake = False
        self._command_buffer = ""
        self._last_voice_time = time.time()
        self._idle_timeout_sec = 60.0 
        self._silence_send_timeout_sec = 10.0 # 🟢 Increased to 3.5 seconds so you have time to think!

        self.get_logger().info("💤 Sleeping. Waiting for wake word...")

        while not self._shutdown.is_set():
            try:
                data = self._stream.read(4096, exception_on_overflow=False)
            except Exception:
                time.sleep(0.5)
                continue

            with self._state_lock:
                speaking = self._speaking
                rec = self._recognizer

            # Ignore the microphone completely while the robot is talking
            if speaking:
                continue

            # 🟢 AUTO-SLEEP TIMEOUT 🟢
            if self._is_awake and not self._command_buffer and (time.time() - self._last_voice_time > self._idle_timeout_sec):
                self._is_awake = False
                self._command_buffer = ""
                self.get_logger().info("💤 20 seconds of silence. Going back to sleep.")
                continue

            # 🟢 SEND BUFFER TO AI 🟢
            # Only send if the buffer has words AND you have paused for 3.5 seconds
            if self._command_buffer.strip() and (time.time() - self._last_voice_time > self._silence_send_timeout_sec):
                final_command = self._command_buffer.strip()
                self._command_buffer = ""
                self.get_logger().info(f'🚀 Silence detected. Sending Full Sentence to AI: "{final_command}"')
                self._process_llm_query(final_command)
                continue

            if self._input_rate != self._target_rate:
                data = _resample_pcm16_mono(data, self._input_rate, self._target_rate)

            # AcceptWaveform triggers when Vosk thinks you paused
            if rec.AcceptWaveform(data):
                try:
                    payload = json.loads(rec.Result())
                except json.JSONDecodeError:
                    continue
                
                text = (payload.get('text') or '').strip()
                if not text:
                    continue
                
                norm = _normalize_phrase(text)
                
                # 1. IF ASLEEP: Look for Wake Word
                if not self._is_awake:
                    words = norm.split()
                    if any(w in words for w in wake_words):
                        self._is_awake = True
                        self._last_voice_time = time.time()
                        
                        # Strip wake word
                        clean_text = norm
                        for w in wake_words:
                            clean_text = re.sub(rf'\b{w}\b', '', clean_text).strip()
                        
                        # Set the buffer. If it's empty, it won't send anything! It will just wait.
                        self._command_buffer = clean_text
                        self.get_logger().info('🟢 Woke up! I am listening to your question now...')
                        
                        if self._command_buffer:
                            self.get_logger().info(f"🧱 [BUILDING SENTENCE]: '{self._command_buffer}'")
                
                # 2. IF AWAKE: Listen to everything
                else:
                    if any(w in norm for w in sleep_words):
                        self._is_awake = False
                        self.get_logger().info('🔴 Sleep command heard. Going to sleep after this response.')
                    
                    # Stitch the broken chunks together
                    if self._command_buffer:
                        self._command_buffer += " " + norm
                    else:
                        self._command_buffer = norm
                        
                    self._last_voice_time = time.time()
                    self.get_logger().info(f"🧱 [BUILDING SENTENCE]: '{self._command_buffer}'")
            
            # PartialResult triggers WHILE you are speaking
            else:
                try:
                    partial_payload = json.loads(rec.PartialResult())
                    partial_text = partial_payload.get('partial', '').strip()
                    
                    if partial_text:
                        self._last_voice_time = time.time() # Keep resetting timers while talking
                        
                        if self._is_awake:
                            self.get_logger().info(f"▶️ Active speaking: {partial_text}")
                except json.JSONDecodeError:
                    pass

    def _in_cooldown(self) -> bool:
        return (time.monotonic() - self._last_speak_time) < self._cooldown

    def _process_llm_query(self, user_text: str) -> None:
        with self._state_lock:
            if self._speaking or self._in_cooldown():
                return
            self._speaking = True

        def run() -> None:
            try:
                self.get_logger().info('Thinking...')
                
                # Ask Groq (Llama 3.1)
                chat_completion = self.llm_client.chat.completions.create(
                    messages=[
                        {"role": "system", "content": SYSTEM_PROMPT},
                        {"role": "user", "content": user_text}
                    ],
                    model="llama-3.1-8b-instant", 
                    temperature=0.7,
                    max_tokens=150
                )
                
                ai_response = chat_completion.choices[0].message.content
                self.get_logger().info(f'Techno says: "{ai_response}"')
                
                # Speak the response
                self._run_tts(ai_response)
                
            except Exception as e:
                self.get_logger().error(f'LLM Error: {e}')
                self._run_tts("Sorry, my connection to the server was interrupted.")
            finally:
                with self._state_lock:
                    self._speaking = False
                    self._last_voice_time = time.time() 
                
                if self._is_awake:
                    self.get_logger().info('🎤 Finished speaking. Listening for your reply...')

        threading.Thread(target=run, daemon=True).start()

    def _run_tts(self, text: str) -> None:
        try:
            from gtts import gTTS
            tts = gTTS(text=text, lang='en', tld='co.in')
            audio_file = '/tmp/techno_intro.mp3'
            tts.save(audio_file)
            subprocess.run(['mpg123', '-q', audio_file], check=True)
            return
        except Exception:
            # Offline Fallback
            cmd = [self._tts_executable, '-v', 'en-us+m3', '-s', str(self._tts_speed), text]
            subprocess.run(cmd, check=False)

    def destroy_node(self) -> bool:
        self._shutdown.set()
        if self._audio_thread is not None:
            self._audio_thread.join(timeout=2.0)
        if self._stream is not None:
            self._stream.stop_stream()
            self._stream.close()
        if self._pa is not None:
            self._pa.terminate()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = TechnoVoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()