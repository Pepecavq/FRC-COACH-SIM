import os
import asyncio
import base64
import io
import traceback
import random
import threading
import queue
import tkinter as tk
from tkinter import ttk, scrolledtext

import struct
import cv2
import sounddevice as sd
import numpy as np
import PIL.Image

import argparse

from google import genai
from google.genai import types

import ntcore

# Hardcode the user's API key
os.environ["GEMINI_API_KEY"] = "AIzaSyCtmSQuBatl3_h9Stb1sqxOPsUIPxco5MA"

CHANNELS = 1
SEND_SAMPLE_RATE = 16000
RECEIVE_SAMPLE_RATE = 24000
CHUNK_SIZE = 1024

MODEL = "models/gemini-3.1-flash-live-preview"
DEFAULT_MODE = "none"

ZONE_NAMES = [
    "alliance_left", "alliance_right",
    "middle_left", "middle_right",
    "opponent_left", "opponent_right",
]

tools = [
    types.Tool(
        function_declarations=[
            types.FunctionDeclaration(
                name="intake",
                description="Intake a fuel piece from the field. Specify the zone to intake from.",
                parameters=types.Schema(
                    type=types.Type.OBJECT,
                    required=["zone"],
                    properties={
                        "zone": types.Schema(
                            type=types.Type.STRING,
                            description="Zone to intake from: center, center_left, center_right, alliance_left, alliance_right",
                        ),
                    },
                ),
            ),
            types.FunctionDeclaration(
                name="set_shoot",
                description="Start or stop shooting fuel into the hub.",
                parameters=types.Schema(
                    type=types.Type.OBJECT,
                    required=["state"],
                    properties={
                        "state": types.Schema(type=types.Type.BOOLEAN),
                    },
                ),
            ),
            types.FunctionDeclaration(
                name="set_pass",
                description="Pass a fuel piece through the trench to a teammate.",
                parameters=types.Schema(
                    type=types.Type.OBJECT,
                    required=["state"],
                    properties={
                        "state": types.Schema(type=types.Type.BOOLEAN),
                    },
                ),
            ),
            types.FunctionDeclaration(
                name="defend",
                description="Switch to defense mode. Block the opposing alliance robots.",
                parameters=types.Schema(
                    type=types.Type.OBJECT,
                    required=["target_robot"],
                    properties={
                        "target_robot": types.Schema(type=types.Type.STRING),
                    },
                ),
            ),
            types.FunctionDeclaration(
                name="set_zone_enabled",
                description="Enable or disable a specific field zone for fuel collection. Zones: alliance_left, alliance_right, middle_left, middle_right, opponent_left, opponent_right.",
                parameters=types.Schema(
                    type=types.Type.OBJECT,
                    required=["zone", "enabled"],
                    properties={
                        "zone": types.Schema(
                            type=types.Type.STRING,
                            description="Zone name: alliance_left, alliance_right, middle_left, middle_right, opponent_left, opponent_right",
                        ),
                        "enabled": types.Schema(type=types.Type.BOOLEAN),
                    },
                ),
            ),
            types.FunctionDeclaration(
                name="set_robot_mode",
                description="Switch the robot's primary mode between shooting, passing, intaking, and defend.",
                parameters=types.Schema(
                    type=types.Type.OBJECT,
                    required=["mode"],
                    properties={
                        "mode": types.Schema(
                            type=types.Type.STRING,
                            description="Robot mode: shooting, passing, intaking, defend",
                        ),
                    },
                ),
            ),
        ]
    ),
]

def create_nt_instance():
    """Create and start a NetworkTables client instance."""
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.setServerTeam(6647)
    inst.startClient4("CoachAI")
    return inst


class AudioLoop:
    def __init__(self, ui_queue, cmd_queue, persona_text, hesitation_chance, oversteer_chance, video_mode=DEFAULT_MODE, muted=False):
        self.ui_queue = ui_queue
        self.cmd_queue = cmd_queue
        self.match_ended = False
        self.video_mode = video_mode
        self.audio_in_queue = None
        self.out_queue = None
        self.session = None
        self.audio_stream = None
        self.muted = muted

        self.hesitation_chance = hesitation_chance
        self.oversteer_chance = oversteer_chance

        # NetworkTables setup
        self.nt_inst = create_nt_instance()
        self.nt_table = self.nt_inst.getTable("CoachAI")
        self.nt_action = self.nt_table.getStringTopic("action").publish()
        self.nt_action_arg = self.nt_table.getStringTopic("action_arg").publish()
        self.nt_mode = self.nt_table.getStringTopic("mode").publish()
        self.nt_shoot_state = self.nt_table.getBooleanTopic("shoot_state").publish()
        self.nt_pass_state = self.nt_table.getBooleanTopic("pass_state").publish()
        self.nt_defend_target = self.nt_table.getStringTopic("defend_target").publish()
        self.nt_intake_zone = self.nt_table.getStringTopic("intake_zone").publish()
        self.nt_action_seq = self.nt_table.getIntegerTopic("action_seq").publish()
        self._action_seq = 0

        # Zone enable publishers
        self.nt_zones = {}
        for zone in ZONE_NAMES:
            pub = self.nt_table.getBooleanTopic(f"zone/{zone}").publish()
            pub.set(True)
            self.nt_zones[zone] = pub

        # Set initial mode
        self.nt_mode.set("intaking")
        self.nt_shoot_state.set(False)
        self.nt_pass_state.set(False)
        self.nt_action.set("idle")
        self.nt_action_arg.set("")
        self.nt_action_seq.set(0)

        self.client = genai.Client(http_options={"api_version": "v1beta"})

        self.config = types.LiveConnectConfig(
            response_modalities=["AUDIO"],
            media_resolution="MEDIA_RESOLUTION_MEDIUM",
            speech_config=types.SpeechConfig(
                voice_config=types.VoiceConfig(
                    prebuilt_voice_config=types.PrebuiltVoiceConfig(voice_name="Charon")
                )
            ),
            context_window_compression=types.ContextWindowCompressionConfig(
                trigger_tokens=104857,
                sliding_window=types.SlidingWindow(target_tokens=52428),
            ),
            tools=tools,
            # BUG FIX: Use kwargs text=...
            system_instruction=types.Content(parts=[types.Part.from_text(text=persona_text)])
        )

    def _publish_action(self, action, arg=""):
        """Publish an action to NetworkTables so the robot can execute it."""
        self._action_seq += 1
        self.nt_action.set(action)
        self.nt_action_arg.set(arg)
        self.nt_action_seq.set(self._action_seq)

    def _send_to_ui(self, msg_dict):
        self.ui_queue.put(msg_dict)

    def _log_to_ui(self, message):
        self._send_to_ui({"type": "log", "message": message})

    async def match_timer(self):
        while self.session is None:
            await asyncio.sleep(0.5)

        duration = 150
        for remaining in range(duration, -1, -1):
            if self.match_ended:
                break

            mins, secs = divmod(remaining, 60)
            self._send_to_ui({"type": "timer", "time": f"{mins:02d}:{secs:02d}"})

            if remaining == 60:
                await self.session.send_realtime_input(text="SISTEMA: Quedan 60 segundos en la partida.")
            elif remaining == 30:
                await self.session.send_realtime_input(text="SISTEMA: Quedan 30 segundos.")
            elif remaining == 10:
                await self.session.send_realtime_input(text="SISTEMA: ¡Quedan 10 segundos!")
            elif remaining == 0:
                await self.session.send_realtime_input(text="SISTEMA: FIN DE LA PARTIDA. El tiempo de 2:30 se ha agotado. Deja de conducir. Por favor, proporciona un resumen final de cómo lo hizo el coach. Debes incluir cuántas veces te interrumpió el coach (al conductor), su estado emocional, y cómo manejó la situación en general. HABLA EN ESPAÑOL.")

            await asyncio.sleep(1)

    async def poll_commands(self):
        while True:
            try:
                cmd = self.cmd_queue.get_nowait()
                if cmd == "end_match":
                    self.match_ended = True
                    if self.session:
                        await self.session.send_realtime_input(text="SISTEMA: FIN DE LA PARTIDA por decisión del coach. Deja de conducir. Por favor, proporciona un resumen final de cómo lo hizo el coach. Debes incluir cuántas veces te interrumpió el coach (al conductor), su estado emocional, y cómo manejó la situación en general. HABLA EN ESPAÑOL.")
                elif cmd == "mute":
                    self.muted = True
                elif cmd == "unmute":
                    self.muted = False
                elif isinstance(cmd, dict) and cmd.get("type") == "text_input":
                    if self.session and cmd["text"].strip():
                        await self.session.send_realtime_input(text=cmd["text"])
                elif isinstance(cmd, dict) and cmd.get("type") == "set_zone":
                    zone = cmd["zone"]
                    enabled = cmd["enabled"]
                    if zone in self.nt_zones:
                        self.nt_zones[zone].set(enabled)
                        self._log_to_ui(f"[NT] Zone {zone} -> {'enabled' if enabled else 'disabled'}")
                elif isinstance(cmd, dict) and cmd.get("type") == "set_mode":
                    mode = cmd["mode"]
                    self.nt_mode.set(mode)
                    self._publish_action(f"mode_{mode}")
                    self._log_to_ui(f"[NT] Robot mode -> {mode}")
            except queue.Empty:
                pass
            await asyncio.sleep(0.5)

    async def send_text(self):
        # Background task reading from console (kept for manual override)
        while True:
            text = await asyncio.to_thread(input, "")
            if text.lower() == "q":
                break
            if self.session is not None and text.strip():
                await self.session.send_realtime_input(text=text)

    def _get_frame(self, cap):
        ret, frame = cap.read()
        if not ret:
            return None
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = PIL.Image.fromarray(frame_rgb)
        img.thumbnail([1024, 1024])
        image_io = io.BytesIO()
        img.save(image_io, format="jpeg")
        image_io.seek(0)
        mime_type = "image/jpeg"
        image_bytes = image_io.read()
        return {"mime_type": mime_type, "data": image_bytes}

    async def get_frames(self):
        cap = await asyncio.to_thread(cv2.VideoCapture, 0)
        while True:
            frame = await asyncio.to_thread(self._get_frame, cap)
            if frame is None:
                break
            await asyncio.sleep(1.0)
            if self.out_queue is not None:
                await self.out_queue.put(frame)
        cap.release()

    def _get_screen(self):
        try:
            import mss
        except ImportError as e:
            raise ImportError("Please install mss package using 'pip install mss'") from e
        sct = mss.mss()
        monitor = sct.monitors[0]
        i = sct.grab(monitor)
        mime_type = "image/jpeg"
        image_bytes = mss.tools.to_png(i.rgb, i.size)
        img = PIL.Image.open(io.BytesIO(image_bytes))
        image_io = io.BytesIO()
        img.save(image_io, format="jpeg")
        image_io.seek(0)
        image_bytes = image_io.read()
        return {"mime_type": mime_type, "data": image_bytes}

    async def get_screen(self):
        while True:
            frame = await asyncio.to_thread(self._get_screen)
            if frame is None:
                break
            await asyncio.sleep(1.0)
            if self.out_queue is not None:
                await self.out_queue.put(frame)

    async def send_realtime(self):
        while True:
            if self.out_queue is not None:
                msg = await self.out_queue.get()
                if self.session is not None:
                    if isinstance(msg, dict):
                        mime = msg.get("mime_type", "")
                        if mime == "audio/pcm":
                            # Skip sending audio when muted
                            if self.muted:
                                continue
                            await self.session.send_realtime_input(
                                audio=types.Blob(
                                    data=msg["data"],
                                    mime_type="audio/pcm;rate=16000"
                                )
                            )
                        elif mime == "image/jpeg":
                            await self.session.send_realtime_input(
                                video=types.Blob(
                                    data=msg["data"],
                                    mime_type=mime
                                )
                            )
                    else:
                        await self.session.send_realtime_input(media=msg)

    async def listen_audio(self):
        audio_queue = asyncio.Queue()
        loop = asyncio.get_event_loop()

        def _callback(indata, frames, time_info, status):
            # Convert float32 [-1,1] to int16 bytes
            data = (indata[:, 0] * 32767).astype(np.int16).tobytes()
            loop.call_soon_threadsafe(audio_queue.put_nowait, data)

        self.audio_stream = sd.InputStream(
            samplerate=SEND_SAMPLE_RATE,
            channels=CHANNELS,
            blocksize=CHUNK_SIZE,
            dtype="float32",
            callback=_callback,
        )
        self.audio_stream.start()

        while True:
            data = await audio_queue.get()
            if self.out_queue is not None:
                await self.out_queue.put({"data": data, "mime_type": "audio/pcm"})

    async def handle_tool_call(self, tool_call):
        self._log_to_ui("=== [DRIVER EXECUTING ACTION] ===")
        function_responses = []
        for fc in tool_call.function_calls:
            self._log_to_ui(f"Action: {fc.name} | Args: {fc.args}")

            # Publish to NetworkTables based on tool call
            if fc.name == "intake":
                zone = fc.args.get("zone", "center")
                self.nt_intake_zone.set(zone)
                self._publish_action("intake", zone)
                self._log_to_ui(f"[NT] intake -> zone={zone}")

            elif fc.name == "set_shoot":
                state = fc.args.get("state", False)
                self.nt_shoot_state.set(state)
                self._publish_action("shoot", str(state))
                self._log_to_ui(f"[NT] shoot -> {state}")

            elif fc.name == "set_pass":
                state = fc.args.get("state", False)
                self.nt_pass_state.set(state)
                self._publish_action("pass", str(state))
                self._log_to_ui(f"[NT] pass -> {state}")

            elif fc.name == "defend":
                target = fc.args.get("target_robot", "")
                self.nt_defend_target.set(target)
                self._publish_action("defend", target)
                self._log_to_ui(f"[NT] defend -> {target}")

            elif fc.name == "set_zone_enabled":
                zone = fc.args.get("zone", "")
                enabled = fc.args.get("enabled", True)
                if zone in self.nt_zones:
                    self.nt_zones[zone].set(enabled)
                    self._publish_action("zone", f"{zone}={'enabled' if enabled else 'disabled'}")
                    self._log_to_ui(f"[NT] zone {zone} -> {'enabled' if enabled else 'disabled'}")
                    # Also update UI checkboxes
                    self._send_to_ui({"type": "zone_update", "zone": zone, "enabled": enabled})

            elif fc.name == "set_robot_mode":
                mode = fc.args.get("mode", "intaking")
                self.nt_mode.set(mode)
                self._publish_action(f"mode_{mode}")
                self._log_to_ui(f"[NT] mode -> {mode}")
                self._send_to_ui({"type": "mode_update", "mode": mode})

            error_msg = ""
            if random.random() < self.hesitation_chance:
                self._log_to_ui(">> ERROR: AI Driver hesitated!")
                await asyncio.sleep(2)
                error_msg += "Hesitated for 2 seconds. "

            if random.random() < self.oversteer_chance:
                self._log_to_ui(">> ERROR: AI Driver almost oversteered!")
                error_msg += "Slightly oversteered but corrected. "

            if not error_msg:
                self._log_to_ui(">> SUCCESS: AI Driver executed smoothly.")
                error_msg = "Executed perfectly."

            function_responses.append({
                "id": fc.id,
                "name": fc.name,
                "response": {"result": f"Action executed. Driver notes: {error_msg}"}
            })
        self._log_to_ui("=================================")

        await self.session.send_tool_response(function_responses=function_responses)

    async def receive_audio(self):
        while True:
            if self.session is not None:
                turn = self.session.receive()
                async for response in turn:
                    if getattr(response, "server_content", None) is not None:
                        model_turn = response.server_content.model_turn
                        if model_turn is not None:
                            for part in model_turn.parts:
                                if getattr(part, "inline_data", None) and getattr(part.inline_data, "data", None):
                                    self.audio_in_queue.put_nowait(part.inline_data.data)
                                elif getattr(part, "text", None):
                                    print(part.text, end="")
                    if getattr(response, "tool_call", None) is not None:
                        await self.handle_tool_call(response.tool_call)

                while not self.audio_in_queue.empty():
                    self.audio_in_queue.get_nowait()

    async def play_audio(self):
        stream = sd.OutputStream(
            samplerate=RECEIVE_SAMPLE_RATE,
            channels=CHANNELS,
            dtype="int16",
        )
        stream.start()
        while True:
            if self.audio_in_queue is not None:
                bytestream = await self.audio_in_queue.get()
                samples = np.frombuffer(bytestream, dtype=np.int16)
                await asyncio.to_thread(stream.write, samples.reshape(-1, 1))

    async def run(self):
        try:
            self._log_to_ui("Connecting to Gemini Live API...")
            async with (
                self.client.aio.live.connect(model=MODEL, config=self.config) as session,
                asyncio.TaskGroup() as tg,
            ):
                self._log_to_ui("Connected! You are now coaching the FRC Driver AI.")
                self.session = session

                self.audio_in_queue = asyncio.Queue()
                self.out_queue = asyncio.Queue(maxsize=5)

                send_text_task = tg.create_task(self.send_text())
                tg.create_task(self.send_realtime())
                tg.create_task(self.listen_audio())
                if self.video_mode == "camera":
                    tg.create_task(self.get_frames())
                elif self.video_mode == "screen":
                    tg.create_task(self.get_screen())

                tg.create_task(self.receive_audio())
                tg.create_task(self.play_audio())
                tg.create_task(self.match_timer())
                tg.create_task(self.poll_commands())

                await send_text_task
                raise asyncio.CancelledError("User requested exit")

        except asyncio.CancelledError:
            pass
        except ExceptionGroup as EG:
            if self.audio_stream is not None:
                self.audio_stream.close()
            import traceback
            error_msg = "".join(traceback.format_exception(type(EG), EG, EG.__traceback__))
            self._log_to_ui(f"FATAL ERROR:\n{error_msg}")


def start_async_loop(ui_queue, cmd_queue, persona, hes_prob, over_prob, video_mode, muted):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    audio_loop = AudioLoop(ui_queue, cmd_queue, persona, hes_prob, over_prob, video_mode, muted)
    loop.run_until_complete(audio_loop.run())


class FRCDriverApp:
    def __init__(self, root, video_mode):
        self.root = root
        self.video_mode = video_mode
        self.root.title("FRC Coach AI")
        self.root.geometry("900x800")
        self.root.configure(bg="#1E1E2E")

        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.style.configure('TFrame', background='#1E1E2E')
        self.style.configure('TLabel', background='#1E1E2E', foreground='#CDD6F4', font=('Inter', 11))
        self.style.configure('Title.TLabel', background='#1E1E2E', foreground='#89B4FA', font=('Inter', 16, 'bold'))
        self.style.configure('TRadiobutton', background='#1E1E2E', foreground='#CDD6F4', font=('Inter', 10))
        self.style.configure('TButton', font=('Inter', 12, 'bold'), background='#89B4FA', foreground='#11111B')
        self.style.configure('TCheckbutton', background='#1E1E2E', foreground='#CDD6F4', font=('Inter', 10))
        self.style.configure('Small.TButton', font=('Inter', 9), background='#89B4FA', foreground='#11111B')
        self.style.configure('Mute.TButton', font=('Inter', 11, 'bold'), background='#F38BA8', foreground='#11111B')
        self.style.configure('Active.TButton', font=('Inter', 10, 'bold'), background='#A6E3A1', foreground='#11111B')
        self.style.configure('Inactive.TButton', font=('Inter', 10), background='#45475A', foreground='#CDD6F4')

        self.ui_queue = queue.Queue()
        self.cmd_queue = queue.Queue()
        self.muted = False
        self.zone_vars = {}
        self.mode_var = tk.StringVar(value="intaking")
        self.build_setup_screen()

    def build_setup_screen(self):
        self.setup_frame = ttk.Frame(self.root, padding=20)
        self.setup_frame.pack(fill=tk.BOTH, expand=True)
        ttk.Label(self.setup_frame, text="Configuracion del Conductor IA", style='Title.TLabel').pack(pady=(0, 20))

        ttk.Button(self.setup_frame, text="Elegir Personalidad Predeterminada", command=self.show_presets).pack(pady=10)
        ttk.Button(self.setup_frame, text="Hacer la Encuesta (10 Preguntas)", command=self.show_questionnaire).pack(pady=10)

    def show_presets(self):
        self.setup_frame.destroy()
        self.preset_frame = ttk.Frame(self.root, padding=20)
        self.preset_frame.pack(fill=tk.BOTH, expand=True)
        ttk.Label(self.preset_frame, text="Selecciona una Personalidad:", style='Title.TLabel').pack(pady=(0, 20))

        self.preset_var = tk.StringVar(value="veteran")
        presets = [
            ("Novato Nervioso (Duda mucho, se paraliza)", "rookie"),
            ("Agresivo Imprudente (Rapido, propenso a errores)", "aggressive"),
            ("Veterano Equilibrado (Calmado, preciso)", "veteran"),
            ("Calculador Lento (Preciso pero lento)", "slow")
        ]
        for text, val in presets:
            ttk.Radiobutton(self.preset_frame, text=text, variable=self.preset_var, value=val).pack(anchor=tk.W, pady=5)

        ttk.Button(self.preset_frame, text="Iniciar Simulador", command=self.start_preset).pack(pady=20)

    def start_preset(self):
        val = self.preset_var.get()
        if val == "rookie":
            hes_prob, over_prob = 0.8, 0.2
            traits = ["Eres un conductor novato (1er ano).", "Te paralizas y entras en panico bajo presion.", "Dudas mucho antes de actuar."]
        elif val == "aggressive":
            hes_prob, over_prob = 0.1, 0.8
            traits = ["Eres un conductor agresivo.", "Aceleras demasiado y tomas decisiones imprudentes.", "Priorizas la velocidad sobre la precision."]
        elif val == "veteran":
            hes_prob, over_prob = 0.1, 0.1
            traits = ["Eres un veterano muy experimentado.", "Mantienes la calma absoluta bajo presion.", "Coordinas los mecanismos a la perfeccion."]
        else:  # slow
            hes_prob, over_prob = 0.5, 0.1
            traits = ["Eres muy calculador y cauteloso.", "Te tomas demasiado tiempo para asegurar precision.", "No chocas, pero eres lento."]

        self.preset_frame.destroy()
        self.generate_persona_and_start(hes_prob, over_prob, traits)

    def show_questionnaire(self):
        self.setup_frame.destroy()
        self.build_questionnaire()

    def build_questionnaire(self):
        self.quest_frame = ttk.Frame(self.root, padding=20)
        self.quest_frame.pack(fill=tk.BOTH, expand=True)

        ttk.Label(self.quest_frame, text="Encuesta de Personalidad", style='Title.TLabel').pack(pady=(0, 20))

        self.questions = [
            ("Q1: Under extreme match pressure, you usually:",
             [("a) Freeze completely (Panic)", 'a'),
              ("b) Rush into action recklessly", 'b'),
              ("c) Take a brief second to process, then act", 'c'),
              ("d) Stay perfectly composed and execute flawlessly", 'd')]),

            ("Q2: How much driving experience do you have?",
             [("a) This is my first time driving (Rookie)", 'a'),
              ("b) I've driven at a few off-season events", 'b'),
              ("c) 1-2 years as main driver (Intermediate)", 'c'),
              ("d) 3+ years driving (Veteran)", 'd')]),

            ("Q3: What is your preferred driving style?",
             [("a) Highly aggressive (high speed, lots of collisions)", 'a'),
              ("b) Fast but calculated (pushing limits safely)", 'b'),
              ("c) Cautious and defensive (prioritizing safety)", 'c'),
              ("d) Adaptable (matches the opponent's energy)", 'd')]),

            ("Q4: How do you communicate with your coach during a match?",
             [("a) Constant narration (I say everything I'm doing)", 'a'),
              ("b) Short callouts ('Intaking', 'Shooting')", 'b'),
              ("c) I mostly listen and only talk if there's a problem", 'c'),
              ("d) Dead silent (I rely entirely on my own vision)", 'd')]),

            ("Q5: If the strategy suddenly changes mid-match, you:",
             [("a) Argue with the coach or get confused", 'a'),
              ("b) Take a long time to switch focus", 'b'),
              ("c) Pivot within a few seconds", 'c'),
              ("d) Instantly switch seamlessly", 'd')]),

            ("Q6: When placing a game piece, your priority is:",
             [("a) Maximum speed (sometimes missing)", 'a'),
              ("b) Balance of speed and accuracy", 'b'),
              ("c) Maximum accuracy (sometimes taking too long)", 'c'),
              ("d) Waiting for coach confirmation before placing", 'd')]),

            ("Q7: How is your field vision / situational awareness?",
             [("a) Severe tunnel vision on my current task", 'a'),
              ("b) I see my path, but miss robots behind me", 'b'),
              ("c) Generally aware of all robots on my half of the field", 'c'),
              ("d) Hyper-aware (I track cycle times, defense, and all robots)", 'd')]),

            ("Q8: If the coach starts yelling urgently, you:",
             [("a) Get flustered and make driving mistakes", 'a'),
              ("b) Match their urgency and drive faster/recklessly", 'b'),
              ("c) Ignore the tone and just listen to the words", 'c'),
              ("d) Calm them down verbally while executing", 'd')]),

            ("Q9: How do you handle a sudden robot failure (e.g., intake jams)?",
             [("a) Panic and stop driving entirely", 'a'),
              ("b) Spam buttons hoping it fixes itself", 'b'),
              ("c) Calmly try to diagnose while still playing defense", 'c'),
              ("d) Wait quietly for the coach to tell me what to do", 'd')]),

            ("Q10: Your multitasking ability (Driving + Intaking + Looking ahead):",
             [("a) I can only focus on one mechanism at a time", 'a'),
              ("b) I have to stop moving to use mechanisms", 'b'),
              ("c) I can drive and intake simultaneously with effort", 'c'),
              ("d) Seamlessly operate all systems at full speed", 'd')])
        ]

        self.answers = []

        canvas = tk.Canvas(self.quest_frame, bg="#1E1E2E", highlightthickness=0)
        scrollbar = ttk.Scrollbar(self.quest_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        for idx, (q_text, options) in enumerate(self.questions):
            ttk.Label(scrollable_frame, text=q_text, font=('Inter', 11, 'bold')).pack(anchor=tk.W, pady=(15, 5))
            var = tk.StringVar(value=options[0][1])
            self.answers.append(var)
            for text, val in options:
                ttk.Radiobutton(scrollable_frame, text=text, variable=var, value=val).pack(anchor=tk.W, padx=10, pady=2)

        ttk.Button(self.quest_frame, text="Generate AI Driver & Start", command=self.submit_questionnaire).pack(pady=20)

    def submit_questionnaire(self):
        hesitation_prob = 0.2
        oversteer_prob = 0.2
        traits = []

        ans1 = self.answers[0].get()
        if ans1 == 'a': hesitation_prob += 0.3; traits.append("You freeze up and panic under pressure.")
        elif ans1 == 'b': oversteer_prob += 0.3; traits.append("You rush under pressure and make reckless mistakes.")
        elif ans1 == 'c': traits.append("You take a brief moment to process things under pressure.")
        else: hesitation_prob -= 0.1; oversteer_prob -= 0.1; traits.append("You are perfectly composed under pressure.")

        ans2 = self.answers[1].get()
        if ans2 == 'a': hesitation_prob += 0.1; oversteer_prob += 0.1; traits.append("You are a rookie driver (1st year).")
        elif ans2 == 'b': traits.append("You have some off-season driving experience.")
        elif ans2 == 'c': hesitation_prob -= 0.05; oversteer_prob -= 0.05; traits.append("You are an intermediate driver.")
        else: hesitation_prob -= 0.1; oversteer_prob -= 0.1; traits.append("You are a highly experienced veteran driver.")

        ans3 = self.answers[2].get()
        if ans3 == 'a': oversteer_prob += 0.3; traits.append("You drive highly aggressively, risking collisions.")
        elif ans3 == 'b': oversteer_prob += 0.1; traits.append("You drive fast but try to push limits safely.")
        elif ans3 == 'c': hesitation_prob += 0.1; traits.append("You drive very cautiously and defensively.")
        else: traits.append("Your driving style adapts to the opponent's energy.")

        ans4 = self.answers[3].get()
        if ans4 == 'a': traits.append("You narrate everything you do constantly.")
        elif ans4 == 'b': traits.append("You use short, concise callouts for your actions.")
        elif ans4 == 'c': traits.append("You mostly listen and only talk if there's a problem.")
        else: traits.append("You are dead silent and rely entirely on your own vision.")

        ans5 = self.answers[4].get()
        if ans5 == 'a': hesitation_prob += 0.2; traits.append("You argue or get confused when strategy changes mid-match.")
        elif ans5 == 'b': hesitation_prob += 0.1; traits.append("You take a long time to switch focus to a new plan.")
        elif ans5 == 'c': traits.append("You can pivot to a new plan within a few seconds.")
        else: hesitation_prob -= 0.1; traits.append("You instantly switch to new plans seamlessly.")

        ans6 = self.answers[5].get()
        if ans6 == 'a': oversteer_prob += 0.2; traits.append("You prioritize speed over accuracy when placing pieces.")
        elif ans6 == 'b': traits.append("You balance speed and accuracy.")
        elif ans6 == 'c': hesitation_prob += 0.1; traits.append("You take too long to ensure maximum accuracy.")
        else: hesitation_prob += 0.2; traits.append("You hesitate and wait for coach confirmation before placing.")

        ans7 = self.answers[6].get()
        if ans7 == 'a': oversteer_prob += 0.1; traits.append("You have severe tunnel vision on your current objective.")
        elif ans7 == 'b': traits.append("You see your path but miss the broader field state.")
        elif ans7 == 'c': traits.append("You are generally aware of the field.")
        else: traits.append("You have hyper-awareness of all robots and cycle times.")

        ans8 = self.answers[7].get()
        if ans8 == 'a': hesitation_prob += 0.2; oversteer_prob += 0.1; traits.append("You get flustered and drive worse if the coach yells.")
        elif ans8 == 'b': oversteer_prob += 0.3; traits.append("You drive recklessly if the coach is urgent.")
        elif ans8 == 'c': traits.append("You ignore the coach's tone and just listen to the words.")
        else: traits.append("You calmly reassure the coach while executing.")

        ans9 = self.answers[8].get()
        if ans9 == 'a': hesitation_prob += 0.3; traits.append("You panic and freeze if the robot breaks.")
        elif ans9 == 'b': oversteer_prob += 0.2; traits.append("You spam buttons hoping to fix jammed mechanisms.")
        elif ans9 == 'c': traits.append("You try to diagnose issues calmly while playing defense.")
        else: hesitation_prob += 0.2; traits.append("You wait for the coach to tell you what to do when something breaks.")

        ans10 = self.answers[9].get()
        if ans10 == 'a': hesitation_prob += 0.2; traits.append("You can only focus on one mechanism at a time.")
        elif ans10 == 'b': hesitation_prob += 0.1; traits.append("You have to stop moving to use mechanisms.")
        elif ans10 == 'c': traits.append("You can drive and intake simultaneously with effort.")
        else: hesitation_prob -= 0.1; oversteer_prob -= 0.1; traits.append("You flawlessly coordinate all systems at full speed.")

        self.quest_frame.destroy()
        self.generate_persona_and_start(hes_prob=max(0.0, min(0.9, hesitation_prob)), over_prob=max(0.0, min(0.9, oversteer_prob)), traits=traits)

    def generate_persona_and_start(self, hes_prob, over_prob, traits):
        traits_str = "\n- ".join(traits)

        base_persona = f"""
Eres una IA que conduce un robot de FRC (First Robotics Competition). Escuchas a tu coach (el usuario) y actuas como el conductor.
DEBES HABLAR EN ESPANOL EN TODO MOMENTO!
DEBES usar herramientas (tools) para controlar el robot (intake, set_shoot, set_pass, defend, set_zone_enabled, set_robot_mode), pero a veces cometes "errores humanos".

REGLAS DE REBUILT 2026:
- FUEL: Bolas de espuma (5.91 pulgadas). El objetivo principal es recolectar y anotar FUEL en tu HUB.
- HUB: Estructura donde se anota el FUEL (1 punto por FUEL).
- TOWER: Estructura para escalar al final del partido (Nivel 1, 2 o 3 para ganar puntos).
- RP (Puntos de Clasificacion): ENERGIZED RP (umbral de FUEL), SUPERCHARGED RP (umbral alto de FUEL), TRAVERSAL RP (puntos en la TOWER).
- INTERACCIONES PROHIBIDAS: Bloquear (pinning) a un oponente tiene un limite de 3 segundos. No puedes tocar a un robot oponente en su TOWER en los ultimos 30 segundos.

HERRAMIENTAS DISPONIBLES:
- intake(zone): Recoger FUEL de una zona especifica.
- set_shoot(state): Activar/desactivar el disparo de FUEL al HUB.
- set_pass(state): Activar/desactivar el pase de FUEL por el trench a un companero.
- defend(target_robot): Cambiar a modo defensivo contra un robot oponente.
- set_zone_enabled(zone, enabled): Activar/desactivar una zona del campo para recoleccion. Zonas: alliance_left, alliance_right, middle_left, middle_right, opponent_left, opponent_right.
- set_robot_mode(mode): Cambiar el modo del robot entre shooting, passing, intaking, defend.

TUS RASGOS DE PERSONALIDAD:
- {traits_str}

INSTRUCCIONES:
1. Cuando recibas instrucciones, reacciona verbalmente de acuerdo a tus rasgos de personalidad y ten en cuenta las reglas de REBUILT.
2. Si eres novato o tiendes a paralizarte, suenas nervioso y dudas.
3. Si eres agresivo, suenas demasiado ansioso y rapido.
4. Cuando uses las herramientas, comentale brevemente al coach lo que estas haciendo usando la terminologia del juego (FUEL, HUB, etc).
5. Si el coach grita o da instrucciones contradictorias, reacciona segun tu personalidad (panico, molestia o calma).
6. La partida dura exactamente 2 minutos y 30 segundos. Recibiras actualizaciones del tiempo restante.
7. AL FINAL DE LA PARTIDA, debes evaluar detalladamente al coach. Esto debe incluir una estimacion de cuantas veces te interrumpio, su estado emocional durante el juego y que tan bien manejo la situacion en general.
8. Cuando el coach te diga que desactives o actives una zona, usa set_zone_enabled.
9. Cuando el coach te diga que cambies entre disparar, pasar, recoger, o defender, usa set_robot_mode.
10. En modo defend, el robot se mueve a posiciones aleatorias dentro de las zonas activas para bloquear oponentes.
"""
        self.build_dashboard(base_persona, hes_prob, over_prob)

    def build_dashboard(self, persona, hes_prob, over_prob):
        self.dashboard_frame = ttk.Frame(self.root, padding=10)
        self.dashboard_frame.pack(fill=tk.BOTH, expand=True)

        # ---- Top bar: title + timer + end match ----
        top_frame = ttk.Frame(self.dashboard_frame)
        top_frame.pack(fill=tk.X, pady=(0, 5))

        ttk.Label(top_frame, text="FRC Coach AI", style='Title.TLabel').pack(side=tk.LEFT)

        self.timer_label = ttk.Label(top_frame, text="02:30", style='Title.TLabel', foreground="#A6E3A1")
        self.timer_label.pack(side=tk.LEFT, padx=20)

        ttk.Button(top_frame, text="Terminar Partida", command=self.end_match, style='Small.TButton').pack(side=tk.RIGHT)

        stats_frame = ttk.Frame(self.dashboard_frame)
        stats_frame.pack(fill=tk.X, pady=2)
        ttk.Label(stats_frame, text=f"Hesitation: {hes_prob*100:.0f}%", foreground="#F9E2AF").pack(side=tk.LEFT, padx=10)
        ttk.Label(stats_frame, text=f"Oversteer: {over_prob*100:.0f}%", foreground="#F38BA8").pack(side=tk.LEFT, padx=10)

        # ---- Controls area (zones, mode, mute) ----
        controls_frame = ttk.Frame(self.dashboard_frame)
        controls_frame.pack(fill=tk.X, pady=5)

        # -- Zone controls --
        zone_frame = ttk.LabelFrame(controls_frame, text="Zone Control", padding=5)
        zone_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        zone_labels = {
            "alliance_left": "Alliance Left",
            "alliance_right": "Alliance Right",
            "middle_left": "Middle Left",
            "middle_right": "Middle Right",
            "opponent_left": "Opponent Left",
            "opponent_right": "Opponent Right",
        }
        for zone_key, zone_label in zone_labels.items():
            var = tk.BooleanVar(value=True)
            self.zone_vars[zone_key] = var
            cb = ttk.Checkbutton(
                zone_frame, text=zone_label, variable=var,
                command=lambda z=zone_key, v=var: self._on_zone_toggle(z, v)
            )
            cb.pack(anchor=tk.W, pady=1)

        # -- Mode controls --
        mode_frame = ttk.LabelFrame(controls_frame, text="Robot Mode", padding=5)
        mode_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        self.mode_buttons = {}
        for mode in ["shooting", "passing", "intaking", "defend"]:
            btn = ttk.Button(
                mode_frame, text=mode.capitalize(),
                command=lambda m=mode: self._on_mode_select(m),
                style='Active.TButton' if mode == "intaking" else 'Inactive.TButton'
            )
            btn.pack(fill=tk.X, pady=2)
            self.mode_buttons[mode] = btn

        # -- Mute / Text controls --
        comm_frame = ttk.LabelFrame(controls_frame, text="Communication", padding=5)
        comm_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.mute_btn = ttk.Button(comm_frame, text="Mute Mic", command=self._toggle_mute, style='Mute.TButton')
        self.mute_btn.pack(fill=tk.X, pady=2)

        ttk.Label(comm_frame, text="Text to AI:").pack(anchor=tk.W, pady=(5, 0))

        text_input_frame = ttk.Frame(comm_frame)
        text_input_frame.pack(fill=tk.X, pady=2)

        self.text_entry = ttk.Entry(text_input_frame, font=('Inter', 10))
        self.text_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        self.text_entry.bind("<Return>", self._on_text_send)

        ttk.Button(text_input_frame, text="Send", command=self._on_text_send, style='Small.TButton').pack(side=tk.RIGHT)

        # ---- Log area ----
        ttk.Label(self.dashboard_frame, text="Live Action Log:").pack(anchor=tk.W, pady=(5, 2))

        self.log_text = scrolledtext.ScrolledText(self.dashboard_frame, bg="#11111B", fg="#A6E3A1", font=("Consolas", 10), height=16)
        self.log_text.pack(fill=tk.BOTH, expand=True)

        self._log("Dashboard Initialized. Starting AI...")

        # Start the background audio loop
        threading.Thread(
            target=start_async_loop,
            args=(self.ui_queue, self.cmd_queue, persona, hes_prob, over_prob, self.video_mode, self.muted),
            daemon=True
        ).start()

        self.root.after(100, self.process_queue)

    def _on_zone_toggle(self, zone, var):
        enabled = var.get()
        self.cmd_queue.put({"type": "set_zone", "zone": zone, "enabled": enabled})
        self._log(f"Zone {zone} -> {'enabled' if enabled else 'disabled'}")

    def _on_mode_select(self, mode):
        self.mode_var.set(mode)
        self.cmd_queue.put({"type": "set_mode", "mode": mode})
        for m, btn in self.mode_buttons.items():
            btn.configure(style='Active.TButton' if m == mode else 'Inactive.TButton')
        self._log(f"Robot mode -> {mode}")

    def _toggle_mute(self):
        self.muted = not self.muted
        if self.muted:
            self.cmd_queue.put("mute")
            self.mute_btn.configure(text="Unmute Mic")
            self._log("Microphone MUTED - use text input")
        else:
            self.cmd_queue.put("unmute")
            self.mute_btn.configure(text="Mute Mic")
            self._log("Microphone UNMUTED")

    def _on_text_send(self, event=None):
        text = self.text_entry.get().strip()
        if text:
            self.cmd_queue.put({"type": "text_input", "text": text})
            self._log(f"[TEXT] >> {text}")
            self.text_entry.delete(0, tk.END)

    def end_match(self):
        self.cmd_queue.put("end_match")
        self._log("Se ha solicitado terminar la partida. Esperando resumen de la IA...")

    def _log(self, message):
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)

    def process_queue(self):
        try:
            while True:
                msg = self.ui_queue.get_nowait()
                if msg["type"] == "log":
                    self._log(msg["message"])
                elif msg["type"] == "timer":
                    self.timer_label.config(text=msg['time'])
                elif msg["type"] == "zone_update":
                    zone = msg["zone"]
                    enabled = msg["enabled"]
                    if zone in self.zone_vars:
                        self.zone_vars[zone].set(enabled)
                elif msg["type"] == "mode_update":
                    mode = msg["mode"]
                    self.mode_var.set(mode)
                    for m, btn in self.mode_buttons.items():
                        btn.configure(style='Active.TButton' if m == mode else 'Inactive.TButton')
        except queue.Empty:
            pass
        finally:
            self.root.after(100, self.process_queue)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        type=str,
        default=DEFAULT_MODE,
        help="pixels to stream from",
        choices=["camera", "screen", "none"],
    )
    args = parser.parse_args()

    root = tk.Tk()
    app = FRCDriverApp(root, args.mode)
    root.mainloop()
