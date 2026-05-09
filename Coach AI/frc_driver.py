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

import cv2
import pyaudio
import PIL.Image

import argparse

from google import genai
from google.genai import types

# Hardcode the user's API key
os.environ["GEMINI_API_KEY"] = "AIzaSyAQ86zf_3DOjg_wj-2B4OJgDCj0oUqNQYU"

FORMAT = pyaudio.paInt16
CHANNELS = 1
SEND_SAMPLE_RATE = 16000
RECEIVE_SAMPLE_RATE = 24000
CHUNK_SIZE = 1024

MODEL = "models/gemini-3.1-flash-live-preview"
DEFAULT_MODE = "none"

tools = [
    types.Tool(
        function_declarations=[
            types.FunctionDeclaration(
                name="intake",
                description="Intake a piece. Tell the coach you are trying.",
                parameters=types.Schema(
                    type=types.Type.OBJECT,
                    required=["center", "center left", "center right", "alliance left", "alliance right"],
                    properties={
                        "center": types.Schema(type=types.Type.STRING),
                        "center left": types.Schema(type=types.Type.STRING),
                        "center right": types.Schema(type=types.Type.STRING),
                        "alliance left": types.Schema(type=types.Type.STRING),
                        "alliance right": types.Schema(type=types.Type.STRING),
                    },
                ),
            ),
            types.FunctionDeclaration(
                name="set_shoot",
                description="Shoot the piece. Let the coach know you are shooting.",
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
        ]
    ),
]

pya = pyaudio.PyAudio()

class AudioLoop:
    def __init__(self, ui_queue, cmd_queue, persona_text, hesitation_chance, oversteer_chance, video_mode=DEFAULT_MODE):
        self.ui_queue = ui_queue
        self.cmd_queue = cmd_queue
        self.match_ended = False
        self.video_mode = video_mode
        self.audio_in_queue = None
        self.out_queue = None
        self.session = None
        self.audio_stream = None
        
        self.hesitation_chance = hesitation_chance
        self.oversteer_chance = oversteer_chance
        
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
        mic_info = pya.get_default_input_device_info()
        self.audio_stream = await asyncio.to_thread(
            pya.open,
            format=FORMAT,
            channels=CHANNELS,
            rate=SEND_SAMPLE_RATE,
            input=True,
            input_device_index=mic_info["index"],
            frames_per_buffer=CHUNK_SIZE,
        )
        kwargs = {"exception_on_overflow": False} if __debug__ else {}
        while True:
            data = await asyncio.to_thread(self.audio_stream.read, CHUNK_SIZE, **kwargs)
            if self.out_queue is not None:
                await self.out_queue.put({"data": data, "mime_type": "audio/pcm"})

    async def handle_tool_call(self, tool_call):
        self._log_to_ui("=== [DRIVER EXECUTING ACTION] ===")
        function_responses = []
        for fc in tool_call.function_calls:
            self._log_to_ui(f"Action: {fc.name} | Args: {fc.args}")
            
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
        stream = await asyncio.to_thread(
            pya.open,
            format=FORMAT,
            channels=CHANNELS,
            rate=RECEIVE_SAMPLE_RATE,
            output=True,
        )
        while True:
            if self.audio_in_queue is not None:
                bytestream = await self.audio_in_queue.get()
                await asyncio.to_thread(stream.write, bytestream)

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

def start_async_loop(ui_queue, cmd_queue, persona, hes_prob, over_prob, video_mode):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    audio_loop = AudioLoop(ui_queue, cmd_queue, persona, hes_prob, over_prob, video_mode)
    loop.run_until_complete(audio_loop.run())

class FRCDriverApp:
    def __init__(self, root, video_mode):
        self.root = root
        self.video_mode = video_mode
        self.root.title("FRC Driver AI Simulator")
        self.root.geometry("600x700")
        self.root.configure(bg="#1E1E2E")
        
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.style.configure('TFrame', background='#1E1E2E')
        self.style.configure('TLabel', background='#1E1E2E', foreground='#CDD6F4', font=('Inter', 11))
        self.style.configure('Title.TLabel', background='#1E1E2E', foreground='#89B4FA', font=('Inter', 16, 'bold'))
        self.style.configure('TRadiobutton', background='#1E1E2E', foreground='#CDD6F4', font=('Inter', 10))
        self.style.configure('TButton', font=('Inter', 12, 'bold'), background='#89B4FA', foreground='#11111B')

        self.ui_queue = queue.Queue()
        self.cmd_queue = queue.Queue()
        self.build_setup_screen()

    def build_setup_screen(self):
        self.setup_frame = ttk.Frame(self.root, padding=20)
        self.setup_frame.pack(fill=tk.BOTH, expand=True)
        ttk.Label(self.setup_frame, text="Configuración del Conductor IA", style='Title.TLabel').pack(pady=(0, 20))
        
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
            ("Agresivo Imprudente (Rápido, propenso a errores)", "aggressive"),
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
            traits = ["Eres un conductor novato (1er año).", "Te paralizas y entras en pánico bajo presión.", "Dudas mucho antes de actuar."]
        elif val == "aggressive":
            hes_prob, over_prob = 0.1, 0.8
            traits = ["Eres un conductor agresivo.", "Aceleras demasiado y tomas decisiones imprudentes.", "Priorizas la velocidad sobre la precisión."]
        elif val == "veteran":
            hes_prob, over_prob = 0.1, 0.1
            traits = ["Eres un veterano muy experimentado.", "Mantienes la calma absoluta bajo presión.", "Coordinas los mecanismos a la perfección."]
        else: # slow
            hes_prob, over_prob = 0.5, 0.1
            traits = ["Eres muy calculador y cauteloso.", "Te tomas demasiado tiempo para asegurar precisión.", "No chocas, pero eres lento."]
            
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

        ans1 = self.answers[0].get() # Pressure
        if ans1 == 'a': hesitation_prob += 0.3; traits.append("You freeze up and panic under pressure.")
        elif ans1 == 'b': oversteer_prob += 0.3; traits.append("You rush under pressure and make reckless mistakes.")
        elif ans1 == 'c': traits.append("You take a brief moment to process things under pressure.")
        else: hesitation_prob -= 0.1; oversteer_prob -= 0.1; traits.append("You are perfectly composed under pressure.")

        ans2 = self.answers[1].get() # Experience (decoupled from feelings)
        if ans2 == 'a': hesitation_prob += 0.1; oversteer_prob += 0.1; traits.append("You are a rookie driver (1st year).")
        elif ans2 == 'b': traits.append("You have some off-season driving experience.")
        elif ans2 == 'c': hesitation_prob -= 0.05; oversteer_prob -= 0.05; traits.append("You are an intermediate driver.")
        else: hesitation_prob -= 0.1; oversteer_prob -= 0.1; traits.append("You are a highly experienced veteran driver.")

        ans3 = self.answers[2].get() # Driving style
        if ans3 == 'a': oversteer_prob += 0.3; traits.append("You drive highly aggressively, risking collisions.")
        elif ans3 == 'b': oversteer_prob += 0.1; traits.append("You drive fast but try to push limits safely.")
        elif ans3 == 'c': hesitation_prob += 0.1; traits.append("You drive very cautiously and defensively.")
        else: traits.append("Your driving style adapts to the opponent's energy.")

        ans4 = self.answers[3].get() # Communication
        if ans4 == 'a': traits.append("You narrate everything you do constantly.")
        elif ans4 == 'b': traits.append("You use short, concise callouts for your actions.")
        elif ans4 == 'c': traits.append("You mostly listen and only talk if there's a problem.")
        else: traits.append("You are dead silent and rely entirely on your own vision.")

        ans5 = self.answers[4].get() # Strategy change
        if ans5 == 'a': hesitation_prob += 0.2; traits.append("You argue or get confused when strategy changes mid-match.")
        elif ans5 == 'b': hesitation_prob += 0.1; traits.append("You take a long time to switch focus to a new plan.")
        elif ans5 == 'c': traits.append("You can pivot to a new plan within a few seconds.")
        else: hesitation_prob -= 0.1; traits.append("You instantly switch to new plans seamlessly.")

        ans6 = self.answers[5].get() # Placing piece
        if ans6 == 'a': oversteer_prob += 0.2; traits.append("You prioritize speed over accuracy when placing pieces.")
        elif ans6 == 'b': traits.append("You balance speed and accuracy.")
        elif ans6 == 'c': hesitation_prob += 0.1; traits.append("You take too long to ensure maximum accuracy.")
        else: hesitation_prob += 0.2; traits.append("You hesitate and wait for coach confirmation before placing.")

        ans7 = self.answers[6].get() # Field vision
        if ans7 == 'a': oversteer_prob += 0.1; traits.append("You have severe tunnel vision on your current objective.")
        elif ans7 == 'b': traits.append("You see your path but miss the broader field state.")
        elif ans7 == 'c': traits.append("You are generally aware of the field.")
        else: traits.append("You have hyper-awareness of all robots and cycle times.")

        ans8 = self.answers[7].get() # Coach yelling
        if ans8 == 'a': hesitation_prob += 0.2; oversteer_prob += 0.1; traits.append("You get flustered and drive worse if the coach yells.")
        elif ans8 == 'b': oversteer_prob += 0.3; traits.append("You drive recklessly if the coach is urgent.")
        elif ans8 == 'c': traits.append("You ignore the coach's tone and just listen to the words.")
        else: traits.append("You calmly reassure the coach while executing.")

        ans9 = self.answers[8].get() # Robot failure
        if ans9 == 'a': hesitation_prob += 0.3; traits.append("You panic and freeze if the robot breaks.")
        elif ans9 == 'b': oversteer_prob += 0.2; traits.append("You spam buttons hoping to fix jammed mechanisms.")
        elif ans9 == 'c': traits.append("You try to diagnose issues calmly while playing defense.")
        else: hesitation_prob += 0.2; traits.append("You wait for the coach to tell you what to do when something breaks.")

        ans10 = self.answers[9].get() # Multitasking
        if ans10 == 'a': hesitation_prob += 0.2; traits.append("You can only focus on one mechanism at a time.")
        elif ans10 == 'b': hesitation_prob += 0.1; traits.append("You have to stop moving to use mechanisms.")
        elif ans10 == 'c': traits.append("You can drive and intake simultaneously with effort.")
        else: hesitation_prob -= 0.1; oversteer_prob -= 0.1; traits.append("You flawlessly coordinate all systems at full speed.")

        self.quest_frame.destroy()
        self.generate_persona_and_start(hes_prob=max(0.0, min(0.9, hesitation_prob)), over_prob=max(0.0, min(0.9, oversteer_prob)), traits=traits)

    def generate_persona_and_start(self, hes_prob, over_prob, traits):
        traits_str = "\n- ".join(traits)

        base_persona = f"""
Eres una IA que conduce un robot de FRC (First Robotics Competition). Escuchas a tu coach (el usuario) y actúas como el conductor.
¡DEBES HABLAR EN ESPAÑOL EN TODO MOMENTO!
DEBES usar herramientas (tools) para controlar el robot (intake, set_shoot, defend), pero a veces cometes "errores humanos".

REGLAS DE REBUILT 2026:
- FUEL: Bolas de espuma (5.91 pulgadas). El objetivo principal es recolectar y anotar FUEL en tu HUB.
- HUB: Estructura donde se anota el FUEL (1 punto por FUEL).
- TOWER: Estructura para escalar al final del partido (Nivel 1, 2 o 3 para ganar puntos).
- RP (Puntos de Clasificación): ENERGIZED RP (umbral de FUEL), SUPERCHARGED RP (umbral alto de FUEL), TRAVERSAL RP (puntos en la TOWER).
- INTERACCIONES PROHIBIDAS: Bloquear (pinning) a un oponente tiene un límite de 3 segundos. No puedes tocar a un robot oponente en su TOWER en los últimos 30 segundos.

TUS RASGOS DE PERSONALIDAD:
- {traits_str}

INSTRUCCIONES:
1. Cuando recibas instrucciones, reacciona verbalmente de acuerdo a tus rasgos de personalidad y ten en cuenta las reglas de REBUILT.
2. Si eres novato o tiendes a paralizarte, suenas nervioso y dudas.
3. Si eres agresivo, suenas demasiado ansioso y rápido.
4. Cuando uses las herramientas 'intake', 'set_shoot' o 'defend', coméntale brevemente al coach lo que estás haciendo usando la terminología del juego (FUEL, HUB, etc).
5. Si el coach grita o da instrucciones contradictorias, reacciona según tu personalidad (pánico, molestia o calma).
6. La partida dura exactamente 2 minutos y 30 segundos. Recibirás actualizaciones del tiempo restante.
7. AL FINAL DE LA PARTIDA, debes evaluar detalladamente al coach. Esto debe incluir una estimación de cuántas veces te interrumpió, su estado emocional durante el juego y qué tan bien manejó la situación en general.
"""
        self.build_dashboard(base_persona, hes_prob, over_prob)

    def build_dashboard(self, persona, hes_prob, over_prob):
        self.dashboard_frame = ttk.Frame(self.root, padding=20)
        self.dashboard_frame.pack(fill=tk.BOTH, expand=True)

        ttk.Label(self.dashboard_frame, text="Live AI Driver Dashboard", style='Title.TLabel').pack(pady=(0, 10))
        
        self.timer_label = ttk.Label(self.dashboard_frame, text="Match Time: 02:30", style='Title.TLabel', foreground="#A6E3A1")
        self.timer_label.pack(pady=(0, 10))
        
        ttk.Button(self.dashboard_frame, text="Terminar Partida (Evaluación IA)", command=self.end_match).pack(pady=5)
        
        stats_frame = ttk.Frame(self.dashboard_frame)
        stats_frame.pack(fill=tk.X, pady=10)
        
        ttk.Label(stats_frame, text=f"Hesitation Probability: {hes_prob*100:.0f}%", foreground="#F9E2AF").pack(side=tk.LEFT, padx=10)
        ttk.Label(stats_frame, text=f"Oversteer Probability: {over_prob*100:.0f}%", foreground="#F38BA8").pack(side=tk.LEFT, padx=10)
        
        ttk.Label(self.dashboard_frame, text="Live Action Log:").pack(anchor=tk.W, pady=(10, 5))
        
        self.log_text = scrolledtext.ScrolledText(self.dashboard_frame, bg="#11111B", fg="#A6E3A1", font=("Consolas", 10), height=20)
        self.log_text.pack(fill=tk.BOTH, expand=True)

        self._log("Dashboard Initialized. Starting AI...")

        # Start the background audio loop
        threading.Thread(target=start_async_loop, args=(self.ui_queue, self.cmd_queue, persona, hes_prob, over_prob, self.video_mode), daemon=True).start()
        
        # Start checking the queue for updates
        self.root.after(100, self.process_queue)

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
                    self.timer_label.config(text=f"Match Time: {msg['time']}")
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