import os
import asyncio
from google import genai
from google.genai import types

def test_api():
    print(dir(types.LiveClientContent))
    print(dir(types.LiveClientRealtimeInput))
    
test_api()
