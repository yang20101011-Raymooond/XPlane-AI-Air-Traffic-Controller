import json
import os
import time
from openai import OpenAI

LastRead = None
API_KEY = os.environ["LLM_API_KEY_AI_ATC_V1"]
client = OpenAI(
    api_key = API_KEY,
    base_url = "https://api.deepseek.com"
)

class ATCState:
    def __init__(self):
        self.active = False

        
try:
    with open("/Users/aic06/AI-ATC/flight_data.json") as f:
        flight_data = json.load(f)
except:
    time.sleep(0.5)

isBelowFL100 = ATCState()
STATE = isBelowFL100.active

if(STATE == False):
    response = client.chat.completions.create(
        model="deepseek-flash",
        messages=[
            {"role": "system", "content": "You are Air Traffic Controller for Xplane User, "
            "frame your words in professional aviation radio communication form and command the "
            "player to climb and maintain Fl100, based on information given. But do not act too"
            "mechanical, speak like real ATC. Only respond in one command, use numbers for numbers"},
            {"role": "user", "content": json.dumps(flight_data)},
        ],
        stream=False,
        reasoning_effort="high",
        extra_body={"thinking": {"type": "enabled"}}
    )

    print(response.choices[0].message.content)
    isBelowFL100.active = True


