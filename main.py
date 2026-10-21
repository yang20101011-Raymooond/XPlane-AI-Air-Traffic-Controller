import json
import os
import time
import queue
import threading
import numpy as np
import random
import whisper
import sounddevice
from openai import OpenAI

API_KEY = os.environ["LLM_API_KEY_AI_ATC_V1"]
client = OpenAI(
    api_key = API_KEY,
    base_url = "https://api.deepseek.com"
)

speech_model = whisper.load_model("small")

CIFP_DIR = "/Users/aic06/X-Plane 12/Resources/default data/CIFP"

user_input = queue.Queue()##create a queue for input

##class for determing if the command was already given
class ATCState:
    def __init__(self):
        self.active = False
isInitialclimbClearance = ATCState()   
isClearanced = ATCState()
isInitialClimbed = ATCState()

        
SID_cache = {} ##dictionary for storing SIDs read from CIFP navigation document of xplane

def get_SID(icao, runway):
    if(not icao or not runway):
        return []

    key = f"{icao}|{runway}"
    if(key in SID_cache):
        return SID_cache[key]
    
    path = os.path.join(CIFP_DIR, f"{icao.upper()}.dat")
    sids = set()
    if(os.path.exists(path) == True):
        try:
            with open(path, encoding="utf-8", errors="ignore") as f:
                for i in f:            ##To Tim: 遍历each line in f, python的for循环的特殊部份
                    if(not i.startswith("SID:")):
                        continue
                    arr = i.split(",")##split the line into seperate string each ",", and store them into an array
                    if(len(arr) < 4):
                        continue
                    sid_name = arr[2].strip()##SID name is the 3rd element in array we converted, 
                    ##originally in text its the 3rd part, strip to prevent space at the end
                    rw_name = arr[3].strip()##simialr reason~
                    if(rw_name == runway and sid_name != None):
                        sids.add(sid_name)#use set since it auto delete the repeated sections
        except OSError:
            return []
    result = sorted(sids)
    SID_cache[key] = result
    return result

def remindReadback(REGISTRATION):
    n = random.randint(1, 4)
    if(n == 1 or n == 2):
        print(REGISTRATION, ", Are you with me, readback instructions")
    elif(n == 3):
        print(REGISTRATION, ", Please confirm and readback")
    elif(n == 4):
        print(REGISTRATION, ", Are you with me, readback and confirm")

def Readback(REGISTRATION):
    n = random.randint(1, 2)
    if(n == 1):
        print(REGISTRATION, ", Readback Correct, clearance granted, contact when ready for departure")
    elif(n == 2):
        print(REGISTRATION, ", Readbac、" \
        " Confirmed, contact when ready for departure")

print(">>> ---- Enter Depature Airport ICAO code: ")
DEP_ICAO = input().strip().upper()
print(">>> ---- Enter Depature Runway(Make Sure runway exists): ")
DEP_RWY = input().strip().upper()

possibleSID = get_SID(DEP_ICAO, DEP_RWY)
SID_text = "SIDs available for this runway are: " + ", ".join(possibleSID)
print("[debug] SIDs available for this runway are: ", SID_text)


##functinon for the sub thread for monitoring input, prevent waiting in main loop
def preInput():
    while True:
        try:
            user_input.put(input())##ptut input value to the queue, if none, continue waiting
        except EOFError:
            break

##start a sub thread/process for user input
threading.Thread(target=preInput, daemon=True).start()


SAMPLE_RATE = 16000
isRecording = False
audio_data = []
def audio_callback(indata, frame, time, status):
    if isRecording == True:
        audio_data.append(indata.copy())

audioStream = sounddevice.InputStream(samplerate=SAMPLE_RATE, channels=1, callback=audio_callback)
audioStream.start()

print(">>> Press R to speak")
while True:
    try:
        with open("/Users/aic06/AI-ATC/flight_data.json") as f:
            flight_data = json.load(f)
            agl_m = flight_data["above ground level"]## AGL get from dataref is in meter
            agl = agl_m * 3.28 ##convert into feetv
            ias = flight_data["indicated-airspeed"]
            hdg = flight_data["heading"]
            alt = flight_data["altitude"]
            vs = flight_data["vertical speed"]

            AIRCRAFT_REG = flight_data["aircraft-registration"]
    except:
        time.sleep(0.5)
        continue

    try:
        with open("/Users/aic06/AI-ATC/clearance.json") as f:
            clearance_data = json.load(f)
            if(clearance_data.get("runway")):
                init_alt = clearance_data["initial_climb_ft"]
            else:
                init_alt = 6000 ##set defult value
    except:
        time.sleep(0.5)

    input_Pilot = None
    try:
        input_Pilot = user_input.get_nowait()
        if(input_Pilot != None):
            print("YOU: ", input_Pilot)
    except queue.Empty:
        pass
    
    pilotSays = None
    if(input_Pilot == "r" or input_Pilot == "R"):
        if(isRecording == False):
            isRecording = True
            print("<<< Recording...")

        else:
            isRecording = False
            AUDIO = np.concatenate(audio_data, axis=0)
            AUDIO = AUDIO.flatten()
            result = speech_model.transcribe(AUDIO, language = "en", initial_prompt="ATC radio communication. User may say alphabet in Alpha Bravo "
                   "Charlie Delta Echo Foxtrot Golf Hotel India Juliett Kilo Lima "
                   "Mike November Oscar Papa Quebec Romeo Sierra Tango Uniform "
                   "Victor Whiskey Xray Yankee Zulu.", fp16=False)
            pilotSays = result["text"]
            print("YOU: ", pilotSays)

            audio_data = []
    
    if(agl >= 500):
        if(isInitialclimbClearance.active == False):
            
            print(AIRCRAFT_REG, ", Climb and maintain ", init_alt, ", contact departure")
            last_remind_initC = time.perf_counter()
            
            isInitialclimbClearance.active = True

    now = time.perf_counter()
    if(agl >= 500 and agl <= init_alt):
        if(isInitialClimbed.active == False):
            if(vs > 200):
                isInitialClimbed.active = True
            else:
                if(now - last_remind_initC >= 60):
                    print(AIRCRAFT_REG, ", I repeat, Climb and maintain ", init_alt, ", you are currently not climbing")
                pass
        pass



    if(input_Pilot != None and pilotSays != None):
        if(isClearanced.active == True):
            response = client.chat.completions.create(
                model="deepseek-flash",
                messages=[
                    {"role": "system", "content": "You are Air Traffic Controller for Xplane User, "
                    "frame your words in professional aviation radio communication form and commands."
                    "Player have entered requests, respond professionally and add greetings if needed"
                    "or imitated real ATC. You will recieve both flight data and players request, flight clearance"
                    "will also be attached.r"
                    " Players may speech to text, if you recieve"
                    "text formed numbers and ICAO phonetic alphabet, response normally without text form"},
                    {"role": "user", "content": json.dumps(flight_data) + input_Pilot + 
                     pilotSays + json.dumps(clearance_data)},
                ],
                stream=False,
                reasoning_effort="high",
                extra_body={"thinking": {"type": "enabled"}}
            )
                
            print(response.choices[0].message.content)
        else:
            response = client.chat.completions.create(
                model="deepseek-flash",
                messages=[
                    {"role": "system", "content": "You are Air Traffic Controller for Xplane User, "
                    "frame your words in professional aviation radio communication form and commands."
                    "Player have entered requests or clearance, respond professionally and add greetings if needed"
                    "or imitated real ATC. If player enter clearance to destination,"
                    "grant the clearance and provide rounting navids and SID, if informations are missing"
                    "in the clearance, remind them for it. If player did not enter clearance, "
                    "remind them to ask for clearance first before other flight followings."
                    "If you are sending a clearance, format it in json and only respond with the"
                    "json text in such format"
                    "{\n"
                    '  "speech": "what you say to the pilot, natural language, described above",\n'
                    '  "data": {\n'
                    '    "callsign": "...",\n'
                    '    "runway": "...",\n'
                    '    "SID": "...",\n'
                    '    "Waypoints in route": "...",\n'
                    '    "initial_climb_ft": n,\n'
                    '    "cruise_altitude_ft": n,\n'
                    '    "squawk": "..."\n'
                    '    "Destination": "..."\n'
                    "  }\n"
                    "}\n"
                    "in such form, addition if needed, but MUST contains and say these followings, "
                    "if users does not state their runway or SID or following informations,"
                    "you must assign to them."
                    "But if the response is not a clearance, "
                    "respond the 'data' as {},"
                    "You will recieve both flight data and players request.Players may speech to text, if you recieve"
                    "text formed numbers and ICAO phonetic alphabet, response normally without text form"
                    
                    "YOU MUST CHOOSE THE SIDS GIVEN BELOW, DO NOT MAKE UO SIDS, Consider the routing to decide"
                    "which SID to assign"},
                    {"role": "user", "content": json.dumps(flight_data) + input_Pilot + pilotSays + SID_text},
                ],
                response_format={"type": "json_object"},
                stream=False,
                reasoning_effort="high",
                extra_body={"thinking": {"type": "enabled"}}
            )

            result = json.loads(response.choices[0].message.content)
            response_pilot = result["speech"] 
            print(response_pilot)
            ##print(result)
            clearance = result["data"]
            
            input_Pilot = None
            ##write clearance to json file, clearance is already similar to json format, sub json fie of "data"
            
            
            if(clearance.get('runway')):
                with open("clearance.json", "w", encoding="utf-8") as f:
                    json.dump(clearance, f, ensure_ascii=False, indent=2)

                last_time = time.time()
                pilotSays = None
                while(pilotSays == None):
                    try:##again detect input in threading
                        input_Pilot = None
                        input_Pilot = user_input.get_nowait()
                        if(input_Pilot != None):
                            print("YOU: ", input_Pilot)
                    except queue.Empty:
                        pass
                    if(input_Pilot == "r" or input_Pilot == "R"):
                        if(isRecording == False):
                            isRecording = True
                            print("<<< Recording...")
                        else:
                            isRecording = False
                            AUDIO = np.concatenate(audio_data, axis=0)
                            AUDIO = AUDIO.flatten()
                            print("[debug] readback 音频秒数:", len(AUDIO) / SAMPLE_RATE)
                            result = speech_model.transcribe(AUDIO, language = "en", initial_prompt="ATC radio communication. User may say alphabet in Alpha Bravo "
                            "Charlie Delta Echo Foxtrot Golf Hotel India Juliett Kilo Lima "
                            "Mike November Oscar Papa Quebec Romeo Sierra Tango Uniform "
                            "Victor Whiskey Xray Yankee Zulu.", condition_on_previous_text= False, fp16=False)
                            pilotSays = result["text"]
                            print("YOU: ", pilotSays)
                            
                            audio_data = []
                    
                    time.sleep(0.02)

                    now = time.time()
                    if(now - last_time >= 45):
                        remindReadback(AIRCRAFT_REG)
                        last_time = time.time()

                with open("/Users/aic06/AI-ATC/clearance.json") as f:
                        clearance_data = json.load(f)
                callsign = clearance_data["callsign"]
                sid = clearance_data["SID"]
                waypoints = clearance_data["Waypoints in route"]
                squawk = clearance_data["squawk"]
                destination = clearance_data["Destination"]
                squawk = clearance_data["squawk"]
                
                if((callsign in pilotSays) or (sid in pilotSays) or (squawk in pilotSays) or 
                   (destination in pilotSays) or (squawk in pilotSays) or ("readback" in pilotSays) or ("roger" in pilotSays)):
                    Readback(AIRCRAFT_REG)
                else:
                    print(AIRCRAFT_REG, ", negative, I repeat, clear to ", destination, " routing as fieled ", waypoints, ", climb via ", sid, 
                          ", squawk ", squawk, ", readback not necessary, contact when ready for departure")
                
                isClearanced.active = True




