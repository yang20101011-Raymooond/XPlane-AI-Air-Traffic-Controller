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

import route_finder as rf
from getSIDandOthers import *
from offcoursejudgement import *

API_KEY = os.environ["LLM_API_KEY_AI_ATC_V1"]
client = OpenAI(
    api_key = API_KEY,
    base_url = "https://api.deepseek.com"
)

speech_model = whisper.load_model("small")

## 数据文件（flight_data.json / clearance.json）所在目录：默认脚本所在目录。
## 插件那边同样认 AIATC_DIR 环境变量，两边指到同一个目录即可。
AIATC_DIR = os.environ.get("AIATC_DIR", os.path.dirname(os.path.abspath(__file__)))
FLIGHT_DATA_PATH = os.path.join(AIATC_DIR, "flight_data.json")
CLEARANCE_PATH = os.path.join(AIATC_DIR, "clearance.json")

## X-Plane 导航数据目录：默认 ~/X-Plane 12/Resources/default data，可用环境变量 XP_DATA_DIR 覆盖。
XP_DATA_DIR = os.environ.get(
    "XP_DATA_DIR",
    os.path.expanduser("~/X-Plane 12/Resources/default data"),
)
CIFP_DIR = os.path.join(XP_DATA_DIR, "CIFP")

user_input = queue.Queue()##create a queue for input

##class for determing if the command was already given
class ATCState:
    def __init__(self):
        self.active = False
isInitialclimbClearance = ATCState()   
isClearanced = ATCState()
isInitialClimbed = ATCState()
isClearToCruiseAlt = ATCState()
isCruiseClimbed = ATCState()
judge_offcourse = ATCState()
isRemindOffcourse = ATCState()


def clr_Readback(REGISTRATION):
    n = random.randint(1, 2)
    if(n == 1):
        print(REGISTRATION, ", Readback Correct, clearance granted, contact when ready for departure")
    elif(n == 2):
        print(REGISTRATION, ", Readback Confirmed, contact when ready for departure")

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
        print(REGISTRATION, ", Readback Correct")
    elif(n == 2):
        print(REGISTRATION, ", Readback Confirmed, contact when ready for departure")

def _func_Readback(): 
    global isRecording, audio_data
    last_time = time.perf_counter()
    pilotSays = None
    audio_data = []
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
                #print("[debug] readback 音频秒数:", len(AUDIO) / SAMPLE_RATE)
                result = speech_model.transcribe(AUDIO, language = "en", initial_prompt="ATC radio communication. User may say alphabet in Alpha Bravo "
                "Charlie Delta Echo Foxtrot Golf Hotel India Juliett Kilo Lima "
                "Mike November Oscar Papa Quebec Romeo Sierra Tango Uniform "
                "Victor Whiskey Xray Yankee Zulu.", condition_on_previous_text= False, fp16=False)
                pilotSays = result["text"]
                print("YOU: ", pilotSays)
                            
                audio_data = []
                    
        time.sleep(0.02)

        now = time.perf_counter()
        if(now - last_time >= 90):
            remindReadback(AIRCRAFT_REG)
            last_time = time.perf_counter()
    return

print(">>> ---- Enter Depature Airport ICAO code: ")
DEP_ICAO = input().strip().upper()
print(">>> ---- Enter Arrival Airport ICAO code: ")
ARR_ICAO = input().strip().upper()

ROUTE_arr = rf.find_route(DEP_ICAO, ARR_ICAO)##return a list of waypoints, the first is departure airport, the last is arrival airport, 
##the second is SID endpoint, the second last is STAR startpoint

possibleSID = get_SID(DEP_ICAO, ROUTE_arr[1], CIFP_DIR)##return a list of possible SID for the departure airport and the SID endpoint
SID_text = "SIDs available are: " + ", ".join(possibleSID)

print("[debug] Route is: ", ROUTE_arr)
print("[debug] SIDs available for this route are: ", SID_text)

SID_CON_RW = match_sid_rw(possibleSID, CIFP_DIR, DEP_ICAO)###return a dictionary of SID and its corresponding runway, key is SID, value is runway
print("[debug] SID and its corresponding runway possible are: ", json.dumps(SID_CON_RW))

APT_COOR = rf._node_coord(ROUTE_arr[0], near=None)
SID_END_COOR = rf._node_coord(ROUTE_arr[1], near=APT_COOR)
APT2_COOR = rf._node_coord(ROUTE_arr[-1], near=None)
STAR_ORI_COOR = rf._node_coord(ROUTE_arr[-2], near=APT2_COOR)
L_APT_SID = rf._nm_distance(APT_COOR[0], APT_COOR[1], SID_END_COOR[0], SID_END_COOR[1])
L_APT2_STAR = rf._nm_distance(APT2_COOR[0], APT2_COOR[1], STAR_ORI_COOR[0], STAR_ORI_COOR[1])

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
        with open(FLIGHT_DATA_PATH) as f:
            flight_data = json.load(f)
            agl_m = flight_data["above ground level"]## AGL get from dataref is in meter
            agl = agl_m * 3.28 ##convert into feetv
            ias = flight_data["indicated-airspeed"]
            hdg = flight_data["heading"]
            alt = flight_data["altitude"]
            vs = flight_data["vertical speed"]
            airlat = flight_data["latitude"]
            airlon = flight_data["longitude"]

            AIRCRAFT_REG = flight_data["aircraft-registration"]
    except:
        time.sleep(0.5)
        continue

    try:
        with open(CLEARANCE_PATH) as f:
            clearance_data = json.load(f)
            if(clearance_data.get("runway")):
                init_alt = clearance_data["initial_climb_ft"]
                crusie_alt_ft = clearance_data["cruise_altitude_ft"]
                crusie_alt_fl = int(crusie_alt_ft / 100)
            else:
                init_alt = 6000 ##set defult value
    except:
        time.sleep(0.5)

    input_Pilot = None
    try:
        input_Pilot = user_input.get(timeout=0.02)##get input value from the queue, if none, continue waiting
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
            
            print(AIRCRAFT_REG, ", Climb and maintain ", init_alt, ", for further information, contact departure")
            last_remind_initC = time.perf_counter()
            _func_Readback()##start a sub thread for readback, if no readback after 90 seconds, remind them to readback
            isInitialclimbClearance.active = True

    now = time.perf_counter()
    if(agl >= 500 and alt <= init_alt):
        if(isInitialClimbed.active == False):
            if(vs > 200):
                isInitialClimbed.active = True
            else:
                if(now - last_remind_initC >= 120):
                    print(AIRCRAFT_REG, ", I repeat, Climb and maintain ", init_alt, ", you are currently not climbing, no readback required")
                    last_remind_initC = time.perf_counter()
                pass
        else:
            if(init_alt - alt <= 200):
                if(isClearToCruiseAlt.active == False):
                    print(AIRCRAFT_REG, ", Climb and maintain to crusie altitude", crusie_alt_fl)
                    last_remind_cruiseC = time.perf_counter()
                    _func_Readback()
                    isClearToCruiseAlt.active = True
    if(agl >= 500 and alt > init_alt and alt < crusie_alt_ft):
        if(isCruiseClimbed.active == False):
            if(vs > 200):
                isCruiseClimbed.active = True
            else:
                if(now - last_remind_cruiseC >= 120):
                    print(AIRCRAFT_REG, ", I repeat, Climb and maintain FL", crusie_alt_fl, ", you are currently not climbing, no readback required")
                    last_remind_cruiseC = time.perf_counter()
                pass
    
    L_AFT_APT = rf._nm_distance(APT_COOR[0], APT_COOR[1], airlat, airlon)#aircraft and departure
    L_AFT_APT2 = rf._nm_distance(APT2_COOR[0], APT2_COOR[1], airlat, airlon)#aircraft and destination
    isOffCourse = None
    if(L_AFT_APT >= L_APT_SID):
        judge_offcourse.active = True
    if(L_AFT_APT2 <= L_APT2_STAR):
        judge_offcourse.active = False
    if(judge_offcourse.active):
        isOffCourse = checkoffcourse(airlat, airlon, CLEARANCE_PATH)
    now_offc = time.perf_counter()
    if(not isOffCourse):
        isRemindOffcourse.active = False
    if(isOffCourse):
        if(isRemindOffcourse.active == False):
            decideDirectto(airlat, airlon, CLEARANCE_PATH, AIRCRAFT_REG)
            last_remind_offc = time.perf_counter()
            isRemindOffcourse.active = True
        if(now_offc - last_remind_offc >= 120):
            decideDirectto(airlat, airlon, CLEARANCE_PATH, AIRCRAFT_REG)
            last_remind_offc = time.perf_counter()

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
                    '    "QNH": "..."\n'
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
                    
                    "YOU MUST CHOOSE THE SIDS GIVEN BELOW, THEN THE RUWAY YOU GIVE SHOULD CORRESPOND TO THE SID YOU CHOOSE, THE CORRELATION WILL BE GIVEN, DO NOT MAKE UO SIDS OR RUNWAYS"
                    "YOUR ROUTE GIVEN MUST BE WHAT I GIVE YOU, DO NOT MAKE UP ROUTES"},
                    {"role": "user", "content": SID_text + json.dumps(SID_CON_RW) + json.dumps(ROUTE_arr) + json.dumps(flight_data) + input_Pilot + pilotSays},
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

                ###check if the waypoints in route is the same as the one we generated, if not, replace it with the one we generated
                if(clearance["Waypoints in route"].split() != ROUTE_arr):
                    clearance["Waypoints in route"] = " ".join(ROUTE_arr)
                
                with open(CLEARANCE_PATH, "w", encoding="utf-8") as f:
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
                            #print("[debug] readback 音频秒数:", len(AUDIO) / SAMPLE_RATE)
                            result = speech_model.transcribe(AUDIO, language = "en", initial_prompt="ATC radio communication. User may say alphabet in Alpha Bravo "
                            "Charlie Delta Echo Foxtrot Golf Hotel India Juliett Kilo Lima "
                            "Mike November Oscar Papa Quebec Romeo Sierra Tango Uniform "
                            "Victor Whiskey Xray Yankee Zulu.", condition_on_previous_text= False, fp16=False)
                            pilotSays = result["text"]
                            print("YOU: ", pilotSays)
                            
                            audio_data = []
                    
                    time.sleep(0.02)

                    now = time.time()
                    if(now - last_time >= 90):
                        remindReadback(AIRCRAFT_REG)
                        last_time = time.time()

                with open(CLEARANCE_PATH) as f:
                        clearance_data = json.load(f)
                callsign = clearance_data["callsign"]
                sid = clearance_data["SID"]
                waypoints = clearance_data["Waypoints in route"]
                squawk = clearance_data["squawk"]
                destination = clearance_data["Destination"]
                squawk = clearance_data["squawk"]
                
                if((callsign in pilotSays) or (sid in pilotSays) or (squawk in pilotSays) or 
                   (destination in pilotSays) or (squawk in pilotSays) or ("readback" in pilotSays) or ("roger" in pilotSays)):
                    clr_Readback(AIRCRAFT_REG)
                else:
                    print(AIRCRAFT_REG, ", negative, I repeat, clear to ", destination, " routing as fieled ", waypoints, ", climb via ", sid, 
                          ", squawk ", squawk, ", readback not necessary, contact when ready for departure")
                
                isClearanced.active = True




