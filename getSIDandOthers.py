import os
import random
import time


##get_SID function is to get the possible SID for the departure airport and the SID endpoint, return a list of possible SID
##possible SID are determined by route, the second waypoint in the route is the SID endpoint
##by comparing the SID endpoint with the SID endpoint in the CIFP data, we can get the possible SID for the departure airport and the SID endpoint
def get_SID(icao, sid_endpoint,cifp_path):
    if(not icao or not sid_endpoint):
        return []

    #key = f"{icao}|{sid_endpoint}"
    #if(key in SID_cache):
       # return SID_cache[key]
    
    path = os.path.join(cifp_path, f"{icao.upper()}.dat")
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
                    #rw_name = arr[3].strip()##simialr reason
                    end_point = arr[4].strip()
                    if(end_point == sid_endpoint):
                        sids.add(sid_name)

        except OSError:
            return []
    result = sorted(sids)
    #SID_cache[key] = result
    return result

##match_sid_rw function is to match the possible SID with the runway, return a dictionary of SID and runway
##by comparing the possible SID with the SID in the CIFP data, we can get the runway for the possible SID
def match_sid_rw(possible_sids, cifp_path, dep_icao):
    sid_to_runway = {}
    path = os.path.join(cifp_path, f"{dep_icao.upper()}.dat")
    
    for index in possible_sids:
        SID = index
        with open (path, encoding="utf-8", errors="ignore") as f:
            for i in f:
                arr = i.split(",")
                if(len(arr) < 4):
                    continue
                sid_name = arr[2].strip()
                if(sid_name == SID):
                    rw_name = arr[3].strip()
                    sid_to_runway[SID] = rw_name
                    break

    return sid_to_runway


