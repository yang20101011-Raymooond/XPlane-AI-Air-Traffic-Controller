import json
import os
import math
import route_finder as rf

def get_current_airway(airlat, airlon, clearancejsPATH):
    with open(clearancejsPATH) as f:
        clearance = json.load(f)
        waypoints_in_route_str = clearance["Waypoints in route"]

    waypoints_in_route = waypoints_in_route_str.split() if isinstance(waypoints_in_route_str, str) else list(waypoints_in_route_str)
    waypoints_in_route.pop(-1)##remove destination airport first
    
    ##create array to store coordinates of waypoints
    ##call _node_coord to get waypoints, the near should be None first, after getting coor of airport, it becomes the previous point
    waypoints_coor = []
    prev_coor = None
    for i in waypoints_in_route:
        coordinate = rf._node_coord(i, near=prev_coor)
        waypoints_coor.append(coordinate)##even if none, append
        if(coordinate):
            prev_coor = coordinate

    waypoints_coor.pop(0)##remove origin airport
    waypoints_in_route.pop(0)

    AIRWAY = []#store airway A-B in such form:
    """ [
            [ori-icao, dest-icao, ori-coor, dest-coor], 
            [ori-icao, dest-icao, ori-coor, dest-coor],
            [ori-icao, dest-icao, ori-coor, dest-coor],
        ]
    where ori-coor, dest-coor are in (lat, lon)
    """
    for i in range(len(waypoints_in_route)-1):
        ori_icao = waypoints_in_route[i]
        dest_icao = waypoints_in_route[i+1]
        ori_coor = waypoints_coor[i]
        dest_coor = waypoints_coor[i+1]
        AIRWAY.append([ori_icao, dest_icao, ori_coor, dest_coor])

    current_airway = None
    min_distance = 1e999
    for awy in AIRWAY:
        if awy[2] is None or awy[3] is None:
            continue#defensive 
        lat1, lon1 = awy[2][0], awy[2][1]
        lat2, lon2 = awy[3][0], awy[3][1]
        along_trk_distance = rf._along_track(airlat, airlon, lat1, lon1, lat2, lon2)##find the component along the track of the displacement from ori 
        L = math.sqrt(
            (lat2-lat1)**2 + ((lon2-lon1)*math.cos(math.radians((lat2+lat1)/2)))**2
            )*60

        if(along_trk_distance < 0):##even before ori
            perpendic_distance = math.sqrt((airlat-lat1)**2 + ((airlon-lon1)*math.cos(math.radians((airlat+lat1)/2)))**2)*60
        
        elif(along_trk_distance > L):##already after dest
            perpendic_distance = math.sqrt((airlat-lat2)**2 + ((airlon-lon2)*math.cos(math.radians((airlat+lat2)/2)))**2)*60
        
        else:
            perpendic_distance = math.sqrt(
            max(
            ((airlat-lat1)*60)**2 + ((airlon-lon1)*math.cos(math.radians((lat1+lat2)/2))*60)**2 - along_trk_distance**2
            , 0)
            )##average latidute must be consistent with the one used in alongtrack
            ##the perpendicular component, max(,0) deal with negative round-off error
        if(perpendic_distance < min_distance):
            min_distance = perpendic_distance
            current_airway = awy

    return current_airway, min_distance, AIRWAY
        
def checkoffcourse(airlat, airlon, clearancejsPATH):
    current_airway, perpendic_offset, n= get_current_airway(airlat, airlon, clearancejsPATH)
    if(perpendic_offset > 4):
        return True
    
    else:
        return False

def decideDirectto(airlat, airlon, clearancejsPATH, REGISTRATION):
    current_airway, perpendic_offset, AIRWAY= get_current_airway(airlat, airlon, clearancejsPATH)
    if(current_airway is None):
        return##defensive
    dest_icao = current_airway[1]
    dest_coor = current_airway[3]
    d_aft_dest = rf._nm_distance(dest_coor[0], dest_coor[1], airlat, airlon)

    index = AIRWAY.index(current_airway)

    if(d_aft_dest >= 18):
        print(REGISTRATION, ", you are off course, fly direct ", dest_icao)
    elif(index < len(AIRWAY)-1):
        next_icao = AIRWAY[index+1][1]
        print(REGISTRATION, ", you are off course, fly direct ", next_icao)
    else:
        print(REGISTRATION, ", you are off course, fly direct ", dest_icao, " and prepare "
        "for Standered Instrument Arrival")

    return


    