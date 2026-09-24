#define APL 1

#include "XPLMPlugin.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMProcessing.h"

#include <cstring>
#include <stdio.h> //snprintf
#include <fstream>
#include <cstdlib>
#include <filesystem>
#include <string>
using namespace std;

XPLMDataRef ias_dataref = NULL;
XPLMDataRef hdg_dataref = NULL;
XPLMDataRef alt_dataref = NULL;
XPLMDataRef elevation_dataref = NULL;
XPLMDataRef latitude_dataref = NULL;
XPLMDataRef longitude_dataref = NULL;
XPLMDataRef aircraft_Registration_dataref = NULL;

float ReadAllCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void *inRefcon);
    
PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc)
{
    strcpy(outName, "Read IAS");
    strcpy(outSig,  "com.raymond.readIAS");
    strcpy(outDesc, "Read IAS from X-Plane 12.");

    //set up the dataref for indicated airspeed (IAS) using XPLMFindDataRef
    //pointer to the dataref is stored in ias_dataref for later use in the flight loop callback
    ias_dataref = XPLMFindDataRef("sim/flightmodel/position/indicated_airspeed");
    hdg_dataref = XPLMFindDataRef("sim/flightmodel/position/mag_psi");
    alt_dataref = XPLMFindDataRef("sim/cockpit2/gauges/indicators/altitude_ft_pilot");
    elevation_dataref = XPLMFindDataRef("sim/flightmodel/position/elevation");
    latitude_dataref = XPLMFindDataRef("sim/flightmodel/position/latitude");
    longitude_dataref = XPLMFindDataRef("sim/flightmodel/position/longitude");
    aircraft_Registration_dataref = XPLMFindDataRef("sim/aircraft/view/acf_tailnum");

    XPLMDebugString("[readIAS] XPluginStart called.\n");

    if(ias_dataref == NULL || hdg_dataref == NULL || alt_dataref == NULL || elevation_dataref == NULL || latitude_dataref == NULL || longitude_dataref == NULL || aircraft_Registration_dataref == NULL) {
        if(ias_dataref == NULL) {
            XPLMDebugString("[readIAS] Failed to find dataref for IAS.\n");
        }
        if(hdg_dataref == NULL) {
            XPLMDebugString("[readIAS] Failed to find dataref for HDG.\n");
        }
        if(alt_dataref == NULL) {
            XPLMDebugString("[readIAS] Failed to find dataref for ALT.\n");
        }
        if(elevation_dataref == NULL){
            XPLMDebugString("[readIAS] Failed to find dataref for Elevation.\n");
        }
        if(latitude_dataref == NULL) {
            XPLMDebugString("[readIAS] Failed to find dataref for LAT.\n");
        }
        if(longitude_dataref == NULL) {
            XPLMDebugString("[readIAS] Failed to find dataref for LON.\n");
        }
        if(aircraft_Registration_dataref == NULL){
            XPLMDebugString("[readIAS] Failed to find dataref for .\n");
        }
        return 0; // failed to initialize the plugin
    } 
    
    //Functions registered at FlightloopCallback will be called every frame in xplane main;
    XPLMRegisterFlightLoopCallback(ReadAllCallback, -1.0f, NULL); // Register the flight loop callback to read all data every frame
    return 1; // successfully initialized the plugin
}

float ReadAllCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void *inRefcon)
{
    float IAS, HDG, ALT;
    double LATITUDE, LONGITUDE, ELEVATION;
    char aircraft_Registration[40];//char array to hold the aircraft registration string, Registration dataref is 40 bytes long.

    if(ias_dataref != NULL) {
        IAS = XPLMGetDataf(ias_dataref);//get indicated airspeed from dataref
    }
    if(hdg_dataref != NULL) {
        HDG = XPLMGetDataf(hdg_dataref);//get heading from dataref
    }
    if(alt_dataref != NULL) {
        ALT = XPLMGetDataf(alt_dataref);//get altitude from dataref
    }
    if(elevation_dataref != NULL){
        ELEVATION = XPLMGetDataf(elevation_dataref);
    }
    if(latitude_dataref != NULL) {
        LATITUDE = XPLMGetDataf(latitude_dataref);//get latitude from dataref
    }
    if(longitude_dataref != NULL) {
        LONGITUDE = XPLMGetDataf(longitude_dataref);//get longitude from dataref
    }
    if(aircraft_Registration_dataref != NULL) {
        XPLMGetDatab(aircraft_Registration_dataref, aircraft_Registration, 0, sizeof(aircraft_Registration));//get aircraft registration from dataref
        //this is used to read strings datarefs, the 0 is the offset, and sizeof(aircraft_Registration) is the bytes to read
        //these bytes of char is written into char array that stores aircraft registration;
        //sizeof(aircraft_Registration) = 40;
    }

    char json_buffer[256];
    snprintf(json_buffer, sizeof(json_buffer), 
    "{\n"
    "  \"indicated-airspeed\": %.2f,\n"
    "  \"heading\": %.2f,\n"
    "  \"altitude\": %.2f,\n"
    "  \"elevation\": %.2f,\n"
    "  \"latitude\": %.2f,\n"
    "  \"longitude\": %.2f,\n"
    "  \"aircraft-registration\": \"%s\"\n"
    "}\n", 
    IAS, HDG, ALT, ELEVATION, LATITUDE, LONGITUDE, aircraft_Registration);
    
    const char* home = std::getenv("HOME"); // Get the user's home directory

    if(home == nullptr) {    // Judge if home directory is gotten
        XPLMDebugString("[readIAS] Failed to get HOME directory.\n");
        return 1.0f;
    }

    std::filesystem::path ai_atc_dir = std::filesystem::path(home) / "AI-ATC"; 

    if(!std::filesystem::exists(ai_atc_dir)) { //If do not exist
        if(!std::filesystem::create_directories(ai_atc_dir)) { // Then do it
        XPLMDebugString("[readIAS] Failed to create AI-ATC directory.\n"); // For some reason the creation is failed : (
        // return 1.0f;
    }
    XPLMDebugString("[readIAS] AI-ATC directory created.\n");
    }

    std::filesystem::permissions( // Grant permissions needed for the user of the directory, "~/AI-ATC".
        ai_atc_dir,
        std::filesystem::perms::owner_all |
        std::filesystem::perms::group_read |
        std::filesystem::perms::group_exec |
        std::filesystem::perms::others_read |
        std::filesystem::perms::others_exec,
        std::filesystem::perm_options::replace
    );
    
    //write to json file 
    ofstream flight_data(ai_atc_dir / "flight_data.json"); //To Raymond: Code logic below remains unchanged but the directory processing logic. No more shit mountain.(i guess)
    if (flight_data.is_open() == true){
        flight_data << json_buffer;
        flight_data.close();
    } else {
        XPLMDebugString("[readIAS] Failed to open flight_data.json for writing.\n");
    }
    
    //Debug
    XPLMDebugString(json_buffer);
    ofstream debug_file(ai_atc_dir / "flight_debug.log", ios::app);
    if(debug_file.is_open() == true){
        debug_file << json_buffer;
        debug_file.close();
    } else {
        XPLMDebugString("[readIAS] Failed to open debug_log.txt for writing.\n");
    }

    return 1.0f; // Return -1.0f to indicate that callback should be called again in 1 second;
}


PLUGIN_API void XPluginStop(void)
{
    XPLMDebugString("[readIAS] XPluginStop called.\n");
    XPLMUnregisterFlightLoopCallback(ReadAllCallback, NULL); // Unregister the flight loop callback when the plugin is stopped
}

PLUGIN_API int XPluginEnable(void)
{
    XPLMDebugString("[readIAS] XPluginEnable called.\n");
    return 1;
}

PLUGIN_API void XPluginDisable(void)
{
    XPLMDebugString("[readIAS] XPluginDisable called.\n");
}

PLUGIN_API void XPluginReceiveMessage(
        XPLMPluginID inFromWho,
        intptr_t     inMessage,
        void *       inParam)
{
    if (inMessage == XPLM_MSG_PLANE_LOADED) {
        XPLMDebugString("[readIAS] A plane was loaded.\n");
    }
    //XPLM_MSG_PLANE_LOADED is a int (constant) defined in XPLMPLugin.h
    //indicates that a plane has been loaded in X-Plane
    //main will call this function and put in the status of plane, if a plane is loaded, it will put inMessage = XPLM_MSG_PLANE_LOADED
}
