#define APL 1

#include "XPLMPlugin.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMProcessing.h"
#include "XPLMNavigation.h"

#include <cstring>
#include <cstdlib>//getenv
#include <stdio.h>//snprintf
#include <fstream>
using namespace std;

// 共享数据文件所在目录（flight_data.json / flight_debug.log）。
// 优先级：环境变量 AIATC_DIR > 编译期默认值（CMake 传入）> 当前目录。
// 换机器时：启动 X-Plane 前 export AIATC_DIR=/你的/路径/AI-ATC，或重新执行 ./bootstrap.sh 重编。
//You don't need to export AIATC_DIR if ALL of you files are together with CMakeList.txt under AI-ATC(in system path recommended)
//since cmake will give the path afer running bootstrap.sh
#ifndef AIATC_DIR_DEFAULT
#define AIATC_DIR_DEFAULT "."
#endif

//if AIATC_DIR_DEFAULT is empty, aiactcDir will be automatically AIATC_DIR, which is set in Cmake, the AI-ATC folder path
static const char *aiactcDir()
{
    const char *env = getenv("AIATC_DIR");
    return (env && env[0]) ? env : AIATC_DIR_DEFAULT;
}
static string aiactcPath(const char *filename)
{
    string dir = aiactcDir();
    if(!dir.empty() && dir.back() != '/') dir += '/';
    return dir + filename;
}

XPLMDataRef ias_dataref = NULL;
XPLMDataRef hdg_dataref = NULL;
XPLMDataRef alt_dataref = NULL;
XPLMDataRef agl_dataref = NULL;
XPLMDataRef vertical_speed_dataref = NULL;
XPLMDataRef latitude_dataref = NULL;
XPLMDataRef longitude_dataref = NULL;
XPLMDataRef wind_speed_dataref = NULL;
XPLMDataRef wind_heading_dataref = NULL;
XPLMDataRef qnh_dataref = NULL;
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
    agl_dataref = XPLMFindDataRef("sim/flightmodel/position/y_agl");
    vertical_speed_dataref = XPLMFindDataRef("sim/cockpit2/gauges/indicators/vvi_fpm_pilot");
    wind_speed_dataref = XPLMFindDataRef("sim/cockpit2/gauges/indicators/wind_speed_kts");
    wind_heading_dataref = XPLMFindDataRef("sim/cockpit2/gauges/indicators/wind_heading_deg_mag");
    latitude_dataref = XPLMFindDataRef("sim/flightmodel/position/latitude");
    longitude_dataref = XPLMFindDataRef("sim/flightmodel/position/longitude");
    qnh_dataref = XPLMFindDataRef("sim/weather/aircraft/qnh_pas");
    aircraft_Registration_dataref = XPLMFindDataRef("sim/aircraft/view/acf_tailnum");

    XPLMDebugString("[readIAS] XPluginStart called.\n");
 
    if(ias_dataref == NULL || hdg_dataref == NULL || alt_dataref == NULL || agl_dataref == NULL 
        || vertical_speed_dataref == NULL || latitude_dataref == NULL || longitude_dataref == NULL || wind_speed_dataref == NULL || wind_heading_dataref == NULL|| aircraft_Registration_dataref == NULL) {
        if(ias_dataref == NULL) {
            XPLMDebugString("[readIAS] Failed to find dataref for IAS.\n");
        }
        if(hdg_dataref == NULL) {
            XPLMDebugString("[readIAS] Failed to find dataref for HDG.\n");
        }
        if(alt_dataref == NULL) {
            XPLMDebugString("[readIAS] Failed to find dataref for ALT.\n");
        }
        if(agl_dataref == NULL){
            XPLMDebugString("[readIAS] Failed to find dataref for ElEVATION.\n");
        }
        if(vertical_speed_dataref == NULL){
            XPLMDebugString("[readIAS] Failed to find dataref for VERTICAL SPEED.\n");
        }
        if(latitude_dataref == NULL) {
            XPLMDebugString("[readIAS] Failed to find dataref for LAT.\n");
        }
        if(longitude_dataref == NULL) {
            XPLMDebugString("[readIAS] Failed to find dataref for LON.\n");
        }
        if(wind_speed_dataref == NULL){
            XPLMDebugString("[readIAS] Failed to find dataref for WINDSPD.\n");
        }
        if(wind_heading_dataref == NULL){
            XPLMDebugString("[readIAS] Failed to find dataref for WINDHDG.\n");
        }
        if(qnh_dataref == NULL){
            XPLMDebugString("[readIAS] Failed to find dataref for QNH.\n");
        }
        if(aircraft_Registration_dataref == NULL){
            XPLMDebugString("[readIAS] Failed to find dataref for REG.\n");
        }
        return 0; // failed to initialize the plugin
    } 
    
    //Functions registered at FlightloopCallback will be called every frame in xplane main;
    XPLMRegisterFlightLoopCallback(ReadAllCallback, 1.0f, NULL); // Register the flight loop callback to read all data every frame
    return 1; // successfully initialized the plugin
}

float ReadAllCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void *inRefcon)
{
    float IAS, HDG, ALT, AGL, WINDSPD, WINDHDG, QNH, VERTICALSPEED;
    double LATITUDE, LONGITUDE;
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
    if(agl_dataref != NULL){
        AGL = XPLMGetDataf(agl_dataref);//get above ground level from dataref IN METERS
    }
    if(vertical_speed_dataref != NULL){
        VERTICALSPEED = XPLMGetDataf(vertical_speed_dataref);//get vertical speed from dataref
    }
    if(latitude_dataref != NULL) {
        LATITUDE = XPLMGetDataf(latitude_dataref);//get latitude from dataref
    }
    if(longitude_dataref != NULL) {
        LONGITUDE = XPLMGetDataf(longitude_dataref);//get longitude from dataref
    }
    if(wind_speed_dataref != NULL){
        WINDSPD = XPLMGetDataf(wind_speed_dataref);//get windspeed from dataref
    }
    if(wind_heading_dataref != NULL){
        WINDHDG = XPLMGetDataf(wind_heading_dataref);//get windheading from dataref
    }
    if(qnh_dataref != NULL){
        QNH = XPLMGetDataf(qnh_dataref);//get qnh from dataref
    }
    if(aircraft_Registration_dataref != NULL) {
        XPLMGetDatab(aircraft_Registration_dataref, aircraft_Registration, 0, sizeof(aircraft_Registration));//get aircraft registration from dataref
        //this is used to read strings datarefs, the 0 is the offset, and sizeof(aircraft_Registration) is the bytes to read
        //these bytes of char is written into char array that stores aircraft registration;
        //sizeof(aircraft_Registration) = 40;
    }

    char json_buffer[1024];
    snprintf(json_buffer, sizeof(json_buffer), 
    "{\n"
    "  \"indicated-airspeed\": %.2f,\n"
    "  \"heading\": %.2f,\n"
    "  \"altitude\": %.2f,\n"
    "  \"above ground level\": %.2f,\n"
    "  \"vertical speed\": %.2f,\n"
    "  \"latitude\": %.2f,\n"
    "  \"longitude\": %.2f,\n"
    "  \"wind speed\": %.2f,\n"
    "  \"wind heading\": %.2f,\n"
    "  \"QNH\": %.2f,\n"
    "  \"aircraft-registration\": \"%s\"\n"
    "}\n", 
    IAS, HDG, ALT, AGL, VERTICALSPEED, LATITUDE, LONGITUDE, WINDSPD, WINDHDG, QNH, aircraft_Registration);
        
    //write to json file
    ofstream flight_data(aiactcPath("flight_data.json").c_str());
    if (flight_data.is_open() == true){
        flight_data << json_buffer;
        flight_data.close();
    } else {
        XPLMDebugString("[readIAS] Failed to open flight_data.json for writing.\n");
    }
    
    //Debug
    XPLMDebugString(json_buffer);
    ofstream debug_file(aiactcPath("flight_debug.log").c_str(), ios::app);
    if(debug_file.is_open() == true){
        debug_file << json_buffer;
        debug_file.close();
    } else {
        XPLMDebugString("[readIAS] Failed to open debug_log.txt for writing.\n");
    }

    return 5.0f; // Return 5.0f to indicate that callback should be called again in 5 seconds;
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
