// newplugin2.cpp — 演示用的第二个插件(独立 XPluginStart)
#include "XPLMPlugin.h"
#include "XPLMUtilities.h"
#include <cstring>

PLUGIN_API int XPluginStart(char *oN, char *oS, char *oD)
{
    strcpy(oN, "My New Plugin 2");                // 名称
    strcpy(oS, "com.raymond.newplugin2");        // 签名(唯一)
    strcpy(oD, "A second, separate plugin.");
    XPLMDebugString("[newplugin2] XPluginStart.\n");
    return 1;
}
PLUGIN_API void XPluginStop(void){}
PLUGIN_API int  XPluginEnable(void){ return 1; }
PLUGIN_API void XPluginDisable(void){}
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID f,intptr_t m,void*){(void)f;(void)m;}
