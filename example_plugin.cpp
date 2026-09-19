#define APL 1

// 这是一个最简单的 X-Plane 插件示例, 仅用于演示如何使用官方 SDK 4.3.0 编译插件。
//由Claw生成以供学习
// XPLM 是 C API, PLUGIN_API / XPLM_API / PLUGIN_API 等宏在 XPLMDefs.h 定义。
#include "XPLMPlugin.h"      // XPluginStart... / 消息常量
#include "XPLMUtilities.h"   // XPLMDebugString (日志)

#include <cstring>           // strcpy

// ---------------------------------------------------------------------------
//  XPluginStart
//  返回 1 = 插件成功初始化。名称/签名/描述会显示在 X-Plane 的插件管理里。
// ---------------------------------------------------------------------------
PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc)
{
    strcpy(outName, "Hello X-Plane (example_plugin)");
    strcpy(outSig,  "com.raymond.example_plugin");
    strcpy(outDesc, "First X-Plane 12 plugin built with the official SDK 4.3.0.");

    // 写日志: 能被 X-Plane 写入 Log.txt。
    XPLMDebugString("[example_plugin] XPluginStart called (SDK 4.3.0).\n");

    return 1;   // 1 = 初始化成功, 0 = 失败(卸载插件)
}

// XPluginStop   —— X-Plane 退出或被移除时调用(清理资源)。
PLUGIN_API void XPluginStop(void)
{
    XPLMDebugString("[example_plugin] XPluginStop called.\n");
}

// XPluginEnable —— 插件被启用(通常在 Start 之后)。
//                返回 1 = 接受启用。
PLUGIN_API int XPluginEnable(void)
{
    XPLMDebugString("[example_plugin] XPluginEnable called.\n");
    return 1;
}

// XPluginDisable —— 插件被禁用(清空在 Enable 时建立的东西)。
PLUGIN_API void XPluginDisable(void)
{
    XPLMDebugString("[example_plugin] XPluginDisable called.\n");
}

// XPluginReceiveMessage —— 接收来自其它插件或 X-Plane 的系统消息。
PLUGIN_API void XPluginReceiveMessage(
        XPLMPluginID inFromWho,
        intptr_t     inMessage,
        void *       inParam)
{//XPLMPluginID 是一个整数类型, 用于标识插件。intptr_t 是一个整数类型, 用于传递指针或整数参数。
    // XPLM_MSG_PLANE_LOADED = 一架飞机被载入(这是后续 AI-ATC 读取飞行状态的时机)。
    if (inMessage == XPLM_MSG_PLANE_LOADED) {
        XPLMDebugString("[example_plugin] A plane was loaded.\n");
    }
}