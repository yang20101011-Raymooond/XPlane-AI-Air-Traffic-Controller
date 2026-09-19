# ============================================================================
#  AI-ATC → example_plugin  (X-Plane 12 · macOS · 官方 SDK 4.3.0)
#
#  极简可扩展: 把入口 .cpp 加进下面的列表即可继续开发。
#  构建后用 add_custom_command 自动把产物复制成 mac.xpl 部署到 X-Plane。
# ============================================================================
cmake_minimum_required(VERSION 3.16)
project(AIATC LANGUAGES C CXX)

# ---- 官方 X-Plane SDK 4.3.0 (本地 SDK/ 目录) -------------------------------
set(XPLANE_SDK "${CMAKE_CURRENT_SOURCE_DIR}/SDK")
set(XPLANE_INCLUDES
    "${XPLANE_SDK}/CHeaders"
    "${XPLANE_SDK}/CHeaders/XPLM"
    "${XPLANE_SDK}/CHeaders/Widgets"
    "${XPLANE_SDK}/CHeaders/Wrappers"
)

# ---- 目标:X-Plane 12.4+ 需要声明到 XPLM430(且此前全部), 见官方版本宏规则 ----
# 用分号分隔(CMake 列表), 避免整串被当成单个编译器参数。
set(XPLM_VERSION_MACROS
    -DXPLM200=1;-DXPLM210=1;-DXPLM300=1;-DXPLM301=1;-DXPLM303=1
    -DXPLM400=1;-DXPLM411=1;-DXPLM420=1;-DXPLM430=1)

# ---- Mac 标识(官方要求):APL=1 / IBM=0 / LIN=0 ------------------------------
set(PLATFORM_DEFS -DAPL=1;-DIBM=0;-DLIN=0)

# ---- 生成一个 X-Plane 插件目标的“共同模板函数”(避免重复粘贴一大段) --------
# 用法: xp_plugin(<目标名> <部署子目录> <源1> <源2> ...)
# 它会创建 add_executable + 编译选项 + 链接 SDK + POST_BUILD 自动部署成 mac.xpl。
set(XP_PLUGIN_DIR "/Users/aic06/X-Plane 12/Resources/plugins" CACHE PATH
    "X-Plane 12 插件目录(用于自动部署)")
find_library(XPLM_LIB XPLM PATHS "${XPLANE_SDK}/Libraries/Mac" NO_DEFAULT_PATH)
find_library(XPWIDGETS_LIB XPWidgets PATHS "${XPLANE_SDK}/Libraries/Mac" NO_DEFAULT_PATH)

function(xp_plugin NAME SUBDIR)
    # 第3个及之后的参数 = 该插件的源文件列表
    add_executable(${NAME} ${ARGN})
    set_target_properties(${NAME} PROPERTIES
        PREFIX ""
        SUFFIX ""
        OUTPUT_NAME "${NAME}.xpl"
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        VISIBILITY_INLINES_HIDDEN ON
        CXX_VISIBILITY_PRESET hidden
    )
    target_compile_options(${NAME} PRIVATE ${XPLM_VERSION_MACROS} ${PLATFORM_DEFS} -fPIC -fvisibility=hidden)
    target_include_directories(${NAME} PRIVATE ${XPLANE_INCLUDES})
    target_link_libraries(${NAME} PRIVATE -bundle ${XPLM_LIB} ${XPWIDGETS_LIB})
    add_custom_command(TARGET ${NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "${XP_PLUGIN_DIR}/${SUBDIR}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:${NAME}>" "${XP_PLUGIN_DIR}/${SUBDIR}/mac.xpl"
        COMMENT "Deployed mac.xpl → ${XP_PLUGIN_DIR}/${SUBDIR}/"
    )
endfunction()

# ============================================================================
#  声明各个插件目标(每个插件 = 一个独立 mac.xpl 文件夹):
#  - 想加第 3/4… 个插件, 照抄下面两行, 改成你自己的名字+源文件即可。
# ============================================================================
#xp_plugin(example_plugin example_plugin example_plugin.cpp)

# 读取并打印 X-Plane 指示空速(Indicated Airspeed, IAS)的插件
xp_plugin(read_IAS read_IAS read-IAS.cpp)
