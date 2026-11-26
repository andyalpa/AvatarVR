cmake_minimum_required(VERSION 3.20)
project(AvatarVR LANGUAGES C CXX)

add_executable(scan_uv_shader_test tools/tests/scan_uv_shader_test.cpp)
target_compile_features(scan_uv_shader_test PRIVATE cxx_std_20)
target_link_libraries(scan_uv_shader_test PRIVATE d3d12 d3dcompiler dxguid)

if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
  message(FATAL_ERROR "AvatarVR is x64 only")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

set(AVATARVR_BIN ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${AVATARVR_BIN})
foreach(cfg DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
  string(TOUPPER ${cfg} CFG)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CFG} ${AVATARVR_BIN})
endforeach()

add_library(minhook STATIC
  third_party/minhook/src/buffer.c
  third_party/minhook/src/hook.c
  third_party/minhook/src/trampoline.c
  third_party/minhook/src/hde/hde32.c
  third_party/minhook/src/hde/hde64.c)
target_include_directories(minhook PUBLIC third_party/minhook/include)
target_include_directories(minhook PRIVATE third_party/minhook/src third_party/minhook/src/hde)

# Official Khronos OpenXR loader (third_party/openxr-sdk, Apache-2.0), linked statically into AvatarVR.dll
# so no second DLL lands in scripts\ (USM loads every DLL there). Pre-generated sources; no Python needed.
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_API_LAYERS OFF CACHE BOOL "" FORCE)
set(BUILD_CONFORMANCE_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_SDK_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(DYNAMIC_LOADER OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_SYSTEM_JSONCPP OFF CACHE BOOL "" FORCE)
set(BUILD_FORCE_GENERATION OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/openxr-sdk EXCLUDE_FROM_ALL)
# The SDK forces the DLL CRT on its static loader; AvatarVR uses the static CRT, so match it.
set_target_properties(openxr_loader PROPERTIES MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# Dear ImGui 1.91.9b (third_party/imgui, MIT): the in-game VR mod menu (src/ui/vr_menu.cpp), D3D12 backend only.
add_library(imgui STATIC
  third_party/imgui/imgui.cpp
  third_party/imgui/imgui_draw.cpp
  third_party/imgui/imgui_tables.cpp
  third_party/imgui/imgui_widgets.cpp
  third_party/imgui/backends/imgui_impl_dx12.cpp)
target_include_directories(imgui PUBLIC third_party/imgui third_party/imgui/backends)
target_compile_definitions(imgui PUBLIC IMGUI_DISABLE_OBSOLETE_FUNCTIONS UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)

# Lua 5.4.7 (third_party/lua, MIT, lua.org source tarball): in-game scripting (src/script/lua_host.cpp,
# docs/SCRIPTING.md). The interpreter and compiler front ends (lua.c, luac.c) are left out.
file(GLOB LUA_SOURCES third_party/lua/*.c)
list(REMOVE_ITEM LUA_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lua/lua.c ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lua/luac.c)
add_library(lua STATIC ${LUA_SOURCES})
target_include_directories(lua PUBLIC third_party/lua)
if(MSVC)
  target_compile_options(lua PRIVATE /W0)
endif()

add_library(AvatarVR SHARED
  src/dllmain.cpp
  src/dx12/dx12.cpp
  src/camera/camera.cpp
  src/camera/ui_layout.cpp
  src/camera/ui_cursor.cpp
  src/camera/ui_piece.cpp
  src/camera/ui_ownership.cpp
  src/vr/wrist_hud.cpp
  src/camera/eye_write.cpp
  src/camera/time_sync.cpp
  src/camera/native_stereo.cpp
  src/camera/mount_camera.cpp
  src/diagnostics/matrix_detector.cpp
  src/diagnostics/upload_sampler.cpp
  src/diagnostics/eye_probe.cpp
  src/diagnostics/frame_capture.cpp
  src/diagnostics/resource_names.cpp
  src/diagnostics/render_activity.cpp
  src/diagnostics/hitch_probe.cpp
  src/diagnostics/input_probe.cpp
  src/diagnostics/player_probe.cpp
  src/diagnostics/bone_copy_probe.cpp
  src/diagnostics/pose_phase_probe.cpp
  src/diagnostics/move_probe.cpp
  src/vr/vr_weapon.cpp
  src/vr/vr_recoil.cpp
  src/vr/bow_draw.cpp
  src/vr/reload_probe.cpp
  src/vr/manual_reload.cpp
  src/vr/vr_arms.cpp
  src/vr/vr_controls.cpp
  src/vr/hand_gestures.cpp
  src/diagnostics/skin_palette_probe.cpp
  src/game/player.cpp
  src/game/mount_state.cpp
  src/script/lua_host.cpp
  src/vr/xr_runtime.cpp
  src/vr/xess_twin.cpp
  src/vr/dlss_twin.cpp
  src/vr/mv_repair.cpp
  src/vr/xr_depth.cpp
  src/vr/frame_gen.cpp
  src/vr/virtual_pad.cpp
  src/vr/xr_input.cpp
  src/ui/vr_menu.cpp
  src/ui/monitor_preview.cpp)
target_compile_definitions(AvatarVR PRIVATE UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)
target_link_libraries(AvatarVR PRIVATE bcrypt dwmapi ole32 oleaut32 windowscodecs minhook dxgi d3d12 d3dcompiler imgui lua OpenXR::openxr_loader)
if(MSVC)
  target_compile_options(AvatarVR PRIVATE /W4 /EHsc)
endif()

add_executable(load_proof_host src/load_proof_host.cpp)
add_executable(gpu_scheduling_probe tools/perf/gpu_scheduling_probe.cpp)
target_link_libraries(gpu_scheduling_probe PRIVATE dxgi gdi32)
add_executable(settings_file_test tools/tests/settings_file_test.cpp)
target_compile_definitions(settings_file_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
add_executable(lens_projection_test tools/tests/lens_projection_test.cpp)
add_executable(ui_canvas_test tools/tests/ui_canvas_test.cpp)
add_executable(ui_cursor_test tools/tests/ui_cursor_test.cpp)
add_executable(menu_pointer_test tools/tests/menu_pointer_test.cpp)
add_executable(menu_transition_test tools/tests/menu_transition_test.cpp)
add_executable(xr_depth_test tools/tests/xr_depth_test.cpp)
add_executable(frame_gen_test tools/tests/frame_gen_test.cpp)
add_executable(ui_layout_test tools/tests/ui_layout_test.cpp)
add_executable(skin_palette_pulse_test tools/tests/skin_palette_pulse_test.cpp)
add_executable(skin_arm_ik_test tools/tests/skin_arm_ik_test.cpp)
add_executable(vr_weapon_test tools/tests/vr_weapon_test.cpp)
add_executable(vr_recoil_test tools/tests/vr_recoil_test.cpp)
add_executable(vr_arms_test tools/tests/vr_arms_test.cpp)
add_executable(vr_controls_test tools/tests/vr_controls_test.cpp)
add_executable(bow_draw_test tools/tests/bow_draw_test.cpp)
add_executable(manual_reload_test tools/tests/manual_reload_test.cpp)
add_executable(hand_gestures_test tools/tests/hand_gestures_test.cpp)
add_executable(xr_input_gestures_test tools/tests/xr_input_gestures_test.cpp)
target_include_directories(xr_input_gestures_test PRIVATE third_party/openxr-sdk/include)
target_compile_definitions(xr_input_gestures_test PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
add_executable(mount_camera_capture_test tools/tests/mount_camera_capture_test.cpp)
add_executable(mount_anchor_test tools/tests/mount_anchor_test.cpp)
add_executable(rider_ik_test tools/tests/rider_ik_test.cpp)
add_executable(mv_repair_test tools/tests/mv_repair_test.cpp)
target_link_libraries(mv_repair_test PRIVATE d3d12 dxgi d3dcompiler)
target_compile_definitions(load_proof_host PRIVATE UNICODE _UNICODE WIN32_LEAN_AND_MEAN NOMINMAX)
if(MSVC)
  target_compile_options(load_proof_host PRIVATE /W4 /EHsc)
endif()

# Only self-contained checks belong in the default test suite.
enable_testing()
add_test(NAME load_proof COMMAND load_proof_host)
foreach(test_target IN ITEMS
    settings_file_test
    lens_projection_test
    ui_canvas_test
    ui_cursor_test
    menu_pointer_test
    menu_transition_test
    xr_depth_test
    frame_gen_test
    ui_layout_test
    skin_palette_pulse_test
    skin_arm_ik_test
    vr_weapon_test
    vr_recoil_test
    vr_arms_test
    vr_controls_test
    bow_draw_test
    manual_reload_test
    hand_gestures_test
    xr_input_gestures_test
    mount_camera_capture_test
    mount_anchor_test
    rider_ik_test)
  add_test(NAME ${test_target} COMMAND ${test_target})
endforeach()
