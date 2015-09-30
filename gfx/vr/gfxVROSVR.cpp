/* -*- Mode: C++; tab-width: 20; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include <math.h>

#include "prlink.h"
#include "prmem.h"
#include "prenv.h"
#include "gfxPrefs.h"
#include "nsString.h"
#include "mozilla/Preferences.h"

#include "mozilla/gfx/Quaternion.h"

#ifdef XP_WIN
#include "../layers/d3d11/CompositorD3D11.h"
#include "../layers/d3d11/TextureD3D11.h"
#endif

#include "gfxVROSVR.h"

#include "nsServiceManagerUtils.h"
#include "nsIScreenManager.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace mozilla::layers;
using namespace mozilla::gfx;
using namespace mozilla::gfx::impl;

namespace {
// need to typedef functions that will be used in the code below
extern "C"{
typedef OSVR_ClientContext(*pfn_osvrClientInit) (const char applicationIdentifier[], 
                                                 uint32_t flags);
typedef OSVR_ReturnCode(*pfn_osvrClientShutdown)(OSVR_ClientContext ctx);
typedef OSVR_ReturnCode(*pfn_osvrClientUpdate)(OSVR_ClientContext ctx);
typedef OSVR_ReturnCode(*pfn_osvrClientGetInterface)(OSVR_ClientContext ctx, 
                                                     const char path[],
                                                     OSVR_ClientInterface *iface);
typedef OSVR_ReturnCode(*pfn_osvrClientFreeInterface)(OSVR_ClientContext ctx,
                                                      OSVR_ClientInterface iface);
typedef OSVR_ReturnCode(*pfn_osvrGetOrientationState)(OSVR_ClientInterface iface, 
                                                      OSVR_TimeValue *timestamp, 
                                                      OSVR_OrientationState *state);
typedef OSVR_ReturnCode(*pfn_osvrGetPositionState)(OSVR_ClientInterface iface,
                                                   OSVR_TimeValue *timestamp,
                                                   OSVR_PositionState *state);
typedef OSVR_ReturnCode(*pfn_osvrClientGetDisplay)(OSVR_ClientContext ctx, 
                                                   OSVR_DisplayConfig *disp);
typedef OSVR_ReturnCode(*pfn_osvrClientFreeDisplay)(OSVR_DisplayConfig disp);
typedef OSVR_ReturnCode(*pfn_osvrClientGetNumEyesForViewer)(
                                                    OSVR_DisplayConfig disp, 
                                                    OSVR_ViewerCount viewer, 
                                                    OSVR_EyeCount *eyes);
typedef OSVR_ReturnCode(*pfn_osvrClientGetViewerEyePose)(OSVR_DisplayConfig disp, 
                                                         OSVR_ViewerCount viewer,
                                                         OSVR_EyeCount eye, 
                                                         OSVR_Pose3 *pose);
typedef OSVR_ReturnCode(*pfn_osvrClientGetDisplayDimensions)(
                                    OSVR_DisplayConfig disp, 
                                    OSVR_DisplayInputCount displayInputIndex, 
                                    OSVR_DisplayDimension *width, 
                                    OSVR_DisplayDimension *height);
typedef OSVR_ReturnCode(
  *pfn_osvrClientGetViewerEyeSurfaceProjectionClippingPlanes)(OSVR_DisplayConfig disp, 
                                                              OSVR_ViewerCount viewer, 
                                                              OSVR_EyeCount eye, 
                                                              OSVR_SurfaceCount surface, 
                                                              double *left, double *right, 
                                                              double *bottom, double *top);
}

static pfn_osvrClientInit osvr_ClientInit = nullptr;
static pfn_osvrClientShutdown osvr_ClientShutdown = nullptr;
static pfn_osvrClientUpdate osvr_ClientUpdate = nullptr;
static pfn_osvrClientGetInterface osvr_ClientGetInterface = nullptr;
static pfn_osvrClientFreeInterface osvr_ClientFreeInterface = nullptr;
static pfn_osvrGetOrientationState osvr_GetOrientationState = nullptr;
static pfn_osvrGetPositionState osvr_GetPositionState = nullptr;
static pfn_osvrClientGetDisplay osvr_ClientGetDisplay = nullptr;
static pfn_osvrClientFreeDisplay osvr_ClientFreeDisplay = nullptr;
static pfn_osvrClientGetNumEyesForViewer osvr_ClientGetNumEyesForViewer = nullptr;
static pfn_osvrClientGetViewerEyePose osvr_ClientGetViewerEyePose = nullptr;
static pfn_osvrClientGetDisplayDimensions osvr_ClientGetDisplayDimensions = nullptr;
static pfn_osvrClientGetViewerEyeSurfaceProjectionClippingPlanes 
          osvr_ClientGetViewerEyeSurfaceProjectionClippingPlanes = nullptr;

bool
LoadOSVRRuntime()
{
  static PRLibrary *osvrUtilLib = nullptr;
  static PRLibrary *osvrCommonLib = nullptr;
  static PRLibrary *osvrClientLib = nullptr;
  static PRLibrary *osvrClientKitLib = nullptr;
  //this looks up the path in the about:config setting, from greprefs.js or modules\libpref\init\all.js
  nsAdoptingCString osvrUtilPath = mozilla::Preferences::GetCString("gfx.vr.osvr.utilLibPath");
  nsAdoptingCString osvrCommonPath = mozilla::Preferences::GetCString("gfx.vr.osvr.commonLibPath");
  nsAdoptingCString osvrClientPath = mozilla::Preferences::GetCString("gfx.vr.osvr.clientLibPath");
  nsAdoptingCString osvrClientKitPath = mozilla::Preferences::GetCString("gfx.vr.osvr.clientKitLibPath");
  
  //we need all the libs to be valid
  if ((!osvrUtilPath) || (!osvrCommonPath) || 
      (!osvrClientPath) || (!osvrClientKitPath)){
    return false;
  }

  osvrUtilLib = PR_LoadLibrary(osvrUtilPath.BeginReading());
  osvrCommonLib = PR_LoadLibrary(osvrCommonPath.BeginReading());
  osvrClientLib = PR_LoadLibrary(osvrClientPath.BeginReading());
  osvrClientKitLib = PR_LoadLibrary(osvrClientKitPath.BeginReading());

  if (!osvrUtilLib) {
    printf_stderr("Failed to load OSVR Util library!\n");
    return false;
  }
  if (!osvrCommonLib) {
    printf_stderr("Failed to load OSVR Common library!\n");
    return false;
  }
  if (!osvrClientLib) {
    printf_stderr("Failed to load OSVR Client library!\n");
    return false;
  }
  if (!osvrClientKitLib) {
    printf_stderr("Failed to load OSVR ClientKit library!\n");
    return false;
  }

  // make sure all functions that we'll be using are available
#define REQUIRE_FUNCTION(_x) do { \
    *(void **)&osvr_##_x = (void *) PR_FindSymbol(osvrClientKitLib, "osvr" #_x);                \
    if (!osvr_##_x) { printf_stderr("osvr" #_x " symbol missing\n"); goto fail; }       \
      } while (0)

  REQUIRE_FUNCTION(ClientInit);
  REQUIRE_FUNCTION(ClientShutdown);
  REQUIRE_FUNCTION(ClientUpdate);
  REQUIRE_FUNCTION(ClientGetInterface);
  REQUIRE_FUNCTION(ClientFreeInterface);
  REQUIRE_FUNCTION(GetOrientationState);
  REQUIRE_FUNCTION(GetPositionState);
  REQUIRE_FUNCTION(ClientGetDisplay);
  REQUIRE_FUNCTION(ClientFreeDisplay);
  REQUIRE_FUNCTION(ClientGetNumEyesForViewer);
  REQUIRE_FUNCTION(ClientGetViewerEyePose);
  REQUIRE_FUNCTION(ClientGetDisplayDimensions);
  REQUIRE_FUNCTION(ClientGetViewerEyeSurfaceProjectionClippingPlanes);

#undef REQUIRE_FUNCTION

  return true;

 fail:
  return false;
}

} // namespace

VRFieldOfView
SetFromTanRadians(double left, double right, double bottom, double top)
{
  VRFieldOfView fovInfo;
  fovInfo.leftDegrees = atan(left) * 180.0 / M_PI;
  fovInfo.rightDegrees = atan(right) * 180.0 / M_PI;
  fovInfo.upDegrees = atan(top) * 180.0 / M_PI;
  fovInfo.downDegrees = atan(bottom) * 180.0 / M_PI;
  return fovInfo;
}

HMDInfoOSVR::HMDInfoOSVR(OSVR_ClientContext *context, OSVR_ClientInterface *iface,
                         OSVR_DisplayConfig *display)
  : VRHMDInfo(VRHMDType::OSVR)
  , m_ctx(context), m_iface(iface), m_display(display)
{

  MOZ_COUNT_CTOR_INHERITED(HMDInfoOSVR, VRHMDInfo);

  mDeviceName.AssignLiteral("OSVR HMD");
  mSupportedSensorBits = State_Orientation | State_Position;

  // @todo OSVR display topology allows for more than one viewer
  // will assume only one viewer for now (most likely stay that way)

  OSVR_EyeCount numEyes;
  osvr_ClientGetNumEyesForViewer(*m_display, 0, &numEyes);

  for (uint8_t eye = 0; eye < numEyes; eye++){
    double left, right, bottom, top;
    // @todo for now there is only one surface per eye
    osvr_ClientGetViewerEyeSurfaceProjectionClippingPlanes(
      *m_display, 0, eye, 0, &left, &right, &bottom, &top);
    mRecommendedEyeFOV[eye] = mMaximumEyeFOV[eye] = SetFromTanRadians(left, right, bottom, top);
    
  }

  SetFOV(mRecommendedEyeFOV[Eye_Left], mRecommendedEyeFOV[Eye_Right], 0.01, 10000.0);

  uint32_t xcoord = 0;
  // @todo Assuming there is only one display input for now
  // however, it's possible to have more than one (dSight with 2 HDMI inputs)
  OSVR_DisplayDimension width, height;
  osvr_ClientGetDisplayDimensions(*m_display, 0, &width, &height);
  
  mScreen = VRHMDManager::MakeFakeScreen(xcoord, 0, std::max(width, height), 
                                          std::min(width, height));

}

void
HMDInfoOSVR::Destroy()
{

  // destroy non-owning pointers
  m_ctx = nullptr;
  m_iface = nullptr;
  m_display = nullptr;
  // @todo unload the libraries here
}

bool
HMDInfoOSVR::SetFOV(const VRFieldOfView& aFOVLeft, const VRFieldOfView& aFOVRight,
                    double zNear, double zFar)
{
  // @todo we'll focus on rendering later 

  return true;  
}

void
HMDInfoOSVR::FillDistortionConstants(uint32_t whichEye,
                                     const IntSize& textureSize,
                                     const IntRect& eyeViewport,
                                     const Size& destViewport,
                                     const Rect& destRect,
                                     VRDistortionConstants& values)
{
  // @todo take care of that later
}

bool
HMDInfoOSVR::StartSensorTracking()
{
  return true;
}

void
HMDInfoOSVR::StopSensorTracking()
{
}

void
HMDInfoOSVR::ZeroSensor()
{
  // @todo add reset yaw
}

VRHMDSensorState
HMDInfoOSVR::GetSensorState(double timeOffset)
{
  
  //update client context before anything
  //this usually goes into app's mainloop
  osvr_ClientUpdate(*m_ctx);

  VRHMDSensorState result;
  OSVR_TimeValue timestamp;
  result.Clear();

  OSVR_OrientationState orientation;
  
  OSVR_ReturnCode ret = osvr_GetOrientationState(*m_iface, &timestamp, &orientation);

  result.timestamp = timestamp.seconds;

  if (ret != OSVR_RETURN_SUCCESS) {
    printf_stderr("No orientation state\n");
  } else {
    result.flags |= State_Orientation;
    result.orientation[0] = orientation.data[1];
    result.orientation[1] = orientation.data[2];
    result.orientation[2] = orientation.data[3];
    result.orientation[3] = orientation.data[0];
  }

  OSVR_PositionState position;
  ret = osvr_GetPositionState(*m_iface, &timestamp, &position);
  if (ret != OSVR_RETURN_SUCCESS) {
    printf_stderr("No pose state\n");
  } else {
    result.flags |= State_Position;
    result.position[0] = position.data[0];
    result.position[1] = position.data[1];
    result.position[2] = position.data[2];
  }
  
  return result;
}

// @todo add Rendering later


already_AddRefed<VRHMDRenderingSupport::RenderTargetSet>
HMDInfoOSVR::CreateRenderTargetSet(layers::Compositor *aCompositor, const IntSize& aSize)
{
  // @todo add Rendering later

  return nullptr;
}

void
HMDInfoOSVR::DestroyRenderTargetSet(RenderTargetSet *aRTSet)
{
  // @todo add Rendering later
}

void
HMDInfoOSVR::SubmitFrame(RenderTargetSet *aRTSet)
{
  // @todo add Rendering later
}

bool
VRHMDManagerOSVR::PlatformInit()
{
  if (mOSVRPlatformInitialized) {
    return true;
  }
  if (!gfxPrefs::VREnabled() || 
      !gfxPrefs::VROSVREnabled()) {
    return false;
  }
  if (!LoadOSVRRuntime()) {
    return false;
  }
  mOSVRPlatformInitialized = true;
  return true;
}

bool
VRHMDManagerOSVR::Init()
{
  // OSVR server should be running in the background
  // It would load plugins and take care of detecting HMDs
  // maybe a @todo to add a check if it's running??
  if (mOSVRInitialized) {
    return true;
  }
  // get client context
  m_ctx = osvr_ClientInit("com.osvr.webvr", 0);

  m_iface = nullptr;
  OSVR_ReturnCode ret = osvr_ClientGetInterface(m_ctx, "/me/head", &m_iface);

  if (ret != OSVR_RETURN_SUCCESS) {
    // couldn't initialize interface
    return false;
  }

  ret = osvr_ClientGetDisplay(m_ctx, &m_display);

  if (ret != OSVR_RETURN_SUCCESS){
    // couldn't get display config
    return false;
  }
  
  //initialize HMD with all necessary sensors
  nsRefPtr<HMDInfoOSVR> hmd = new HMDInfoOSVR(&m_ctx, &m_iface, &m_display);
  mOSVRHMD = hmd;

  mOSVRInitialized = true;
  return true;
}

void
VRHMDManagerOSVR::Destroy()
{
  if (!mOSVRInitialized) { 
    return; 
  }

  if (mOSVRHMD) { 
    mOSVRHMD->Destroy(); 
  }

  mOSVRHMD = nullptr;
  //free interface
  osvr_ClientFreeInterface(m_ctx, m_iface);
  osvr_ClientFreeDisplay(m_display);
  osvr_ClientShutdown(m_ctx);    
  mOSVRInitialized = false;
}

void
VRHMDManagerOSVR::GetHMDs(nsTArray<nsRefPtr<VRHMDInfo>>& aHMDResult)
{
  Init();
  if (mOSVRHMD) {
    aHMDResult.AppendElement(mOSVRHMD);
  }
}