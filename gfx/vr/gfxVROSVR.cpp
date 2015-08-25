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


#if 0
#ifdef HAVE_64BIT_BUILD
#pragma comment(lib, "c:/osvr/win64/osvrClient.lib")
else
#pragma comment(lib, "c:/osvr/win32/osvrClient.lib")
#endif
#endif
#endif


#include "gfxVROSVR.h"

#include "nsServiceManagerUtils.h"
#include "nsIScreenManager.h"

#ifndef M_PI
# define M_PI 3.14159265358979323846
#endif

using namespace mozilla::gfx;
using namespace mozilla::gfx::impl;

namespace {

// need to typedef functions that will be used in the code below
extern "C"{
typedef OSVR_ClientContext(*pfn_osvrClientInit) (const char applicationIdentifier[],
    uint32_t flags);
typedef OSVR_ReturnCode(*pfn_osvrClientShutdown)(OSVR_ClientContext ctx);
typedef OSVR_ReturnCode(*pfn_osvrClientUpdate)(OSVR_ClientContext ctx);
typedef OSVR_ReturnCode(*pfn_osvrClientGetInterface)(OSVR_ClientContext ctx, const char path[],
    OSVR_ClientInterface *iface);
typedef OSVR_ReturnCode(*pfn_osvrClientFreeInterface)(OSVR_ClientContext ctx,
    OSVR_ClientInterface iface);
typedef OSVR_ReturnCode(*pfn_osvrGetOrientationState)(OSVR_ClientInterface iface,
                        OSVR_TimeValue *timestamp, OSVR_OrientationState *state);
typedef OSVR_ReturnCode(*pfn_osvrGetPositionState)(OSVR_ClientInterface iface,
                        OSVR_TimeValue *timestamp, OSVR_PositionState *state);

}

static pfn_osvrClientInit osvr_ClientInit = nullptr;
static pfn_osvrClientShutdown osvr_ClientShutdown = nullptr;
static pfn_osvrClientUpdate osvr_ClientUpdate = nullptr;
static pfn_osvrClientGetInterface osvr_ClientGetInterface = nullptr;
static pfn_osvrClientFreeInterface osvr_ClientFreeInterface = nullptr;
static pfn_osvrGetOrientationState osvr_GetOrientationState = nullptr;
static pfn_osvrGetPositionState osvr_GetPositionState = nullptr;


bool LoadOSVRRuntime()
{
    static PRLibrary *osvrUtilLib = nullptr;
    static PRLibrary *osvrCommonLib = nullptr;
    static PRLibrary *osvrClientLib = nullptr;
    static PRLibrary *osvrClientKitLib = nullptr;

  nsAdoptingCString osvrUtilPath = mozilla::Preferences::GetCString("gfx.vr.osvrUtil.lib");
  nsAdoptingCString osvrCommonPath = mozilla::Preferences::GetCString("gfx.vr.osvrCommon.lib");
  nsAdoptingCString osvrClientPath = mozilla::Preferences::GetCString("gfx.vr.osvrClient.lib");
  nsAdoptingCString osvrClientKitPath = mozilla::Preferences::GetCString("gfx.vr.osvrClientKit.lib");
  
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

#undef REQUIRE_FUNCTION

  return true;

 fail:
  return false;
}

} // namespace

HMDInfoOSVR::HMDInfoOSVR(OSVR_ClientContext *context)
  : VRHMDInfo(VRHMDType::OSVR)
  , m_ctx(context)
{

  MOZ_COUNT_CTOR_INHERITED(HMDInfoOSVR, VRHMDInfo);

  mDeviceName.AssignLiteral("OSVR HMD");

  mSupportedSensorBits = State_Orientation | State_Position;
  
  OSVR_ReturnCode ret = osvr_ClientGetInterface(*m_ctx, "/me/head", &m_iface);

  if (ret != OSVR_RETURN_SUCCESS){
      printf("Couldn't initialize interface\n");
  }

  /*
  mVRCompositor->SetTrackingSpace(vr::TrackingUniverseSeated);

  // SteamVR gives the application a single FOV to use; it's not configurable as with Oculus
  for (uint32_t eye = 0; eye < 2; ++eye) {
      // get l/r/t/b clip plane coordinates
      float l, r, t, b;
      mVRSystem->GetProjectionRaw(static_cast<vr::Hmd_Eye>(eye), &l, &r, &t, &b);
      mEyeFOV[eye].SetFromTanRadians(-t, r, b, -l);
      mRecommendedEyeFOV[eye] = mMaximumEyeFOV[eye] = mEyeFOV[eye];
  }

  SetFOV(mEyeFOV[Eye_Left], mEyeFOV[Eye_Right], 0.01, 10000.0);

  uint32_t xcoord = 0;
  uint32_t w = 2160, h = 1200;
  mScreen = VRHMDManager::MakeFakeScreen(xcoord, 0, std::max(w, h), std::min(w, h));
  */

}

void
HMDInfoOSVR::Destroy()
{
    // destroy non-owning pointer
    m_ctx = nullptr;
}

bool
HMDInfoOSVR::SetFOV(const VRFieldOfView& aFOVLeft, const VRFieldOfView& aFOVRight,
                      double zNear, double zFar)
{

    //we'll focus on rendering later @todo

    /*
  float pixelsPerDisplayPixel = 1.0;
  ovrSizei texSize[2];

  // get eye parameters and create the mesh
  for (uint32_t eye = 0; eye < NumEyes; eye++) {
    mEyeFOV[eye] = eye == 0 ? aFOVLeft : aFOVRight;
    mFOVPort[eye] = ToFovPort(mEyeFOV[eye]);

    ovrEyeRenderDesc renderDesc = ovrHmd_GetRenderDesc(mHMD, (ovrEyeType) eye, mFOVPort[eye]);

    // As of OSVR 0.6.0, the HmdToEyeViewOffset values are correct and don't need to be negated.
    mEyeTranslation[eye] = Point3D(renderDesc.HmdToEyeViewOffset.x, renderDesc.HmdToEyeViewOffset.y, renderDesc.HmdToEyeViewOffset.z);

    // note that we are using a right-handed coordinate system here, to match CSS
    mEyeProjectionMatrix[eye] = mEyeFOV[eye].ConstructProjectionMatrix(zNear, zFar, true);

    texSize[eye] = ovrHmd_GetFovTextureSize(mHMD, (ovrEyeType) eye, mFOVPort[eye], pixelsPerDisplayPixel);
  }

  // take the max of both for eye resolution
  mEyeResolution.width = std::max(texSize[Eye_Left].w, texSize[Eye_Right].w);
  mEyeResolution.height = std::max(texSize[Eye_Left].h, texSize[Eye_Right].h);

  mConfiguration.hmdType = mType;
  mConfiguration.value = 0;
  mConfiguration.fov[0] = aFOVLeft;
  mConfiguration.fov[1] = aFOVRight;
  */
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
    // @todo take care of that

}

bool
HMDInfoOSVR::StartSensorTracking()
{
    /*
  if (mStartCount == 0) {
      //try to get an interface
      OSVR_ReturnCode ret = osvr_ClientGetInterface(*m_ctx, "/me/head", &m_iface);
      
      if (ret != OSVR_RETURN_SUCCESS){
          return false;
      }
  }

  mStartCount++;
  */
  return true;
}

void
HMDInfoOSVR::StopSensorTracking()
{
  if (--mStartCount == 0) {
      osvr_ClientFreeInterface(*m_ctx, m_iface);
  }
}

void
HMDInfoOSVR::ZeroSensor()
{
  // @todo 
}

VRHMDSensorState
HMDInfoOSVR::GetSensorState(double timeOffset)
{
    //update client context before anything
    osvr_ClientUpdate(*m_ctx);

  VRHMDSensorState result;
  OSVR_TimeValue timestamp;
  result.Clear();

  // XXX this is the wrong time base for timeOffset; we need to figure out how to synchronize
  // the OSVR time base and the browser one.
  OSVR_OrientationState orientation;
  
  OSVR_ReturnCode ret = osvr_GetOrientationState(m_iface, &timestamp, &orientation);

  result.timestamp = timestamp.seconds;
  int ctr = 0;
  while ((ret == OSVR_RETURN_FAILURE) && (ctr < 10000)){
      printf("Trying to get orientation\n");
      osvr_ClientUpdate(*m_ctx);
      ret = osvr_GetOrientationState(m_iface, &timestamp, &orientation);
      ctr++;
      
  }

  if (ret != OSVR_RETURN_SUCCESS) {
      printf_stderr("No orientation state\n");
  }
  else{
    result.flags |= State_Orientation;

    result.orientation[0] = orientation.data[1];
    result.orientation[1] = orientation.data[2];
    result.orientation[2] = orientation.data[3];
    result.orientation[3] = orientation.data[0];
  }

  OSVR_PositionState position;
  ret = osvr_GetPositionState(m_iface, &timestamp, &position);
  if (ret != OSVR_RETURN_SUCCESS){
      printf_stderr("No pose state\n");
  }
  else{
    result.flags |= State_Position;

    result.position[0] = position.data[0];
    result.position[1] = position.data[1];
    result.position[2] = position.data[2];
  }
  
  return result;
}
/*
struct RenderTargetSetOSVR : public VRHMDRenderingSupport::RenderTargetSet
{
  RenderTargetSetOSVR(const IntSize& aSize,
                        HMDInfoOSVR *aHMD,
                        ovrSwapTextureSet *aTS)
    : hmd(aHMD)
  {
    textureSet = aTS;
    size = aSize;
  }
  
  already_AddRefed<layers::CompositingRenderTarget> GetNextRenderTarget() override {
    currentRenderTarget = (currentRenderTarget + 1) % renderTargets.Length();
    textureSet->CurrentIndex = currentRenderTarget;
    renderTargets[currentRenderTarget]->ClearOnBind();
    nsRefPtr<layers::CompositingRenderTarget> rt = renderTargets[currentRenderTarget];
    return rt.forget();
  }

  void Destroy() {
    if (!hmd)
      return;
    
    if (hmd->GetOSVRHMD()) {
      // If the ovrHmd was already destroyed, so were all associated
      // texture sets
      ovrHmd_DestroySwapTextureSet(hmd->GetOSVRHMD(), textureSet);
    }
    hmd = nullptr;
    textureSet = nullptr;
  }
  
  ~RenderTargetSetOSVR() {
    Destroy();
  }

  nsRefPtr<HMDInfoOSVR> hmd;
  ovrSwapTextureSet *textureSet;
};

#ifdef XP_WIN
class BasicTextureSourceD3D11 : public layers::TextureSourceD3D11
{
public:
  BasicTextureSourceD3D11(ID3D11Texture2D *aTexture, const IntSize& aSize) {
    mTexture = aTexture;
    mSize = aSize;
  }
};

struct RenderTargetSetD3D11 : public RenderTargetSetOSVR
{
  RenderTargetSetD3D11(layers::CompositorD3D11 *aCompositor,
                       const IntSize& aSize,
                       HMDInfoOSVR *aHMD,
                       ovrSwapTextureSet *aTS)
    : RenderTargetSetOSVR(aSize, aHMD, aTS)
  {
    compositor = aCompositor;
    
    renderTargets.SetLength(aTS->TextureCount);
    
    currentRenderTarget = aTS->CurrentIndex;

    for (int i = 0; i < aTS->TextureCount; ++i) {
      ovrD3D11Texture *tex11;
      nsRefPtr<layers::CompositingRenderTargetD3D11> rt;
      
      tex11 = (ovrD3D11Texture*)&aTS->Textures[i];
      rt = new layers::CompositingRenderTargetD3D11(tex11->D3D11.pTexture, IntPoint(0, 0));
      rt->SetSize(size);
      renderTargets[i] = rt;
    }
  }
};
#endif
*/

already_AddRefed<VRHMDRenderingSupport::RenderTargetSet>
HMDInfoOSVR::CreateRenderTargetSet(layers::Compositor *aCompositor, const IntSize& aSize)
{

    // skip for now
    /*
#ifdef XP_WIN
  if (aCompositor->GetBackendType() == layers::LayersBackend::LAYERS_D3D11)
  {
    layers::CompositorD3D11 *comp11 = static_cast<layers::CompositorD3D11*>(aCompositor);

    CD3D11_TEXTURE2D_DESC desc(DXGI_FORMAT_B8G8R8A8_UNORM, aSize.width, aSize.height, 1, 1,
                               D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_RENDER_TARGET);
    ovrSwapTextureSet *ts = nullptr;
    
    ovrResult orv = ovrHmd_CreateSwapTextureSetD3D11(mHMD, comp11->GetDevice(), &desc, &ts);
    if (orv != ovrSuccess) {
      return nullptr;
    }

    nsRefPtr<RenderTargetSetD3D11> rts = new RenderTargetSetD3D11(comp11, aSize, this, ts);
    return rts.forget();
  }
#endif

  if (aCompositor->GetBackendType() == layers::LayersBackend::LAYERS_OPENGL) {
  }
  */

  return nullptr;
}

void
HMDInfoOSVR::DestroyRenderTargetSet(RenderTargetSet *aRTSet)
{
    // skip for now
    /*
  RenderTargetSetOSVR *rts = static_cast<RenderTargetSetOSVR*>(aRTSet);
  rts->Destroy();
  */
}

void
HMDInfoOSVR::SubmitFrame(RenderTargetSet *aRTSet)
{
    // we'll skip for now

    /*
  RenderTargetSetOSVR *rts = static_cast<RenderTargetSetOSVR*>(aRTSet);
  MOZ_ASSERT(rts->hmd != nullptr);
  MOZ_ASSERT(rts->textureSet != nullptr);

  ovrLayerEyeFov layer;
  layer.Header.Type = ovrLayerType_EyeFov;
  layer.Header.Flags = 0;
  layer.ColorTexture[0] = rts->textureSet;
  layer.ColorTexture[1] = nullptr;
  layer.Fov[0] = mFOVPort[0];
  layer.Fov[1] = mFOVPort[1];
  layer.Viewport[0].Pos.x = 0;
  layer.Viewport[0].Pos.y = 0;
  layer.Viewport[0].Size.w = rts->size.width / 2;
  layer.Viewport[0].Size.h = rts->size.height;
  layer.Viewport[1].Pos.x = rts->size.width / 2;
  layer.Viewport[1].Pos.y = 0;
  layer.Viewport[1].Size.w = rts->size.width / 2;
  layer.Viewport[1].Size.h = rts->size.height;

  const Point3D& l = rts->hmd->mEyeTranslation[0];
  const Point3D& r = rts->hmd->mEyeTranslation[1];
  const ovrVector3f hmdToEyeViewOffset[2] = { { l.x, l.y, l.z },
                                              { r.x, r.y, r.z } };
  do_CalcEyePoses(rts->hmd->mLastTrackingState.HeadPose.ThePose, hmdToEyeViewOffset, layer.RenderPose);

  ovrLayerHeader *layers = &layer.Header;
  ovrResult orv = ovrHmd_SubmitFrame(mHMD, 0, nullptr, &layers, 1);
  //printf_stderr("Submitted frame %d, result: %d\n", rts->textureSet->CurrentIndex, orv);
  if (orv != ovrSuccess) {
    // not visible? failed?
  }
  */
}

bool
VRHMDManagerOSVR::PlatformInit()
{
  printf("VRHMDManagerOSVR::PlatformInit : HERE\n");
  if (mOSVRPlatformInitialized){
      printf("VRHMDManagerOSVR::PlatformInit : Returning true, alread initialized\n");
      return true;
  }
  if (!gfxPrefs::VREnabled())
  {
    printf("VRHMDManagerOSVR::PlatformInit : Returning false, VR disabled\n");
    return false;
  }
  if (!LoadOSVRRuntime()){
      printf("VRHMDManagerOSVR::PlatformInit : Couldn't load OSVR RunTime\n");
      return false;
  }
  printf("VRHMDManagerOSVR::PlatformInit : Returning true, all initialized\n");
  mOSVRPlatformInitialized = true;
  return true;
}

bool
VRHMDManagerOSVR::Init()
{
    printf("VRHMDManagerOSVR::Init : HERE\n");
    // OSVR server should be running in the background
    // It would load plugins and take care of detecting HMDs
    // maybe a @todo to add a check if it's running??
  if (mOSVRInitialized)
    return true;

  // get client context
  m_ctx = osvr_ClientInit("com.osvr.webvr", 0);
  //try to get an interface
  //OSVR_ReturnCode ret = osvr_ClientGetInterface(m_ctx, "/me/head", &m_iface);

  nsRefPtr<HMDInfoOSVR> hmd = new HMDInfoOSVR(&m_ctx);
  mOSVRHMD = hmd;

  mOSVRInitialized = true;
  return true;
}

void
VRHMDManagerOSVR::Destroy()
{
    printf("VRHMDManagerOSVR::Destroy : HERE\n");
    if (!mOSVRInitialized){ return; }

    if (mOSVRHMD) { mOSVRHMD->Destroy(); }

    mOSVRHMD = nullptr;

    osvr_ClientShutdown(m_ctx);
    
    mOSVRInitialized = false;
}

void
VRHMDManagerOSVR::GetHMDs(nsTArray<nsRefPtr<VRHMDInfo>>& aHMDResult)
{
    printf("VRHMDManagerOSVR::GetHMDs : HERE\n");
    Init();
    if (mOSVRHMD){
        printf("VRHMDManagerOSVR::GetHMDs : Added HMD\n");
        aHMDResult.AppendElement(mOSVRHMD);
    }
}
