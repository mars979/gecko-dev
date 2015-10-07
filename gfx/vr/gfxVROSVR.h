/* -*- Mode: C++; tab-width: 20; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef GFX_VR_OSVR_H
#define GFX_VR_OSVR_H

#include "nsTArray.h"
#include "nsIScreen.h"
#include "nsCOMPtr.h"
#include "mozilla/nsRefPtr.h"

#include "mozilla/gfx/2D.h"
#include "mozilla/EnumeratedArray.h"

#include "gfxVR.h"

#include <osvr/ClientKit/ClientKitC.h>
#include <osvr/ClientKit/DisplayC.h>

namespace mozilla {
namespace gfx {
namespace impl {

class HMDInfoOSVR : public VRHMDInfo, public VRHMDRenderingSupport
{
public:
  explicit HMDInfoOSVR(OSVR_ClientContext* context, OSVR_ClientInterface* iface,
                       OSVR_DisplayConfig* display);

  bool SetFOV(const VRFieldOfView& aFOVLeft, const VRFieldOfView& aFOVRight,
              double zNear, double zFar) override;

  bool StartSensorTracking() override;
  VRHMDSensorState GetSensorState(double timeOffset) override;
  void StopSensorTracking() override;
  void ZeroSensor() override;

  void FillDistortionConstants(uint32_t whichEye, const IntSize& textureSize,
                               const IntRect& eyeViewport,
                               const Size& destViewport, const Rect& destRect,
                               VRDistortionConstants& values) override;

  VRHMDRenderingSupport* GetRenderingSupport() override { return this; }

  void Destroy();

  /* VRHMDRenderingSupport */
  already_AddRefed<RenderTargetSet> CreateRenderTargetSet(
    layers::Compositor* aCompositor, const IntSize& aSize) override;
  void DestroyRenderTargetSet(RenderTargetSet* aRTSet) override;
  void SubmitFrame(RenderTargetSet* aRTSet) override;

protected:
  // must match the size of VRDistortionVertex
  struct DistortionVertex
  {
    float pos[2];
    float texR[2];
    float texG[2];
    float texB[2];
    float genericAttribs[4];
  };

  virtual ~HMDInfoOSVR()
  {
    Destroy();
    MOZ_COUNT_DTOR_INHERITED(HMDInfoOSVR, VRHMDInfo);
  }

  uint32_t mStartCount;
  OSVR_ClientContext* m_ctx;
  OSVR_ClientInterface* m_iface;
  OSVR_DisplayConfig* m_display;
};

} // namespace impl

class VRHMDManagerOSVR : public VRHMDManager
{
public:
  VRHMDManagerOSVR()
    : mOSVRInitialized(false)
    , mOSVRPlatformInitialized(false)
  {
  }

  virtual bool PlatformInit() override;
  virtual bool Init() override;
  virtual void Destroy() override;
  virtual void GetHMDs(nsTArray<nsRefPtr<VRHMDInfo>>& aHMDResult) override;

protected:
  // only one HMD for now @todo we can have more than one HMD connected
  // nsTArray<nsRefPtr<impl::HMDInfoOSVR>> mOSVRHMDs;
  nsRefPtr<impl::HMDInfoOSVR> mOSVRHMD;
  bool mOSVRInitialized;
  bool mOSVRPlatformInitialized;

  OSVR_ClientContext m_ctx;
  OSVR_ClientInterface m_iface;
  OSVR_DisplayConfig m_display;
};

} // namespace gfx
} // namespace mozilla

#endif /* GFX_VR_OSVR_H */