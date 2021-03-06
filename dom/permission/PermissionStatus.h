/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* vim: set ts=8 sts=2 et sw=2 tw=80: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef mozilla_dom_PermissionStatus_h_
#define mozilla_dom_PermissionStatus_h_

#include "mozilla/dom/PermissionStatusBinding.h"
#include "mozilla/DOMEventTargetHelper.h"

namespace mozilla {
namespace dom {

class PermissionStatus final
  : public DOMEventTargetHelper
{
public:
  explicit PermissionStatus(nsPIDOMWindow* aWindow, PermissionState aState);

  JSObject* WrapObject(JSContext* aCx,
                       JS::Handle<JSObject*> aGivenProto) override;

  PermissionState State() const { return mState; }

  IMPL_EVENT_HANDLER(change)

private:
  ~PermissionStatus();

  PermissionState mState;
};

} // namespace dom
} // namespace mozilla

#endif // mozilla_dom_permissionstatus_h_
