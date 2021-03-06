/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "Common.h"

#include <cstdlib>
#include "gtest/gtest.h"

#include "nsDirectoryServiceDefs.h"
#include "nsIDirectoryService.h"
#include "nsIFile.h"
#include "nsIInputStream.h"
#include "nsIProperties.h"
#include "nsNetUtil.h"
#include "mozilla/nsRefPtr.h"
#include "nsString.h"

namespace mozilla {

using namespace gfx;

using std::abs;

///////////////////////////////////////////////////////////////////////////////
// Helpers
///////////////////////////////////////////////////////////////////////////////

// These macros work like gtest's ASSERT_* macros, except that they can be used
// in functions that return values.
#define ASSERT_TRUE_OR_RETURN(e, rv) \
  EXPECT_TRUE(e);                    \
  if (!(e)) {                        \
    return rv;                       \
  }

#define ASSERT_EQ_OR_RETURN(a, b, rv) \
  EXPECT_EQ(a, b);                    \
  if ((a) != (b)) {                   \
    return rv;                        \
  }

#define ASSERT_LE_OR_RETURN(a, b, rv) \
  EXPECT_LE(a, b);                    \
  if (!((a) <= (b))) {                 \
    return rv;                        \
  }

already_AddRefed<nsIInputStream>
LoadFile(const char* aRelativePath)
{
  nsresult rv;

  nsCOMPtr<nsIProperties> dirService =
    do_GetService(NS_DIRECTORY_SERVICE_CONTRACTID);
  ASSERT_TRUE_OR_RETURN(dirService != nullptr, nullptr);

  // Retrieve the current working directory.
  nsCOMPtr<nsIFile> file;
  rv = dirService->Get(NS_OS_CURRENT_WORKING_DIR,
                       NS_GET_IID(nsIFile), getter_AddRefs(file));
  ASSERT_TRUE_OR_RETURN(NS_SUCCEEDED(rv), nullptr);

  // Construct the final path by appending the working path to the current
  // working directory.
  file->AppendNative(nsAutoCString(aRelativePath));

  // Construct an input stream for the requested file.
  nsCOMPtr<nsIInputStream> inputStream;
  rv = NS_NewLocalFileInputStream(getter_AddRefs(inputStream), file);
  ASSERT_TRUE_OR_RETURN(NS_SUCCEEDED(rv), nullptr);

  return inputStream.forget();
}

bool
IsSolidColor(SourceSurface* aSurface, BGRAColor aColor, bool aFuzzy)
{
  nsRefPtr<DataSourceSurface> dataSurface = aSurface->GetDataSurface();
  ASSERT_TRUE_OR_RETURN(dataSurface != nullptr, false);

  ASSERT_EQ_OR_RETURN(dataSurface->Stride(), aSurface->GetSize().width * 4,
                      false);

  DataSourceSurface::ScopedMap mapping(dataSurface,
                                       DataSourceSurface::MapType::READ);
  ASSERT_TRUE_OR_RETURN(mapping.IsMapped(), false);

  uint8_t* data = dataSurface->GetData();
  ASSERT_TRUE_OR_RETURN(data != nullptr, false);

  int32_t length = dataSurface->Stride() * aSurface->GetSize().height;
  for (int32_t i = 0 ; i < length ; i += 4) {
    if (aFuzzy) {
      ASSERT_LE_OR_RETURN(abs(aColor.mBlue - data[i + 0]), 1, false);
      ASSERT_LE_OR_RETURN(abs(aColor.mGreen - data[i + 1]), 1, false);
      ASSERT_LE_OR_RETURN(abs(aColor.mRed - data[i + 2]), 1, false);
      ASSERT_LE_OR_RETURN(abs(aColor.mAlpha - data[i + 3]), 1, false);
    } else {
      ASSERT_EQ_OR_RETURN(aColor.mBlue,  data[i + 0], false);
      ASSERT_EQ_OR_RETURN(aColor.mGreen, data[i + 1], false);
      ASSERT_EQ_OR_RETURN(aColor.mRed,   data[i + 2], false);
      ASSERT_EQ_OR_RETURN(aColor.mAlpha, data[i + 3], false);
    }
  }

  return true;
}


///////////////////////////////////////////////////////////////////////////////
// Test Data
///////////////////////////////////////////////////////////////////////////////

ImageTestCase GreenPNGTestCase()
{
  return ImageTestCase("green.png", "image/png", IntSize(100, 100));
}

ImageTestCase GreenGIFTestCase()
{
  return ImageTestCase("green.gif", "image/gif", IntSize(100, 100));
}

ImageTestCase GreenJPGTestCase()
{
  return ImageTestCase("green.jpg", "image/jpeg", IntSize(100, 100),
                       /* aFuzzy = */ true);
}

ImageTestCase GreenBMPTestCase()
{
  return ImageTestCase("green.bmp", "image/bmp", IntSize(100, 100));
}

ImageTestCase GreenICOTestCase()
{
  return ImageTestCase("green.ico", "image/x-icon", IntSize(100, 100));
}

ImageTestCase GreenFirstFrameAnimatedGIFTestCase()
{
  return ImageTestCase("first-frame-green.gif", "image/gif", IntSize(100, 100));
}

ImageTestCase GreenFirstFrameAnimatedPNGTestCase()
{
  return ImageTestCase("first-frame-green.png", "image/png", IntSize(100, 100));
}

ImageTestCase CorruptTestCase()
{
  return ImageTestCase("corrupt.jpg", "image/jpeg", IntSize(100, 100));
}

} // namespace mozilla
