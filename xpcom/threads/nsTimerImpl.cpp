/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* vim: set ts=8 sts=2 et sw=2 tw=80: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "nsTimerImpl.h"
#include "TimerThread.h"
#include "nsAutoPtr.h"
#include "nsThreadManager.h"
#include "nsThreadUtils.h"
#include "pratom.h"
#include "GeckoProfiler.h"
#include "mozilla/Atomics.h"
#include "mozilla/Logging.h"
#ifdef MOZ_NUWA_PROCESS
#include "ipc/Nuwa.h"
#endif
#ifdef MOZ_TASK_TRACER
#include "GeckoTaskTracerImpl.h"
using namespace mozilla::tasktracer;
#endif

using mozilla::Atomic;
using mozilla::LogLevel;
using mozilla::TimeDuration;
using mozilla::TimeStamp;

static Atomic<int32_t>  gGenerator;
static TimerThread*     gThread = nullptr;

PRLogModuleInfo*
GetTimerLog()
{
  static PRLogModuleInfo* sLog;
  if (!sLog) {
    sLog = PR_NewLogModule("nsTimerImpl");
  }
  return sLog;
}

#include <math.h>

double nsTimerImpl::sDeltaSumSquared = 0;
double nsTimerImpl::sDeltaSum = 0;
double nsTimerImpl::sDeltaNum = 0;

static void
myNS_MeanAndStdDev(double n, double sumOfValues, double sumOfSquaredValues,
                   double* meanResult, double* stdDevResult)
{
  double mean = 0.0, var = 0.0, stdDev = 0.0;
  if (n > 0.0 && sumOfValues >= 0) {
    mean = sumOfValues / n;
    double temp = (n * sumOfSquaredValues) - (sumOfValues * sumOfValues);
    if (temp < 0.0 || n <= 1) {
      var = 0.0;
    } else {
      var = temp / (n * (n - 1));
    }
    // for some reason, Windows says sqrt(0.0) is "-1.#J" (?!) so do this:
    stdDev = var != 0.0 ? sqrt(var) : 0.0;
  }
  *meanResult = mean;
  *stdDevResult = stdDev;
}

NS_IMPL_QUERY_INTERFACE(nsTimerImpl, nsITimer)
NS_IMPL_ADDREF(nsTimerImpl)

NS_IMETHODIMP_(MozExternalRefCountType)
nsTimerImpl::Release(void)
{
  nsrefcnt count;

  MOZ_ASSERT(int32_t(mRefCnt) > 0, "dup release");
  count = --mRefCnt;
  NS_LOG_RELEASE(this, count, "nsTimerImpl");
  if (count == 0) {
    mRefCnt = 1; /* stabilize */

    /* enable this to find non-threadsafe destructors: */
    /* NS_ASSERT_OWNINGTHREAD(nsTimerImpl); */
    delete this;
    return 0;
  }

  // If only one reference remains, and mArmed is set, then the ref must be
  // from the TimerThread::mTimers array, so we Cancel this timer to remove
  // the mTimers element, and return 0 if Cancel in fact disarmed the timer.
  //
  // We use an inlined version of nsTimerImpl::Cancel here to check for the
  // NS_ERROR_NOT_AVAILABLE code returned by gThread->RemoveTimer when this
  // timer is not found in the mTimers array -- i.e., when the timer was not
  // in fact armed once we acquired TimerThread::mLock, in spite of mArmed
  // being true here.  That can happen if the armed timer is being fired by
  // TimerThread::Run as we race and test mArmed just before it is cleared by
  // the timer thread.  If the RemoveTimer call below doesn't find this timer
  // in the mTimers array, then the last ref to this timer is held manually
  // and temporarily by the TimerThread, so we should fall through to the
  // final return and return 1, not 0.
  //
  // The original version of this thread-based timer code kept weak refs from
  // TimerThread::mTimers, removing this timer's weak ref in the destructor,
  // but that leads to double-destructions in the race described above, and
  // adding mArmed doesn't help, because destructors can't be deferred, once
  // begun.  But by combining reference-counting and a specialized Release
  // method with "is this timer still in the mTimers array once we acquire
  // the TimerThread's lock" testing, we defer destruction until we're sure
  // that only one thread has its hot little hands on this timer.
  //
  // Note that both approaches preclude a timer creator, and everyone else
  // except the TimerThread who might have a strong ref, from dropping all
  // their strong refs without implicitly canceling the timer.  Timers need
  // non-mTimers-element strong refs to stay alive.

  if (count == 1 && mArmed) {
    mCanceled = true;

    MOZ_ASSERT(gThread, "Armed timer exists after the thread timer stopped.");
    if (NS_SUCCEEDED(gThread->RemoveTimer(this))) {
      return 0;
    }
  }

  return count;
}

nsTimerImpl::nsTimerImpl() :
  mClosure(nullptr),
  mCallbackType(CallbackType::Unknown),
  mFiring(false),
  mArmed(false),
  mCanceled(false),
  mGeneration(0),
  mDelay(0)
{
  // XXXbsmedberg: shouldn't this be in Init()?
  mEventTarget = static_cast<nsIEventTarget*>(NS_GetCurrentThread());

  mCallback.c = nullptr;
}

nsTimerImpl::~nsTimerImpl()
{
  ReleaseCallback();
}

//static
nsresult
nsTimerImpl::Startup()
{
  nsresult rv;

  gThread = new TimerThread();

  NS_ADDREF(gThread);
  rv = gThread->InitLocks();

  if (NS_FAILED(rv)) {
    NS_RELEASE(gThread);
  }

  return rv;
}

void
nsTimerImpl::Shutdown()
{
  if (MOZ_LOG_TEST(GetTimerLog(), LogLevel::Debug)) {
    double mean = 0, stddev = 0;
    myNS_MeanAndStdDev(sDeltaNum, sDeltaSum, sDeltaSumSquared, &mean, &stddev);

    MOZ_LOG(GetTimerLog(), LogLevel::Debug,
           ("sDeltaNum = %f, sDeltaSum = %f, sDeltaSumSquared = %f\n",
            sDeltaNum, sDeltaSum, sDeltaSumSquared));
    MOZ_LOG(GetTimerLog(), LogLevel::Debug,
           ("mean: %fms, stddev: %fms\n", mean, stddev));
  }

  if (!gThread) {
    return;
  }

  gThread->Shutdown();
  NS_RELEASE(gThread);
}


nsresult
nsTimerImpl::InitCommon(uint32_t aType, uint32_t aDelay)
{
  nsresult rv;

  if (NS_WARN_IF(!gThread)) {
    return NS_ERROR_NOT_INITIALIZED;
  }
  if (!mEventTarget) {
    NS_ERROR("mEventTarget is NULL");
    return NS_ERROR_NOT_INITIALIZED;
  }

  rv = gThread->Init();
  if (NS_WARN_IF(NS_FAILED(rv))) {
    return rv;
  }

  gThread->RemoveTimer(this);
  mCanceled = false;
  mTimeout = TimeStamp();
  mGeneration = gGenerator++;

  mType = (uint8_t)aType;
  SetDelayInternal(aDelay);

  return gThread->AddTimer(this);
}

NS_IMETHODIMP
nsTimerImpl::InitWithFuncCallback(nsTimerCallbackFunc aFunc,
                                  void* aClosure,
                                  uint32_t aDelay,
                                  uint32_t aType)
{
  if (NS_WARN_IF(!aFunc)) {
    return NS_ERROR_INVALID_ARG;
  }

  ReleaseCallback();
  mCallbackType = CallbackType::Function;
  mCallback.c = aFunc;
  mClosure = aClosure;

  return InitCommon(aType, aDelay);
}

NS_IMETHODIMP
nsTimerImpl::InitWithCallback(nsITimerCallback* aCallback,
                              uint32_t aDelay,
                              uint32_t aType)
{
  if (NS_WARN_IF(!aCallback)) {
    return NS_ERROR_INVALID_ARG;
  }

  ReleaseCallback();
  mCallbackType = CallbackType::Interface;
  mCallback.i = aCallback;
  NS_ADDREF(mCallback.i);

  return InitCommon(aType, aDelay);
}

NS_IMETHODIMP
nsTimerImpl::Init(nsIObserver* aObserver, uint32_t aDelay, uint32_t aType)
{
  if (NS_WARN_IF(!aObserver)) {
    return NS_ERROR_INVALID_ARG;
  }

  ReleaseCallback();
  mCallbackType = CallbackType::Observer;
  mCallback.o = aObserver;
  NS_ADDREF(mCallback.o);

  return InitCommon(aType, aDelay);
}

NS_IMETHODIMP
nsTimerImpl::Cancel()
{
  mCanceled = true;

  if (gThread) {
    gThread->RemoveTimer(this);
  }

  ReleaseCallback();

  return NS_OK;
}

NS_IMETHODIMP
nsTimerImpl::SetDelay(uint32_t aDelay)
{
  if (mCallbackType == CallbackType::Unknown && mType == TYPE_ONE_SHOT) {
    // This may happen if someone tries to re-use a one-shot timer
    // by re-setting delay instead of reinitializing the timer.
    NS_ERROR("nsITimer->SetDelay() called when the "
             "one-shot timer is not set up.");
    return NS_ERROR_NOT_INITIALIZED;
  }

  // If we're already repeating precisely, update mTimeout now so that the
  // new delay takes effect in the future.
  if (!mTimeout.IsNull() && mType == TYPE_REPEATING_PRECISE) {
    mTimeout = TimeStamp::Now();
  }

  SetDelayInternal(aDelay);

  if (!mFiring && gThread) {
    gThread->TimerDelayChanged(this);
  }

  return NS_OK;
}

NS_IMETHODIMP
nsTimerImpl::GetDelay(uint32_t* aDelay)
{
  *aDelay = mDelay;
  return NS_OK;
}

NS_IMETHODIMP
nsTimerImpl::SetType(uint32_t aType)
{
  mType = (uint8_t)aType;
  // XXX if this is called, we should change the actual type.. this could effect
  // repeating timers.  we need to ensure in Fire() that if mType has changed
  // during the callback that we don't end up with the timer in the queue twice.
  return NS_OK;
}

NS_IMETHODIMP
nsTimerImpl::GetType(uint32_t* aType)
{
  *aType = mType;
  return NS_OK;
}


NS_IMETHODIMP
nsTimerImpl::GetClosure(void** aClosure)
{
  *aClosure = mClosure;
  return NS_OK;
}


NS_IMETHODIMP
nsTimerImpl::GetCallback(nsITimerCallback** aCallback)
{
  if (mCallbackType == CallbackType::Interface) {
    NS_IF_ADDREF(*aCallback = mCallback.i);
  } else if (mTimerCallbackWhileFiring) {
    NS_ADDREF(*aCallback = mTimerCallbackWhileFiring);
  } else {
    *aCallback = nullptr;
  }

  return NS_OK;
}


NS_IMETHODIMP
nsTimerImpl::GetTarget(nsIEventTarget** aTarget)
{
  NS_IF_ADDREF(*aTarget = mEventTarget);
  return NS_OK;
}


NS_IMETHODIMP
nsTimerImpl::SetTarget(nsIEventTarget* aTarget)
{
  if (NS_WARN_IF(mCallbackType != CallbackType::Unknown)) {
    return NS_ERROR_ALREADY_INITIALIZED;
  }

  if (aTarget) {
    mEventTarget = aTarget;
  } else {
    mEventTarget = static_cast<nsIEventTarget*>(NS_GetCurrentThread());
  }
  return NS_OK;
}


void
nsTimerImpl::Fire()
{
  if (mCanceled) {
    return;
  }

#if !defined(MOZILLA_XPCOMRT_API)
  PROFILER_LABEL("Timer", "Fire",
                 js::ProfileEntry::Category::OTHER);
#endif

  TimeStamp now = TimeStamp::Now();
  if (MOZ_LOG_TEST(GetTimerLog(), LogLevel::Debug)) {
    TimeDuration   a = now - mStart; // actual delay in intervals
    TimeDuration   b = TimeDuration::FromMilliseconds(mDelay); // expected delay in intervals
    TimeDuration   delta = (a > b) ? a - b : b - a;
    uint32_t       d = delta.ToMilliseconds(); // delta in ms
    sDeltaSum += d;
    sDeltaSumSquared += double(d) * double(d);
    sDeltaNum++;

    MOZ_LOG(GetTimerLog(), LogLevel::Debug,
           ("[this=%p] expected delay time %4ums\n", this, mDelay));
    MOZ_LOG(GetTimerLog(), LogLevel::Debug,
           ("[this=%p] actual delay time   %fms\n", this,
            a.ToMilliseconds()));
    MOZ_LOG(GetTimerLog(), LogLevel::Debug,
           ("[this=%p] (mType is %d)       -------\n", this, mType));
    MOZ_LOG(GetTimerLog(), LogLevel::Debug,
           ("[this=%p]     delta           %4dms\n",
            this, (a > b) ? (int32_t)d : -(int32_t)d));

    mStart = mStart2;
    mStart2 = TimeStamp();
  }

  TimeStamp timeout = mTimeout;
  if (IsRepeatingPrecisely()) {
    // Precise repeating timers advance mTimeout by mDelay without fail before
    // calling Fire().
    timeout -= TimeDuration::FromMilliseconds(mDelay);
  }

  if (mCallbackType == CallbackType::Interface) {
    mTimerCallbackWhileFiring = mCallback.i;
  }
  mFiring = true;

  // Handle callbacks that re-init the timer, but avoid leaking.
  // See bug 330128.
  CallbackUnion callback = mCallback;
  CallbackType callbackType = mCallbackType;
  if (callbackType == CallbackType::Interface) {
    NS_ADDREF(callback.i);
  } else if (callbackType == CallbackType::Observer) {
    NS_ADDREF(callback.o);
  }
  ReleaseCallback();

  switch (callbackType) {
    case CallbackType::Function:
      callback.c(this, mClosure);
      break;
    case CallbackType::Interface:
      callback.i->Notify(this);
      break;
    case CallbackType::Observer:
      callback.o->Observe(static_cast<nsITimer*>(this),
                          NS_TIMER_CALLBACK_TOPIC,
                          nullptr);
      break;
    default:
      ;
  }

  // If the callback didn't re-init the timer, and it's not a one-shot timer,
  // restore the callback state.
  if (mCallbackType == CallbackType::Unknown &&
      mType != TYPE_ONE_SHOT && !mCanceled) {
    mCallback = callback;
    mCallbackType = callbackType;
  } else {
    // The timer was a one-shot, or the callback was reinitialized.
    if (callbackType == CallbackType::Interface) {
      NS_RELEASE(callback.i);
    } else if (callbackType == CallbackType::Observer) {
      NS_RELEASE(callback.o);
    }
  }

  mFiring = false;
  mTimerCallbackWhileFiring = nullptr;

  MOZ_LOG(GetTimerLog(), LogLevel::Debug,
         ("[this=%p] Took %fms to fire timer callback\n",
          this, (TimeStamp::Now() - now).ToMilliseconds()));

  // Reschedule repeating timers, except REPEATING_PRECISE which already did
  // that in PostTimerEvent, but make sure that we aren't armed already (which
  // can happen if the callback reinitialized the timer).
  if (IsRepeating() && mType != TYPE_REPEATING_PRECISE && !mArmed) {
    if (mType == TYPE_REPEATING_SLACK) {
      SetDelayInternal(mDelay);  // force mTimeout to be recomputed.  For
    }
    // REPEATING_PRECISE_CAN_SKIP timers this has
    // already happened.
    if (gThread) {
      gThread->AddTimer(this);
    }
  }
}

void
nsTimerImpl::SetDelayInternal(uint32_t aDelay)
{
  TimeDuration delayInterval = TimeDuration::FromMilliseconds(aDelay);

  mDelay = aDelay;

  TimeStamp now = TimeStamp::Now();
  if (mTimeout.IsNull() || mType != TYPE_REPEATING_PRECISE) {
    mTimeout = now;
  }

  mTimeout += delayInterval;

  if (MOZ_LOG_TEST(GetTimerLog(), LogLevel::Debug)) {
    if (mStart.IsNull()) {
      mStart = now;
    } else {
      mStart2 = now;
    }
  }
}

size_t
nsTimerImpl::SizeOfIncludingThis(mozilla::MallocSizeOf aMallocSizeOf) const
{
  return aMallocSizeOf(this);
}

#ifdef MOZ_TASK_TRACER
void
nsTimerImpl::GetTLSTraceInfo()
{
  mTracedTask.GetTLSTraceInfo();
}

TracedTaskCommon
nsTimerImpl::GetTracedTask()
{
  return mTracedTask;
}
#endif

