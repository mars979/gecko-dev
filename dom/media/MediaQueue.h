/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* vim:set ts=2 sw=2 sts=2 et cindent: */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#if !defined(MediaQueue_h_)
#define MediaQueue_h_

#include "mozilla/ReentrantMonitor.h"
#include "mozilla/TaskQueue.h"
#include "mozilla/UniquePtr.h"

#include "nsDeque.h"
#include "nsTArray.h"

namespace mozilla {

// Thread and type safe wrapper around nsDeque.
template <class T>
class MediaQueueDeallocator : public nsDequeFunctor {
  virtual void* operator() (void* aObject) {
    nsRefPtr<T> releaseMe = dont_AddRef(static_cast<T*>(aObject));
    return nullptr;
  }
};

template <class T>
class MediaQueue : private nsDeque {
  struct Listener {
    virtual ~Listener() {}
    virtual void Dispatch(T* aItem) = 0;
  };

  template<typename Function>
  class PopListener : public Listener {
  public:
    explicit PopListener(const Function& aFunction, TaskQueue* aTarget)
      : mFunction(aFunction), mTarget(aTarget) {}

    void Dispatch(T* aItem) override {
      nsRefPtr<T> item = aItem;
      Function function = mFunction;
      nsCOMPtr<nsIRunnable> r = NS_NewRunnableFunction([=] () {
        function(item);
      });
      mTarget->Dispatch(r.forget());
    }
  private:
    Function mFunction;
    nsRefPtr<TaskQueue> mTarget;
  };

  void NotifyPopListeners(T* aItem) {
    for (auto&& l : mPopListeners) {
      l->Dispatch(aItem);
    }
  }

public:
  MediaQueue()
    : nsDeque(new MediaQueueDeallocator<T>()),
      mReentrantMonitor("mediaqueue"),
      mEndOfStream(false)
  {}

  ~MediaQueue() {
    Reset();
  }

  inline int32_t GetSize() {
    ReentrantMonitorAutoEnter mon(mReentrantMonitor);
    return nsDeque::GetSize();
  }

  inline void Push(T* aItem) {
    ReentrantMonitorAutoEnter mon(mReentrantMonitor);
    MOZ_ASSERT(aItem);
    NS_ADDREF(aItem);
    nsDeque::Push(aItem);
  }

  inline void PushFront(T* aItem) {
    ReentrantMonitorAutoEnter mon(mReentrantMonitor);
    MOZ_ASSERT(aItem);
    NS_ADDREF(aItem);
    nsDeque::PushFront(aItem);
  }

  inline already_AddRefed<T> PopFront() {
    ReentrantMonitorAutoEnter mon(mReentrantMonitor);
    nsRefPtr<T> rv = dont_AddRef(static_cast<T*>(nsDeque::PopFront()));
    if (rv) {
      NotifyPopListeners(rv);
    }
    return rv.forget();
  }

  inline T* Peek() {
    ReentrantMonitorAutoEnter mon(mReentrantMonitor);
    return static_cast<T*>(nsDeque::Peek());
  }

  inline T* PeekFront() {
    ReentrantMonitorAutoEnter mon(mReentrantMonitor);
    return static_cast<T*>(nsDeque::PeekFront());
  }

  void Reset() {
    ReentrantMonitorAutoEnter mon(mReentrantMonitor);
    while (GetSize() > 0) {
      nsRefPtr<T> x = PopFront();
    }
    mEndOfStream = false;
  }

  bool AtEndOfStream() {
    ReentrantMonitorAutoEnter mon(mReentrantMonitor);
    return GetSize() == 0 && mEndOfStream;
  }

  // Returns true if the media queue has had its last item added to it.
  // This happens when the media stream has been completely decoded. Note this
  // does not mean that the corresponding stream has finished playback.
  bool IsFinished() {
    ReentrantMonitorAutoEnter mon(mReentrantMonitor);
    return mEndOfStream;
  }

  // Informs the media queue that it won't be receiving any more items.
  void Finish() {
    ReentrantMonitorAutoEnter mon(mReentrantMonitor);
    mEndOfStream = true;
  }

  // Returns the approximate number of microseconds of items in the queue.
  int64_t Duration() {
    ReentrantMonitorAutoEnter mon(mReentrantMonitor);
    if (GetSize() == 0) {
      return 0;
    }
    T* last = Peek();
    T* first = PeekFront();
    return last->GetEndTime() - first->mTime;
  }

  void LockedForEach(nsDequeFunctor& aFunctor) const {
    ReentrantMonitorAutoEnter mon(mReentrantMonitor);
    ForEach(aFunctor);
  }

  // Extracts elements from the queue into aResult, in order.
  // Elements whose start time is before aTime are ignored.
  void GetElementsAfter(int64_t aTime, nsTArray<nsRefPtr<T>>* aResult) {
    ReentrantMonitorAutoEnter mon(mReentrantMonitor);
    if (!GetSize())
      return;
    int32_t i;
    for (i = GetSize() - 1; i > 0; --i) {
      T* v = static_cast<T*>(ObjectAt(i));
      if (v->GetEndTime() < aTime)
        break;
    }
    // Elements less than i have a end time before aTime. It's also possible
    // that the element at i has a end time before aTime, but that's OK.
    for (; i < GetSize(); ++i) {
      nsRefPtr<T> elem = static_cast<T*>(ObjectAt(i));
      aResult->AppendElement(elem);
    }
  }

  void GetFirstElements(uint32_t aMaxElements, nsTArray<nsRefPtr<T>>* aResult) {
    ReentrantMonitorAutoEnter mon(mReentrantMonitor);
    for (int32_t i = 0; i < (int32_t)aMaxElements && i < GetSize(); ++i) {
      *aResult->AppendElement() = static_cast<T*>(ObjectAt(i));
    }
  }

  uint32_t FrameCount() {
    ReentrantMonitorAutoEnter mon(mReentrantMonitor);
    uint32_t frames = 0;
    for (int32_t i = 0; i < GetSize(); ++i) {
      T* v = static_cast<T*>(ObjectAt(i));
      frames += v->mFrames;
    }
    return frames;
  }

  void ClearListeners() {
    ReentrantMonitorAutoEnter mon(mReentrantMonitor);
    mPopListeners.Clear();
  }

  template<typename Function>
  void AddPopListener(const Function& aFunction, TaskQueue* aTarget) {
    ReentrantMonitorAutoEnter mon(mReentrantMonitor);
    mPopListeners.AppendElement()->reset(
      new PopListener<Function>(aFunction, aTarget));
  }

private:
  mutable ReentrantMonitor mReentrantMonitor;
  nsTArray<UniquePtr<Listener>> mPopListeners;
  // True when we've decoded the last frame of data in the
  // bitstream for which we're queueing frame data.
  bool mEndOfStream;
};

} // namespace mozilla

#endif
