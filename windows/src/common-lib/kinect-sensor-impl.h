#pragma once

#include "holojs/private/chakra.h"
#include "holojs/private/kinect-interface.h"
#include <memory>
#include <mutex>

#include <Kinect.h>
#include <k4a/k4a.h>

namespace HoloJs {
class IHoloJsScriptHostInternal;
}

namespace HoloJs {
namespace Platforms {
namespace Win32 {

template <class FrameSourceInterface, class FrameReaderInterface, class SingleFrameInterface>
class KinectFrameListener {
   public:
    bool stopping = false;
    HANDLE frameEvent = nullptr;
    HANDLE waitObject = nullptr;
    FrameSourceInterface* frameSource = nullptr;
    FrameReaderInterface* frameReader = nullptr;
    std::function<HRESULT(SingleFrameInterface*)> callback;
    std::map<const char*, byte*> buffers;

    byte* getBuffer(const char* name, unsigned int byteCount)
    {
        byte* result = nullptr;
        if (buffers.count(name) == 0) {
            auto buffer = (byte*)malloc(byteCount);
            buffers[name] = buffer;
        }

        return buffers[name];
    }

    HRESULT connect(FrameSourceInterface* _frameSource, std::function<HRESULT(SingleFrameInterface*)> _callback)
    {
        stopping = false;
        frameSource = _frameSource;
        callback = _callback;
        waitObject = nullptr;

        RETURN_IF_FAILED(frameSource->SubscribeFrameCaptured((WAITABLE_HANDLE*)&frameEvent));
        RETURN_IF_FALSE(
            RegisterWaitForSingleObject(&waitObject, frameEvent, worker, this, INFINITE, WT_EXECUTEDEFAULT));

        return S_OK;
    }

    HRESULT disconnect()
    {
        stopping = true;

        RETURN_IF_FAILED(stop());

        if (frameSource) {
            RETURN_IF_FAILED(frameSource->UnsubscribeFrameCaptured((WAITABLE_HANDLE)frameEvent));
            SafeRelease(&frameSource);
        }

        if (frameEvent) {
            ::SetEvent(frameEvent);
        }

        if (waitObject) {
            ::UnregisterWaitEx(waitObject, INVALID_HANDLE_VALUE);
        }

        frameEvent = waitObject = nullptr;
        frameSource = nullptr;

        for (auto const& item : buffers) {
            free(item.second);
        }
        buffers.clear();

        return S_OK;
    }

    HRESULT start()
    {
        RETURN_IF_FAILED(stop());
        RETURN_IF_FAILED(frameSource->OpenReader(&frameReader));

        return S_OK;
    }

    HRESULT stop()
    {
        if (frameReader) {
            SafeRelease(&frameReader);
            frameReader = nullptr;
        }

        return S_OK;
    }

    static void CALLBACK worker(void* context, BOOLEAN TimerOrWaitFired)
    {
        auto p = static_cast<KinectFrameListener<FrameSourceInterface, FrameReaderInterface, SingleFrameInterface>*>(
            context);
        if (p->stopping) {
            return;
        }

        SingleFrameInterface* singleFrame = nullptr;
        if (SUCCEEDED(p->frameReader->AcquireLatestFrame(&singleFrame))) {
            p->callback(singleFrame);
            SafeRelease(&singleFrame);
        }
    }
};

class KinectMultiFrameListener {
   public:
    bool stopping = false;
    HANDLE frameEvent = nullptr;
    HANDLE waitObject = nullptr;
    IMultiSourceFrameReader* frameReader = nullptr;
    std::function<HRESULT(IMultiSourceFrame*)> callback;
    std::map<const char*, byte*> buffers;

    byte* getBuffer(const char* name, unsigned int byteCount)
    {
        byte* result = nullptr;
        if (buffers.count(name) == 0) {
            auto buffer = (byte*)malloc(byteCount);
            buffers[name] = buffer;
        }

        return buffers[name];
    }

    HRESULT connect(std::function<HRESULT(IMultiSourceFrame*)> _callback)
    {
        stopping = false;
        callback = _callback;
        waitObject = nullptr;

        RETURN_IF_FAILED(frameReader->SubscribeMultiSourceFrameArrived((WAITABLE_HANDLE*)&frameEvent));
        RETURN_IF_FALSE(
            RegisterWaitForSingleObject(&waitObject, frameEvent, worker, this, INFINITE, WT_EXECUTEDEFAULT));

        return S_OK;
    }

    HRESULT disconnect()
    {
        stopping = true;

        RETURN_IF_FAILED(stop());

        if (frameReader) {
            frameReader->UnsubscribeMultiSourceFrameArrived((WAITABLE_HANDLE)frameEvent);
        }

        if (frameEvent) {
            ::SetEvent(frameEvent);
        }

        if (waitObject) {
            ::UnregisterWaitEx(waitObject, INVALID_HANDLE_VALUE);
        }

        frameEvent = waitObject = nullptr;

        for (auto const& item : buffers) {
            free(item.second);
        }
        buffers.clear();

        return S_OK;
    }

    HRESULT start(IKinectSensor* sensor)
    {
        RETURN_IF_FAILED(stop());
        RETURN_IF_FAILED(sensor->OpenMultiSourceFrameReader(
            FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color, &frameReader));

        return S_OK;
    }

    HRESULT stop()
    {
        if (frameReader) {
            frameReader->Release();
            frameReader = nullptr;
        }

        return S_OK;
    }

    static void CALLBACK worker(void* context, BOOLEAN TimerOrWaitFired)
    {
        auto p = static_cast<KinectMultiFrameListener*>(context);
        if (p->stopping) {
            return;
        }

        IMultiSourceFrame* singleFrame = nullptr;
        if (SUCCEEDED(p->frameReader->AcquireLatestFrame(&singleFrame))) {
            p->callback(singleFrame);
            singleFrame->Release();
        }
    }
};

class Kinect4aFrameListener {
   public:
    bool stopping = false;
    HANDLE rundownEvent = nullptr;
    HANDLE waitObject = nullptr;
    k4a_device_t device = nullptr;
    std::function<HRESULT(k4a_capture_t)> callback;
    std::map<const char*, byte*> buffers;

    byte* getBuffer(const char* name, unsigned int byteCount)
    {
        byte* result = nullptr;
        if (buffers.count(name) == 0) {
            auto buffer = (byte*)malloc(byteCount);
            buffers[name] = buffer;
        }

        return buffers[name];
    }

    HRESULT connect(std::function<HRESULT(k4a_capture_t)> _callback)
    {
        stopping = false;
        callback = _callback;
        waitObject = nullptr;

        rundownEvent = CreateEvent(nullptr, false, false, nullptr);
        RETURN_IF_FALSE(RegisterWaitForSingleObject(&waitObject, rundownEvent, worker, this, 10, WT_EXECUTEDEFAULT));

        return S_OK;
    }

    HRESULT disconnect()
    {
        stopping = true;

        RETURN_IF_FAILED(stop());

        if (rundownEvent) {
            ::SetEvent(rundownEvent);
        }

        if (waitObject) {
            ::UnregisterWaitEx(waitObject, INVALID_HANDLE_VALUE);
            waitObject = nullptr;
        }

        if (rundownEvent) {
            CloseHandle(rundownEvent);
            rundownEvent = nullptr;
        }

        for (auto const& item : buffers) {
            free(item.second);
        }
        buffers.clear();

        return S_OK;
    }

    HRESULT start(k4a_device_t _device)
    {
        RETURN_IF_FAILED(stop());
        device = _device;

        return S_OK;
    }

    HRESULT stop()
    {
        device = nullptr;
        return S_OK;
    }

    static void CALLBACK worker(void* context, BOOLEAN TimerOrWaitFired)
    {
        auto p = static_cast<Kinect4aFrameListener*>(context);
        if (p->stopping) {
            return;
        }

        k4a_capture_t capture = nullptr;
        switch (k4a_device_get_capture(p->device, &capture, 1000)) {
            case K4A_WAIT_RESULT_SUCCEEDED:
                break;
            case K4A_WAIT_RESULT_TIMEOUT:
                return;
            case K4A_WAIT_RESULT_FAILED:
                return;
        }

        p->callback(capture);

        k4a_capture_release(capture);
    }
};

class Kinect : public HoloJs::IKinect, public HoloJs::EventTarget {
   public:
    Kinect(HoloJs::IHoloJsScriptHostInternal* host);
    ~Kinect();

    virtual void Release() {}

    virtual long start(JsValueRef kinectRef);
    virtual long stop();

   private:
    bool m_isStarted = false;
    HoloJs::IHoloJsScriptHostInternal* m_host = nullptr;
    JsValueRef m_kinectRef = nullptr;

    HMODULE m_kinectDll = nullptr;
    IKinectSensor* m_pKinectSensor = nullptr;
    ICoordinateMapper* m_mapper = nullptr;
    KinectFrameListener<IDepthFrameSource, IDepthFrameReader, IDepthFrame> m_depthFrameListener;
    KinectMultiFrameListener m_multiFrameListener;
    std::mutex m_depthFrameLock;
    HRESULT depthFrameWorker(IDepthFrame* frame);
    HRESULT multiFrameWorker(IMultiSourceFrame* frame);
    HRESULT initKinect2();

    CRITICAL_SECTION m_workerLock;
    k4a_image_t m_xy_table = nullptr;
    k4a_image_t m_point_cloud = nullptr;
    k4a_image_t m_transformed_color_image = nullptr;
    k4a_device_t m_device = nullptr;
    k4a_transformation_t m_transformation = nullptr;
    HRESULT initKinect4a();
    Kinect4aFrameListener m_k4aListener;
    HRESULT k4aFrameWorker(k4a_capture_t);
};

}  // namespace Win32
}  // namespace Platforms
}  // namespace HoloJs
