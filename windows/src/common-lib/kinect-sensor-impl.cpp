#include "stdafx.h"
#include "holojs/private/error-handling.h"
#include "holojs/private/holojs-script-host-internal.h"
#include "holojs/private/script-host-utilities.h"
#include "include/holojs/windows/IBufferOnMemory.h"
#include "kinect-sensor-impl.h"
#include <algorithm>
#include <list>
#include <map>
#include <ppltasks.h>
#include <string>
#include <vector>

using namespace concurrency;

using namespace HoloJs;
using namespace HoloJs::Platforms::Win32;
using namespace std;

using namespace Windows::Perception::Spatial::Surfaces;
using namespace Windows::Perception::Spatial;
using namespace Windows::Foundation::Collections;
using namespace Windows::Graphics::DirectX;
using namespace std::placeholders;
using namespace DirectX;
using namespace Windows::Storage::Streams;

const int cDepthWidth = 512;
const int cDepthHeight = 424;
const int cColorWidth = 1920;
const int cColorHeight = 1080;

namespace {
class scope_exit {
   public:
    scope_exit(const std::function<void()>& func) : m_func(func) {}

    void dismiss() {}

    virtual ~scope_exit()
    {
        try {
            if (!m_isDismissed) {
                m_func();
            }
        } catch (...) {
        }
    }

   private:
    bool m_isDismissed = false;
    std::function<void()> m_func;
};

template <class T>
void SafeRelease(T** ppT)
{
    if (*ppT) {
        (*ppT)->Release();
        *ppT = NULL;
    }
}

}  // namespace

Kinect::Kinect(HoloJs::IHoloJsScriptHostInternal* host) : m_host(host) 
{ 
    InitializeCriticalSection(&m_workerLock); 
}

Kinect::~Kinect() 
{ 
    stop(); 
    DeleteCriticalSection(&m_workerLock);
}

// Intermediate Buffers
const int width = 512;
const int height = 424;
const int colorwidth = 1920;
const int colorheight = 1080;

unsigned char rgbimage[colorwidth * colorheight * 4];  // Stores RGB color image
ColorSpacePoint depth2rgb[width * height];             // Maps depth pixels to rgb pixels
CameraSpacePoint depth2xyz[width * height];            // Maps depth pixels to 3d coordinates

HRESULT getDepthData(ICoordinateMapper* mapper, IMultiSourceFrame* frame, byte* dest)
{
    IDepthFrame* depthframe = nullptr;
    IDepthFrameReference* frameref = nullptr;

    auto cleanup = scope_exit([&]() {
        if (frameref) {
            frameref->Release();
        }

        if (depthframe) {
            depthframe->Release();
        }
    });

    RETURN_IF_FAILED(frame->get_DepthFrameReference(&frameref));
    RETURN_IF_FAILED(frameref->AcquireFrame(&depthframe));

    // Get data from frame
    unsigned int sz;
    unsigned short* buf;
    RETURN_IF_FAILED(depthframe->AccessUnderlyingBuffer(&sz, &buf));

    // Write vertex coordinates
    RETURN_IF_FAILED(mapper->MapDepthFrameToCameraSpace(width * height, buf, width * height, depth2xyz));
    float* fdest = (float*)dest;
    for (unsigned int i = 0; i < sz; i++) {
        *fdest++ = -depth2xyz[i].X * width;
        *fdest++ = depth2xyz[i].Y * height;
        *fdest++ = -depth2xyz[i].Z * 512;
    }

    // Fill in depth2rgb map
    RETURN_IF_FAILED(mapper->MapDepthFrameToColorSpace(width * height, buf, width * height, depth2rgb));

    return S_OK;
}

HRESULT getRgbData(IMultiSourceFrame* frame, byte* dest)
{
    IColorFrame* colorframe = nullptr;
    IColorFrameReference* frameref = nullptr;

    auto cleanup = scope_exit([&]() {
        if (frameref) {
            frameref->Release();
        }

        if (colorframe) {
            colorframe->Release();
        }
    });

    RETURN_IF_FAILED(frame->get_ColorFrameReference(&frameref));
    RETURN_IF_FAILED(frameref->AcquireFrame(&colorframe));

    // Get data from frame
    RETURN_IF_FAILED(
        colorframe->CopyConvertedFrameDataToArray(colorwidth * colorheight * 4, rgbimage, ColorImageFormat_Rgba));

    // Write color array for vertices
    float* fdest = (float*)dest;
    for (int i = 0; i < width * height; i++) {
        ColorSpacePoint p = depth2rgb[i];

        // Check if color pixel coordinates are in bounds
        if (p.X < 0 || p.Y < 0 || p.X > colorwidth || p.Y > colorheight) {
            *fdest++ = 0;
            *fdest++ = 0;
            *fdest++ = 0;
        } else {
            int idx = (int)p.X + colorwidth * (int)p.Y;
            *fdest++ = rgbimage[4 * idx + 0] / 255.;
            *fdest++ = rgbimage[4 * idx + 1] / 255.;
            *fdest++ = rgbimage[4 * idx + 2] / 255.;
        }

        // Don't copy alpha channel
    }

    return S_OK;
}

HRESULT Kinect::multiFrameWorker(IMultiSourceFrame* frame)
{
    auto depthBuffer = (byte*)m_multiFrameListener.getBuffer("depthBuffer", width * height * sizeof(float) * 3);
    RETURN_IF_NULL(depthBuffer);
    RETURN_IF_FAILED(getDepthData(m_mapper, frame, depthBuffer));

    auto colorBuffer = (byte*)m_multiFrameListener.getBuffer("colorBuffer", width * height * sizeof(float) * 3);
    RETURN_IF_NULL(colorBuffer);
    RETURN_IF_FAILED(getRgbData(frame, colorBuffer));

    m_host->runInScriptContext([this, depthBuffer, colorBuffer]() {
        JsValueRef widthRef;
        EXIT_IF_FAILED(JsIntToNumber(width, &widthRef));
        JsValueRef heightRef;
        EXIT_IF_FAILED(JsIntToNumber(height, &heightRef));
        JsValueRef depthBufferRef;
        JsValueRef colorBufferRef;

        {
            std::lock_guard<std::mutex> guard(m_depthFrameLock);
            EXIT_IF_FAILED(ScriptHostUtilities::CreateTypedArrayFromBuffer(
                JsArrayTypeFloat32, &depthBufferRef, width * height * 3, (byte*)depthBuffer));

            EXIT_IF_FAILED(ScriptHostUtilities::CreateTypedArrayFromBuffer(
                JsArrayTypeFloat32, &colorBufferRef, width * height * 3, (byte*)colorBuffer));
        }

        JsValueRef depthFrameObject;
        EXIT_IF_JS_ERROR(JsCreateObject(&depthFrameObject));
        ScriptHostUtilities::SetJsProperty(depthFrameObject, L"width", widthRef);
        ScriptHostUtilities::SetJsProperty(depthFrameObject, L"height", heightRef);
        ScriptHostUtilities::SetJsProperty(depthFrameObject, L"position", depthBufferRef);
        ScriptHostUtilities::SetJsProperty(depthFrameObject, L"color", colorBufferRef);

        invokeEventListeners(L"newdepthframe", depthFrameObject);
    });

    return S_OK;
}

HRESULT Kinect::depthFrameWorker(IDepthFrame* frame)
{
    USHORT depthMinReliableDistance = 0;
    USHORT depthMaxDistance = 0;
    frame->get_DepthMinReliableDistance(&depthMinReliableDistance);
    frame->get_DepthMaxReliableDistance(&depthMaxDistance);

    auto pixelCount = cDepthWidth * cDepthHeight;

    // Temporary work buffers.
    auto vertexWorkBuffer = (float*)m_depthFrameListener.getBuffer("vertexWork", pixelCount * sizeof(float) * 3);
    RETURN_IF_NULL(vertexWorkBuffer);
    auto colorWorkBuffer = (float*)m_depthFrameListener.getBuffer("colorWork", pixelCount * sizeof(float) * 3);
    RETURN_IF_NULL(colorWorkBuffer);

    // Buffer to be used by render thread
    auto vertexBuffer = (float*)m_depthFrameListener.getBuffer("vertex", pixelCount * sizeof(float) * 3);
    RETURN_IF_NULL(vertexWorkBuffer);
    auto colorBuffer = (float*)m_depthFrameListener.getBuffer("color", pixelCount * sizeof(float) * 3);
    RETURN_IF_NULL(colorWorkBuffer);

    // Process the depth frame into a vertex buffer.
    UINT depthBufferSize = 0;
    UINT16* depthBuffer = nullptr;
    RETURN_IF_FAILED(frame->AccessUnderlyingBuffer(&depthBufferSize, &depthBuffer));

    auto currVertex = vertexWorkBuffer;
    auto currColor = colorWorkBuffer;
    auto currDepth = depthBuffer;

    for (int y = -cDepthHeight / 2; y < cDepthHeight / 2; y++) {
        for (int x = -cDepthWidth / 2; x < cDepthWidth / 2; x++) {
            float depthValue = (float)(*currDepth++);

            float color;
            if ((depthValue > depthMinReliableDistance) && (depthValue < depthMaxDistance)) {
                *currVertex++ = (float)x;
                *currVertex++ = -(float)y;
                *currVertex++ = (depthValue * 512) / 4000;
                color = 1 - (depthValue) / 8000;
            } else {
                *currVertex++ = 0;
                *currVertex++ = 0;
                *currVertex++ = 0;
                color = 0;
            }

            *currColor++ = color;
            *currColor++ = color;
            *currColor++ = color;
        }
    }

    // Copy buffers that are shared with the scripting thread.
    {
        std::lock_guard<std::mutex> guard(m_depthFrameLock);
        memcpy(vertexBuffer, vertexWorkBuffer, pixelCount * sizeof(float) * 3);
        memcpy(colorBuffer, colorWorkBuffer, pixelCount * sizeof(float) * 3);
    }

    m_host->runInScriptContext([this, pixelCount, vertexBuffer, colorBuffer]() {
        JsValueRef widthRef;
        EXIT_IF_FAILED(JsIntToNumber(cDepthWidth, &widthRef));
        JsValueRef heightRef;
        EXIT_IF_FAILED(JsIntToNumber(cDepthHeight, &heightRef));
        JsValueRef vertexBufferRef;
        JsValueRef colorBufferRef;

        {
            std::lock_guard<std::mutex> guard(m_depthFrameLock);
            EXIT_IF_FAILED(ScriptHostUtilities::CreateTypedArrayFromBuffer(
                JsArrayTypeFloat32, &vertexBufferRef, pixelCount * 3, (byte*)vertexBuffer));
            EXIT_IF_FAILED(ScriptHostUtilities::CreateTypedArrayFromBuffer(
                JsArrayTypeFloat32, &colorBufferRef, pixelCount * 3, (byte*)colorBuffer));
        }

        JsValueRef depthFrameObject;
        EXIT_IF_JS_ERROR(JsCreateObject(&depthFrameObject));
        ScriptHostUtilities::SetJsProperty(depthFrameObject, L"width", widthRef);
        ScriptHostUtilities::SetJsProperty(depthFrameObject, L"height", heightRef);
        ScriptHostUtilities::SetJsProperty(depthFrameObject, L"position", vertexBufferRef);
        ScriptHostUtilities::SetJsProperty(depthFrameObject, L"color", colorBufferRef);

        invokeEventListeners(L"newdepthframe", depthFrameObject);
    });

    return S_OK;
}

HRESULT Kinect::initKinect2()
{
    m_kinectDll = LoadLibrary(L"kinect20.dll");
    RETURN_IF_NULL(m_kinectDll);

    typedef UINT(CALLBACK * GET_DEFAULT_KINECT_SENSOR)(IKinectSensor**);
    auto pGetDefaultKinectSensor = (GET_DEFAULT_KINECT_SENSOR)GetProcAddress(m_kinectDll, "GetDefaultKinectSensor");
    RETURN_IF_NULL(pGetDefaultKinectSensor);

    RETURN_IF_FAILED(pGetDefaultKinectSensor(&m_pKinectSensor));
    RETURN_IF_FAILED(m_pKinectSensor->Open());
    m_pKinectSensor->get_CoordinateMapper(&m_mapper);

    // Register for depth frames.
    // IDepthFrameSource* depthFrameSource = nullptr;
    // RETURN_IF_FAILED(m_pKinectSensor->get_DepthFrameSource(&depthFrameSource));
    // RETURN_IF_FAILED(m_depthFrameListener.connect(depthFrameSource, std::bind(&Kinect::depthFrameWorker, this, _1)));
    // RETURN_IF_FAILED(m_depthFrameListener.start());

    RETURN_IF_FAILED(m_multiFrameListener.start(m_pKinectSensor));
    RETURN_IF_FAILED(m_multiFrameListener.connect(std::bind(&Kinect::multiFrameWorker, this, _1)));

    return S_OK;
}

static void create_xy_table(const k4a_calibration_t *calibration, k4a_image_t xy_table)
{
    k4a_float2_t *table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);

    int width = calibration->depth_camera_calibration.resolution_width;
    int height = calibration->depth_camera_calibration.resolution_height;

    k4a_float2_t p;
    k4a_float3_t ray;
    int valid;

    for (int y = 0, idx = 0; y < height; y++)
    {
        p.xy.y = (float)y;
        for (int x = 0; x < width; x++, idx++)
        {
            p.xy.x = (float)x;

            k4a_calibration_2d_to_3d(
                calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

            if (valid)
            {
                table_data[idx].xy.x = ray.xyz.x;
                table_data[idx].xy.y = ray.xyz.y;
            }
            else
            {
                table_data[idx].xy.x = nanf("");
                table_data[idx].xy.y = nanf("");
            }
        }
    }
}

static void generate_point_cloud(const k4a_image_t depth_image,
    const k4a_image_t xy_table,
    k4a_image_t point_cloud,
    int *point_count)
{
    int width = k4a_image_get_width_pixels(depth_image);
    int height = k4a_image_get_height_pixels(depth_image);

    uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(depth_image);
    k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);
    k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)k4a_image_get_buffer(point_cloud);

    *point_count = 0;
    for (int i = 0; i < width * height; i++)
    {
        if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
        {
            point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
            point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
            point_cloud_data[i].xyz.z = (float)depth_data[i];
            (*point_count)++;
        }
        else
        {
            point_cloud_data[i].xyz.x = nanf("");
            point_cloud_data[i].xyz.y = nanf("");
            point_cloud_data[i].xyz.z = nanf("");
        }
    }
}

HRESULT Kinect::k4aFrameWorker(k4a_capture_t capture)
{
    //EnterCriticalSection(&m_workerLock);
    //auto cleanup = scope_exit([this]() { LeaveCriticalSection(&m_workerLock); });

    // Capture depth and return a point cloud.
    k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
    RETURN_IF_TRUE(depth_image == nullptr);
    int width = k4a_image_get_width_pixels(depth_image);
    int height = k4a_image_get_height_pixels(depth_image);
    uint32_t pixelCount = width * height;
    if (m_point_cloud == nullptr) {
        //k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, width, height, width * sizeof(int16_t) * 3, &m_point_cloud);
        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
            width, height, width * (int)sizeof(k4a_float3_t), &m_point_cloud);
    }
    //RETURN_IF_FAILED(k4a_transformation_depth_image_to_point_cloud(
    //    m_transformation, depth_image, K4A_CALIBRATION_TYPE_DEPTH, m_point_cloud));

    int point_count = 0;
    generate_point_cloud(depth_image, m_xy_table, m_point_cloud, &point_count);


    // Capture color and map it to depth camera space.
    k4a_image_t color_image = k4a_capture_get_color_image(capture);
    RETURN_IF_TRUE(color_image == nullptr);
    int colorWidth = k4a_image_get_width_pixels(color_image);
    int colorHeight = k4a_image_get_height_pixels(color_image);
    if (m_transformed_color_image == nullptr) {
        k4a_image_create(
            K4A_IMAGE_FORMAT_COLOR_BGRA32, width, height, width * sizeof(uint32_t), &m_transformed_color_image);
    }
    RETURN_IF_FAILED(k4a_transformation_color_image_to_depth_camera(
        m_transformation, depth_image, color_image, m_transformed_color_image));

    // Compute for JS
    auto vertexBuffer = (float*)m_depthFrameListener.getBuffer("vertex", pixelCount * sizeof(float) * 3);
    RETURN_IF_NULL(vertexBuffer);
    auto colorBuffer = (float*)m_depthFrameListener.getBuffer("color", pixelCount * sizeof(float) * 3);
    RETURN_IF_NULL(colorBuffer);

    auto currDepth = (float*)(void*)k4a_image_get_buffer(m_point_cloud);
    auto currColor = (uint32_t*)(void*)k4a_image_get_buffer(m_transformed_color_image);
    auto currV = vertexBuffer;
    auto currC = colorBuffer;

    for (uint32_t i = 0; i < pixelCount; i++) {
        if (currDepth[2] != 0)
        {
            currV[0] = currDepth[0];
            currV[1] = -currDepth[1];
            currV[2] = -currDepth[2];
            auto bgra = *currColor;
            currC[0] = (float)((bgra & 0xFF0000) >> 16) / (float)0xFF;
            currC[1] = (float)((bgra & 0xFF00) >> 8) / (float)0xFF;
            currC[2] = (float)(bgra & 0xFF) / (float)0xFF;
        }
        else
        {
            currV[0] = 0;
            currV[1] = 0;
            currV[2] = 0;
            currC[0] = 0;
            currC[1] = 0;
            currC[2] = 0;
        }
        currV += 3;
        currDepth += 3;
        currC += 3;
        currColor++;
    }

    k4a_image_release(depth_image);
    k4a_image_release(color_image);

    m_host->runInScriptContext([this, width, height, pixelCount, vertexBuffer, colorBuffer]() {
        JsValueRef widthRef;
        EXIT_IF_FAILED(JsIntToNumber(width, &widthRef));
        JsValueRef heightRef;
        EXIT_IF_FAILED(JsIntToNumber(height, &heightRef));
        JsValueRef vertexBufferRef;
        JsValueRef colorBufferRef;

        EXIT_IF_FAILED(ScriptHostUtilities::CreateTypedArrayFromBuffer(
            JsArrayTypeFloat32, &vertexBufferRef, pixelCount * 3, (byte*)vertexBuffer));

        EXIT_IF_FAILED(ScriptHostUtilities::CreateTypedArrayFromBuffer(
            JsArrayTypeFloat32, &colorBufferRef, pixelCount * 3, (byte*)colorBuffer));

        JsValueRef depthFrameObject;
        EXIT_IF_JS_ERROR(JsCreateObject(&depthFrameObject));
        ScriptHostUtilities::SetJsProperty(depthFrameObject, L"width", widthRef);
        ScriptHostUtilities::SetJsProperty(depthFrameObject, L"height", heightRef);
        ScriptHostUtilities::SetJsProperty(depthFrameObject, L"position", vertexBufferRef);
        ScriptHostUtilities::SetJsProperty(depthFrameObject, L"color", colorBufferRef);

        invokeEventListeners(L"newdepthframe", depthFrameObject);
        //JsRealease
    });

    return S_OK;
}

HRESULT Kinect::initKinect4a()
{
    auto device_count = k4a_device_get_installed_count();
    RETURN_IF_TRUE(device_count == 0);

    RETURN_IF_FAILED(k4a_device_open(K4A_DEVICE_DEFAULT, &m_device));

    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;// K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;

    k4a_calibration_t calibration;
    RETURN_IF_FAILED(k4a_device_get_calibration(m_device, config.depth_mode, config.color_resolution, &calibration));
    m_transformation = k4a_transformation_create(&calibration);

    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        calibration.depth_camera_calibration.resolution_width,
        calibration.depth_camera_calibration.resolution_height,
        calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
        &m_xy_table);
    create_xy_table(&calibration, m_xy_table);

    RETURN_IF_FAILED(k4a_device_start_cameras(m_device, &config));

    m_k4aListener.connect(std::bind(&Kinect::k4aFrameWorker, this, _1));
    m_k4aListener.start(m_device);

    return S_OK;
}

long Kinect::start(JsValueRef kinectRef)
{
    RETURN_IF_TRUE(m_isStarted);

    m_isStarted = true;
    m_kinectRef = kinectRef;
    RETURN_IF_JS_ERROR(JsAddRef(m_kinectRef, nullptr));

    if (FAILED(initKinect4a())) {
        initKinect2();
    }

    return S_OK;
}

long Kinect::stop()
{
    if (m_pKinectSensor) {
        m_depthFrameListener.disconnect();
        m_multiFrameListener.disconnect();
        m_pKinectSensor->Close();
        m_pKinectSensor->Release();
        m_pKinectSensor = nullptr;
    }

    if (m_kinectDll) {
        FreeLibrary(m_kinectDll);
        m_kinectDll = nullptr;
    }

    m_k4aListener.disconnect();

    if (m_point_cloud) {
        k4a_image_release(m_point_cloud);
        m_point_cloud = nullptr;
    }

    if (m_transformed_color_image) {
        k4a_image_release(m_transformed_color_image);
        m_transformed_color_image = nullptr;
    }

    if (m_transformation) {
        k4a_transformation_destroy(m_transformation);
        m_transformation = nullptr;
    }

    if (m_device) {
        k4a_device_close(m_device);
        m_device = nullptr;
    }

    JsRelease(m_kinectRef, nullptr);

    return S_OK;
}
