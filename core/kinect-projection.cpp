#include "blob.h"
#include "include/holojs/private/error-handling.h"
#include "include/holojs/private/platform-interfaces.h"
#include "include/holojs/private/script-host-utilities.h"
#include "include/holojs/private/kinect-interface.h"
#include "resource-management/external-object.h"
#include "kinect-projection.h"

#include <string>
#include <vector>

using namespace HoloJs;
using namespace HoloJs::ResourceManagement;
using namespace std;

long KinectProjection::initialize()
{
    JS_PROJECTION_REGISTER(L"Kinect", L"isAvailable", isAvailable);
    JS_PROJECTION_REGISTER(L"Kinect", L"create", create);
    JS_PROJECTION_REGISTER(L"Kinect", L"start", start);
    JS_PROJECTION_REGISTER(L"Kinect", L"stop", stop);

    return HoloJs::Success;
}

JsValueRef KinectProjection::_isAvailable(JsValueRef* arguments, unsigned short argumentCount)
{
    auto kinectAvailable = getPlatform()->isKinectAvailable();
    JsValueRef kinectAvailableRef;
    RETURN_INVALID_REF_IF_JS_ERROR(JsBoolToBoolean(kinectAvailable, &kinectAvailableRef));

    return kinectAvailableRef;
}

JsValueRef KinectProjection::_create(JsValueRef* arguments, unsigned short argumentCount)
{
    RETURN_INVALID_REF_IF_FALSE(argumentCount == 2);

    auto newKinect = getPlatform()->getKinect(m_host);
    RETURN_INVALID_REF_IF_NULL(newKinect);

    auto kinectExternal =
        m_resourceManager->objectToDirectExternal(newKinect, ObjectType::IKinect);

    RETURN_INVALID_REF_IF_FAILED(ScriptHostUtilities::SetJsProperty(arguments[1], L"native", kinectExternal));

    return JS_INVALID_REFERENCE;
}

JsValueRef KinectProjection::_start(JsValueRef* arguments, unsigned short argumentCount)
{
    RETURN_INVALID_REF_IF_FALSE(argumentCount == 2);

    auto kinect = m_resourceManager->externalToObject<IKinect>(arguments[1], ObjectType::IKinect);
    RETURN_INVALID_REF_IF_NULL(kinect);

    kinect->start(arguments[1]);

    return JS_INVALID_REFERENCE;
}

JsValueRef KinectProjection::_stop(JsValueRef* arguments, unsigned short argumentCount)
{
    RETURN_INVALID_REF_IF_FALSE(argumentCount == 2);

    auto kinect = m_resourceManager->externalToObject<IKinect>(arguments[1], ObjectType::IKinect);
    RETURN_INVALID_REF_IF_NULL(kinect);

    kinect->stop();

    return JS_INVALID_REFERENCE;
}
