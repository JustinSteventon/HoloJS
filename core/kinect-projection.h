#pragma once

#include "include/holojs/private/chakra.h"
#include "resource-management/resource-manager.h"
#include <memory>

namespace HoloJs {

class IHoloJsScriptHostInternal;

class KinectProjection {
   public:
    KinectProjection(std::shared_ptr<HoloJs::ResourceManagement::ResourceManager> resourceManager,
                             HoloJs::IHoloJsScriptHostInternal* host)
        : m_resourceManager(resourceManager), m_host(host)
    {
    }
    ~KinectProjection() {}

    virtual void Release() {}

    long initialize();

   private:
    JS_PROJECTION_DEFINE(KinectProjection, isAvailable)
    JS_PROJECTION_DEFINE(KinectProjection, create)
    JS_PROJECTION_DEFINE(KinectProjection, start)
    JS_PROJECTION_DEFINE(KinectProjection, stop)

    std::shared_ptr<HoloJs::ResourceManagement::ResourceManager> m_resourceManager;
    HoloJs::IHoloJsScriptHostInternal* m_host;
};

}  // namespace HoloJs