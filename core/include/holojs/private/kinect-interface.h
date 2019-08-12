#pragma once

#include "chakra.h"
#include "event-target.h"
#include "holojs-view.h"
#include "object-lifetime.h"
#include "blob-interface.h"

namespace HoloJs {

class IKinect : public HoloJs::ResourceManagement::IRelease {
   public:
    virtual ~IKinect() {}

    virtual long start(JsValueRef mapperRef) = 0;
    virtual long stop() = 0;
};

}  // namespace HoloJs