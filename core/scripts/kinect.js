function Kinect() {
    navigator.holojs.nativeInterface.Kinect.create(this);

    //if (typeof this.native === 'undefined') {
    //    throw "cannot create kinect";
    //}

    this.addEventListener = function (eventType, eventHandler) {
        navigator.holojs.nativeInterface.eventRegistration.addEventListener(this.native, eventType, eventHandler.bind(this));
    };

    this.removeEventListener = function (eventType, eventHandler) {
        navigator.holojs.nativeInterface.eventRegistration.removeEventListener(this.native, eventType, eventHandler.bind(this));
    };

    this.start = function () {
        this.native = navigator.holojs.nativeInterface.Kinect.start(this.native);
    };

    this.stop = function () {
        navigator.holojs.nativeInterface.Kinect.stop(this.native);
    };

    Object.defineProperty(this, 'onnewdepthframe', {
        get: function () {
            return this.onnewdepthframeEvent;
        },
        set: function (value) {
            if (this.onnewdepthframeEvent) {
                this.removeEventListener('newdepthframe', this.onnewdepthframeEvent);
            }

            if (value) {
                this.addEventListener('newdepthframe', value);
            }

            this.onnewdepthframeEvent = value;
        }
    });
}

Kinect.isAvailable = function () {
    return navigator.holojs.nativeInterface.Kinect.isAvailable();
};