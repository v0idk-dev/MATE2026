#import <Cocoa/Cocoa.h>
#import <objc/runtime.h>
#include <napi.h>

// We can't use SwiftUI @main inside an addon (NSApp already running).
// Instead we build the window imperatively using NSHostingController loaded
// from the compiled SwiftUI framework that ships alongside the addon.
// Because we can't import Swift modules from ObjC++, we expose a plain-C
// entry point from a companion Swift file (settings_ui.swift) compiled into
// a dylib that this addon dlopen()s — all within the same Electron process.

#include <dlfcn.h>

static void* g_lib = nullptr;
typedef void (*VoidFn)(void);

static bool LoadUI() {
    if (g_lib) return true;
    Dl_info info;
    if (!dladdr((void*)LoadUI, &info)) return false;
    NSString* addonPath = [NSString stringWithUTF8String:info.dli_fname];
    NSString* dir = [addonPath stringByDeletingLastPathComponent];
    NSString* libPath = [dir stringByAppendingPathComponent:@"libSettingsUI.dylib"];
    g_lib = dlopen(libPath.UTF8String, RTLD_LAZY | RTLD_LOCAL);
    return g_lib != nullptr;
}

Napi::Value Open(const Napi::CallbackInfo& info) {
    dispatch_async(dispatch_get_main_queue(), ^{
        if (!LoadUI()) return;
        auto fn = (VoidFn)dlsym(g_lib, "settings_show");
        if (fn) fn();
    });
    return info.Env().Undefined();
}

Napi::Value Close(const Napi::CallbackInfo& info) {
    dispatch_async(dispatch_get_main_queue(), ^{
        if (!LoadUI()) return;
        auto fn = (VoidFn)dlsym(g_lib, "settings_hide");
        if (fn) fn();
    });
    return info.Env().Undefined();
}

// setManaged(handle: Buffer) — takes getNativeWindowHandle() buffer, sets
// collectionBehavior to [.managed, .participatesInCycle] so the window
// appears over fullscreen spaces without taking over a space of its own.
Napi::Value SetManaged(const Napi::CallbackInfo& info) {
    if (info.Length() < 1 || !info[0].IsBuffer()) return info.Env().Undefined();
    auto buf = info[0].As<Napi::Buffer<uint8_t>>();
    NSView* view = *reinterpret_cast<NSView* __strong *>(buf.Data());
    dispatch_async(dispatch_get_main_queue(), ^{
        NSWindow* win = view.window;
        if (!win) return;
        win.collectionBehavior = NSWindowCollectionBehaviorManaged
                               | NSWindowCollectionBehaviorParticipatesInCycle;
    });
    return info.Env().Undefined();
}

Napi::Object Init(Napi::Env env, Napi::Object exports) {
    exports.Set("open",       Napi::Function::New(env, Open));
    exports.Set("close",      Napi::Function::New(env, Close));
    exports.Set("setManaged", Napi::Function::New(env, SetManaged));
    // Load dylib and register scene at addon init time
    dispatch_async(dispatch_get_main_queue(), ^{ LoadUI(); });
    return exports;
}

NODE_API_MODULE(settings, Init)
