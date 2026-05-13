#import <Cocoa/Cocoa.h>
#import <objc/runtime.h>
#import <objc/message.h>
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
typedef const char* (*StrFn)(void);

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

// ── Traffic-light scaling ─────────────────────────────────────────────────────
// Strategy: find the _NSThemeCloseWidgetView (the container that holds all three
// buttons) and apply a CALayer scale transform to it.  Scaling the container
// shrinks the buttons AND their spacing in one shot, and the transform survives
// hover because macOS only resets individual button frames, not the parent layer.
//
// To lock the container in place (macOS also resets its frame on hover) we use
// objc_setAssociatedObject to store the desired scale on the container and then
// swizzle its -setFrame: so any system-driven resize is immediately re-applied
// with our transform.

static CGFloat g_trafficScale = 1.0;
static BOOL    g_swizzled     = NO;
// Saved original IMPs — calling these is the only safe way to chain to the
// previous implementation. objc_msgSendSuper recurses because we replace the
// method on the class itself (not a subclass), so the lookup walks back into us.
static IMP g_orig_sendEvent = NULL;
static IMP g_orig_setFrame  = NULL;

// Swizzled -[NSTitlebarView hitTest:] — intercepts clicks in the titlebar.
// Point is in NSTitlebarView's coordinate space. We check if it falls within
// any button's VISUAL bounds (frame * scale + ty offset), and if so return
// that button directly, bypassing the widget's broken hit region.
// Swizzled -[NSWindow sendEvent:] — intercepts mouse events BEFORE macOS's
// internal titlebar routing (which bypasses hitTest: entirely).
// If a click lands on a scaled button's visual position, we rewrite the
// event's location to the button's real (unscaled) center so macOS finds it.
static void swizzled_sendEvent(id self, SEL _cmd, NSEvent* event) {
    if (event.type == NSEventTypeLeftMouseDown || event.type == NSEventTypeLeftMouseUp) {
        NSWindow* win = (NSWindow*)self;
        // Bail in fullscreen: traffic-light widget is removed/replaced by
        // AppKit during the transition; touching its geometry crashes.
        BOOL skip = (win.styleMask & NSWindowStyleMaskFullScreen) != 0
                 || ([win inLiveResize])
                 || (g_trafficScale == 1.0);
        NSButton* closeBtn = skip ? nil : [win standardWindowButton:NSWindowCloseButton];
        NSView* widget = closeBtn.superview;
        // Make sure the widget is still parented to this window's hierarchy.
        if (closeBtn && widget && widget.window == win && widget.superview) {
            @try {
                CGFloat s = g_trafficScale;
                NSRect wf = widget.frame;
                CGFloat h = wf.size.height;
                CGFloat ty = h * (1.0 - s);
                NSPoint loc = event.locationInWindow;

                NSWindowButton kinds[3] = {
                    NSWindowCloseButton, NSWindowMiniaturizeButton, NSWindowZoomButton
                };
                for (int i = 0; i < 3; i++) {
                    NSButton* btn = [win standardWindowButton:kinds[i]];
                    if (!btn) continue;
                    NSRect bf = btn.frame;

                    // Visual bounds in window coords
                    NSPoint widgetInWindow = [widget.superview
                        convertPoint:wf.origin toView:nil];
                    NSRect visualInWindow = NSMakeRect(
                        widgetInWindow.x + bf.origin.x * s,
                        widgetInWindow.y + bf.origin.y * s + ty,
                        bf.size.width * s,
                        bf.size.height * s);

                    if (NSPointInRect(loc, visualInWindow)) {
                        // Remap click to the button's real unscaled center
                        NSPoint realCenter = [widget.superview convertPoint:
                            NSMakePoint(
                                wf.origin.x + bf.origin.x + bf.size.width  * 0.5,
                                wf.origin.y + bf.origin.y + bf.size.height * 0.5)
                            toView:nil];
                        NSEvent* remapped = [NSEvent mouseEventWithType:event.type
                            location:realCenter
                            modifierFlags:event.modifierFlags
                            timestamp:event.timestamp
                            windowNumber:event.windowNumber
                            context:nil
                            eventNumber:event.eventNumber
                            clickCount:event.clickCount
                            pressure:event.pressure];

                        if (g_orig_sendEvent) {
                            ((void(*)(id, SEL, NSEvent*))g_orig_sendEvent)(self, _cmd, remapped);
                        }
                        return;
                    }
                }
            } @catch (NSException* ex) {
                // Geometry call hit a torn-down view; fall through to normal dispatch.
            }
        }
    }
    if (g_orig_sendEvent) {
        ((void(*)(id, SEL, NSEvent*))g_orig_sendEvent)(self, _cmd, event);
    }
}


// Swizzled -[_NSThemeCloseWidgetView setFrame:] — re-applies transform after hover reset.
static void swizzled_setFrame(id self, SEL _cmd, NSRect frame) {
    if (g_orig_setFrame) {
        ((void(*)(id, SEL, NSRect))g_orig_setFrame)(self, _cmd, frame);
    }
    NSNumber* stored = objc_getAssociatedObject(self, "tlScale");
    if (!stored) return;
    CGFloat s = stored.doubleValue;
    if (s == 1.0) return;
    NSView* v = (NSView*)self;
    v.wantsLayer = YES;
    CGFloat h = v.frame.size.height;
    CGFloat ty = h * (1.0 - s);
    v.layer.anchorPoint = CGPointMake(0.0, 0.0);
    CATransform3D t = CATransform3DMakeScale(s, s, 1.0);
    t = CATransform3DTranslate(t, 0, ty / s, 0);
    v.layer.transform = t;
}

static void ApplyTrafficScale(NSWindow* win, CGFloat scale) {
    NSButton* closeBtn = [win standardWindowButton:NSWindowCloseButton];
    if (!closeBtn) return;
    NSView* widget = closeBtn.superview;
    if (!widget) return;

    if (!g_swizzled) {
        // Swizzle setFrame: on the widget — save original IMP for chaining.
        Class wcls = object_getClass(widget);
        Method origSF = class_getInstanceMethod(wcls, @selector(setFrame:));
        if (origSF) {
            g_orig_setFrame = method_getImplementation(origSF);
            class_replaceMethod(wcls, @selector(setFrame:),
                (IMP)swizzled_setFrame, method_getTypeEncoding(origSF));
        }
        // Swizzle sendEvent: on this window's class. Save original IMP and
        // call it directly — objc_msgSendSuper would recurse forever because
        // we replaced the method on the class itself, not a subclass.
        Class winCls = object_getClass(win);
        Method origSE = class_getInstanceMethod(winCls, @selector(sendEvent:));
        if (origSE) {
            g_orig_sendEvent = method_getImplementation(origSE);
            class_replaceMethod(winCls, @selector(sendEvent:),
                (IMP)swizzled_sendEvent, method_getTypeEncoding(origSE));
        }

        g_swizzled = YES;
    }

    objc_setAssociatedObject(widget, "tlScale",
        [NSNumber numberWithDouble:scale],
        OBJC_ASSOCIATION_RETAIN_NONATOMIC);

    NSView* p = widget.superview;
    while (p) {
        p.wantsLayer = YES;
        p.layer.masksToBounds = NO;
        if ([NSStringFromClass(p.class) containsString:@"Titlebar"] ||
            [NSStringFromClass(p.class) containsString:@"TitleBar"]) break;
        p = p.superview;
    }

    widget.wantsLayer = YES;
    widget.layer.masksToBounds = NO;
    CGFloat h = widget.frame.size.height;
    CGFloat ty = h * (1.0 - scale);
    widget.layer.anchorPoint = CGPointMake(0.0, 0.0);
    CATransform3D t = CATransform3DMakeScale(scale, scale, 1.0);
    t = CATransform3DTranslate(t, 0, ty / scale, 0);
    widget.layer.transform = t;
}


Napi::Value ScaleTrafficLights(const Napi::CallbackInfo& info) {
    if (info.Length() < 2 || !info[0].IsBuffer() || !info[1].IsNumber())
        return info.Env().Undefined();
    auto buf     = info[0].As<Napi::Buffer<uint8_t>>();
    double scale = info[1].As<Napi::Number>().DoubleValue();
    NSView* view = *reinterpret_cast<NSView* __strong *>(buf.Data());
    dispatch_async(dispatch_get_main_queue(), ^{
        NSWindow* win = view.window;
        if (!win) return;
        g_trafficScale = scale;
        ApplyTrafficScale(win, (CGFloat)scale);
    });
    return info.Env().Undefined();
}

// Walk the menu tree, find the item whose action is toggleFullScreen:,
// and give it an SF Symbol icon. AppKit auto-injects this item into the
// View menu, so Electron's JS-side menu API can't reach it.
static void SetFullScreenIconRecursive(NSMenu* menu, NSImage* image) {
    if (!menu) return;
    for (NSMenuItem* item in menu.itemArray) {
        if (item.action == @selector(toggleFullScreen:)) {
            item.image = image;
        }
        if (item.hasSubmenu) SetFullScreenIconRecursive(item.submenu, image);
    }
}

// Suppress AppKit's auto-injected "Enter Full Screen" item. Must be called
// before NSApp builds its main menu (i.e. very early at process startup).
Napi::Value SuppressAutoFullScreen(const Napi::CallbackInfo& info) {
    [[NSUserDefaults standardUserDefaults] registerDefaults:@{
        @"NSFullScreenMenuItemEverywhere": @NO
    }];
    return info.Env().Undefined();
}

// Find a menu item by its action selector and override its displayed key
// equivalent (independent of the actual accelerator). Used so a Cmd+= binding
// renders as "⌘+" in the menu.
static NSMenuItem* FindItemByAction(NSMenu* menu, SEL action) {
    if (!menu) return nil;
    for (NSMenuItem* item in menu.itemArray) {
        if (item.action == action) return item;
        if (item.hasSubmenu) {
            NSMenuItem* found = FindItemByAction(item.submenu, action);
            if (found) return found;
        }
    }
    return nil;
}

// Override an item's displayed key equivalent. Args: (label, keyEquiv, withShift).
// Matched by exact label string.
static NSMenuItem* FindItemByLabel(NSMenu* menu, NSString* label) {
    if (!menu) return nil;
    for (NSMenuItem* item in menu.itemArray) {
        if ([item.title isEqualToString:label]) return item;
        if (item.hasSubmenu) {
            NSMenuItem* found = FindItemByLabel(item.submenu, label);
            if (found) return found;
        }
    }
    return nil;
}

Napi::Value SetMenuKeyEquivalent(const Napi::CallbackInfo& info) {
    if (info.Length() < 3 || !info[0].IsString() || !info[1].IsString() || !info[2].IsBoolean())
        return info.Env().Undefined();
    std::string labelStr = info[0].As<Napi::String>().Utf8Value();
    std::string keyStr   = info[1].As<Napi::String>().Utf8Value();
    bool shift           = info[2].As<Napi::Boolean>().Value();
    NSString* label = [NSString stringWithUTF8String:labelStr.c_str()];
    NSString* keyEq = [NSString stringWithUTF8String:keyStr.c_str()];
    __block int attempts = 0;
    __block void (^retry)(void) = nil;
    retry = [^{
        NSMenuItem* item = FindItemByLabel([NSApp mainMenu], label);
        if (item) {
            item.keyEquivalent = keyEq;
            NSEventModifierFlags mods = NSEventModifierFlagCommand;
            if (shift) mods |= NSEventModifierFlagShift;
            item.keyEquivalentModifierMask = mods;
            retry = nil;
            return;
        }
        if (++attempts < 20) {
            dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 50 * NSEC_PER_MSEC),
                dispatch_get_main_queue(), retry);
        } else {
            retry = nil;
        }
    } copy];
    dispatch_async(dispatch_get_main_queue(), retry);
    return info.Env().Undefined();
}

// Decorate AppKit's auto-injected fullscreen item: insert a separator above
// it, set the right icon for the current state, and force the correct label.
// Called repeatedly (after each menu rebuild + on FS transitions).
static void DecorateFullScreenItemNow(void) {
    NSMenuItem* item = FindItemByAction([NSApp mainMenu], @selector(toggleFullScreen:));
    if (!item) return;
    NSMenu* parent = item.menu;
    if (!parent) return;
    NSInteger idx = [parent indexOfItem:item];

    // Determine current state from the key window.
    BOOL isFS = NO;
    NSWindow* key = [NSApp keyWindow];
    if (!key) {
        for (NSWindow* w in [NSApp windows]) {
            if ((w.styleMask & NSWindowStyleMaskFullScreen) != 0) { isFS = YES; break; }
        }
    } else {
        isFS = (key.styleMask & NSWindowStyleMaskFullScreen) != 0;
    }

    item.title = isFS ? @"Exit Full Screen" : @"Enter Full Screen";

    if (@available(macOS 11.0, *)) {
        NSString* sym = isFS
            ? @"arrow.down.right.and.arrow.up.left.rectangle"
            : @"arrow.up.left.and.arrow.down.right.rectangle";
        NSImage* img = [NSImage imageWithSystemSymbolName:sym accessibilityDescription:nil];
        if (img) {
            [img setTemplate:YES];
            item.image = img;
        }
    }

    // Insert a separator above if there isn't one already.
    if (idx > 0) {
        NSMenuItem* prev = [parent itemAtIndex:idx - 1];
        if (!prev.isSeparatorItem) {
            [parent insertItem:[NSMenuItem separatorItem] atIndex:idx];
        }
    }
}

// Decorate immediately, plus retry for the case where AppKit hasn't injected
// the item yet. Also installs (once) NSWindow notification observers so the
// icon and label flip live when fullscreen is toggled — without rebuilding
// Electron's menu, which would replace the item and lose state.
static BOOL g_fsObserversInstalled = NO;
Napi::Value DecorateFullScreenMenuItem(const Napi::CallbackInfo& info) {
    __block int attempts = 0;
    __block void (^retry)(void) = nil;
    retry = [^{
        NSMenuItem* item = FindItemByAction([NSApp mainMenu], @selector(toggleFullScreen:));
        if (item) {
            DecorateFullScreenItemNow();
            retry = nil;
            return;
        }
        if (++attempts < 40) {
            dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 50 * NSEC_PER_MSEC),
                dispatch_get_main_queue(), retry);
        } else {
            retry = nil;
        }
    } copy];
    dispatch_async(dispatch_get_main_queue(), retry);

    if (!g_fsObserversInstalled) {
        g_fsObserversInstalled = YES;
        NSNotificationCenter* nc = [NSNotificationCenter defaultCenter];
        [nc addObserverForName:NSWindowDidEnterFullScreenNotification object:nil queue:nil
            usingBlock:^(NSNotification*) { DecorateFullScreenItemNow(); }];
        [nc addObserverForName:NSWindowDidExitFullScreenNotification object:nil queue:nil
            usingBlock:^(NSNotification*) { DecorateFullScreenItemNow(); }];
    }
    return info.Env().Undefined();
}

Napi::Value ListCameras(const Napi::CallbackInfo& info) {
    if (!LoadUI()) return info.Env().Null();
    auto fn = (StrFn)dlsym(g_lib, "list_cameras_json");
    if (!fn) return info.Env().Null();
    const char* json = fn();
    auto result = Napi::String::New(info.Env(), json ? json : "[]");
    free((void*)json);
    return result;
}

// ── AI provider key access (Keychain-backed; implemented in Swift) ──────────
// All five entry points are looked up by name in libSettingsUI.dylib via
// dlsym. They never block the main thread for any meaningful time (Keychain
// access is local), so we call them synchronously here.

typedef int32_t (*AIKeySetFn)(const char* provider, const char* key);
typedef char*   (*AIKeyGetFn)(const char* provider);
typedef int32_t (*AIKeyDelFn)(const char* provider);
typedef int32_t (*AIKeyHasFn)(const char* provider);
typedef int32_t (*AIAppleAvailFn)(void);
typedef char*   (*AIProvidersJsonFn)(void);

Napi::Value AIKeySet(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();
    if (info.Length() < 2 || !info[0].IsString() || !info[1].IsString())
        return Napi::Boolean::New(env, false);
    if (!LoadUI()) return Napi::Boolean::New(env, false);
    auto fn = (AIKeySetFn)dlsym(g_lib, "ai_key_set");
    if (!fn) return Napi::Boolean::New(env, false);
    std::string provider = info[0].As<Napi::String>().Utf8Value();
    std::string key      = info[1].As<Napi::String>().Utf8Value();
    int32_t ok = fn(provider.c_str(), key.c_str());
    return Napi::Boolean::New(env, ok != 0);
}

Napi::Value AIKeyGet(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();
    if (info.Length() < 1 || !info[0].IsString()) return env.Null();
    if (!LoadUI()) return env.Null();
    auto fn = (AIKeyGetFn)dlsym(g_lib, "ai_key_get");
    if (!fn) return env.Null();
    std::string provider = info[0].As<Napi::String>().Utf8Value();
    char* val = fn(provider.c_str());
    if (!val) return env.Null();
    auto result = Napi::String::New(env, val);
    free(val);
    return result;
}

Napi::Value AIKeyDelete(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();
    if (info.Length() < 1 || !info[0].IsString())
        return Napi::Boolean::New(env, false);
    if (!LoadUI()) return Napi::Boolean::New(env, false);
    auto fn = (AIKeyDelFn)dlsym(g_lib, "ai_key_delete");
    if (!fn) return Napi::Boolean::New(env, false);
    std::string provider = info[0].As<Napi::String>().Utf8Value();
    int32_t ok = fn(provider.c_str());
    return Napi::Boolean::New(env, ok != 0);
}

Napi::Value AIKeyHas(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();
    if (info.Length() < 1 || !info[0].IsString())
        return Napi::Boolean::New(env, false);
    if (!LoadUI()) return Napi::Boolean::New(env, false);
    auto fn = (AIKeyHasFn)dlsym(g_lib, "ai_key_has");
    if (!fn) return Napi::Boolean::New(env, false);
    std::string provider = info[0].As<Napi::String>().Utf8Value();
    int32_t has = fn(provider.c_str());
    return Napi::Boolean::New(env, has != 0);
}

Napi::Value AppleIntelligenceAvailable(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();
    if (!LoadUI()) return Napi::Boolean::New(env, false);
    auto fn = (AIAppleAvailFn)dlsym(g_lib, "apple_intelligence_available");
    if (!fn) return Napi::Boolean::New(env, false);
    return Napi::Boolean::New(env, fn() != 0);
}

Napi::Value AIProvidersJson(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();
    if (!LoadUI()) return Napi::String::New(env, "{}");
    auto fn = (AIProvidersJsonFn)dlsym(g_lib, "ai_providers_json");
    if (!fn) return Napi::String::New(env, "{}");
    char* json = fn();
    auto result = Napi::String::New(env, json ? json : "{}");
    if (json) free(json);
    return result;
}

typedef char* (*AIAppleGenFn)(const char*);
Napi::Value AppleIntelligenceGenerate(const Napi::CallbackInfo& info) {
    Napi::Env env = info.Env();
    if (info.Length() < 1 || !info[0].IsString())
        return Napi::String::New(env, "{\"error\":\"prompt required\"}");
    if (!LoadUI()) return Napi::String::New(env, "{\"error\":\"addon\"}");
    auto fn = (AIAppleGenFn)dlsym(g_lib, "apple_intelligence_generate");
    if (!fn) return Napi::String::New(env, "{\"error\":\"symbol\"}");
    std::string prompt = info[0].As<Napi::String>().Utf8Value();
    char* out = fn(prompt.c_str());
    auto r = Napi::String::New(env, out ? out : "{}");
    if (out) free(out);
    return r;
}

Napi::Object Init(Napi::Env env, Napi::Object exports) {
    exports.Set("open",               Napi::Function::New(env, Open));
    exports.Set("close",              Napi::Function::New(env, Close));
    exports.Set("setManaged",         Napi::Function::New(env, SetManaged));
    exports.Set("scaleTrafficLights", Napi::Function::New(env, ScaleTrafficLights));
    exports.Set("listCameras",        Napi::Function::New(env, ListCameras));
    exports.Set("decorateFullScreenMenuItem", Napi::Function::New(env, DecorateFullScreenMenuItem));
    exports.Set("suppressAutoFullScreen", Napi::Function::New(env, SuppressAutoFullScreen));
    exports.Set("setMenuKeyEquivalent", Napi::Function::New(env, SetMenuKeyEquivalent));
    // AI provider key access (Keychain-backed)
    exports.Set("aiKeySet",            Napi::Function::New(env, AIKeySet));
    exports.Set("aiKeyGet",            Napi::Function::New(env, AIKeyGet));
    exports.Set("aiKeyDelete",         Napi::Function::New(env, AIKeyDelete));
    exports.Set("aiKeyHas",            Napi::Function::New(env, AIKeyHas));
    exports.Set("appleIntelligenceAvailable",
                                       Napi::Function::New(env, AppleIntelligenceAvailable));
    exports.Set("aiProvidersJson",     Napi::Function::New(env, AIProvidersJson));
    exports.Set("appleIntelligenceGenerate", Napi::Function::New(env, AppleIntelligenceGenerate));
    // Load dylib and register scene at addon init time
    dispatch_async(dispatch_get_main_queue(), ^{ LoadUI(); });
    return exports;
}

NODE_API_MODULE(settings, Init)