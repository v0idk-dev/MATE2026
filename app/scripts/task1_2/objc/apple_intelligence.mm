// apple_intelligence.mm — bridge to Apple FoundationModels (system LLM).
// Compiled with -ObjC++ -fobjc-arc on macOS 26+.
//
// FoundationModels is loaded weakly — if the framework isn't present
// (older OS, AI not enabled, unsupported hardware) `isAvailable` returns
// false and `generate` reports a friendly error.
#import <Foundation/Foundation.h>
#include "apple_intelligence.hpp"
#include <dispatch/dispatch.h>

#if __has_include(<FoundationModels/FoundationModels.h>)
  #import <FoundationModels/FoundationModels.h>
  #define MATE_HAS_FM 1
#else
  #define MATE_HAS_FM 0
#endif

namespace mate::apple_ai {

bool isAvailable() {
#if MATE_HAS_FM
    if (@available(macOS 26.0, *)) {
        Class C = NSClassFromString(@"SystemLanguageModel");
        if (!C) return false;
        id model = [C performSelector:@selector(default)];
        if (!model) return false;
        // SystemLanguageModel.availability is an enum; .available == 0
        SEL availSel = NSSelectorFromString(@"availability");
        if ([model respondsToSelector:availSel]) {
            NSMethodSignature* sig = [model methodSignatureForSelector:availSel];
            NSInvocation* inv = [NSInvocation invocationWithMethodSignature:sig];
            inv.target = model; inv.selector = availSel;
            [inv invoke];
            NSInteger v = 0; [inv getReturnValue:&v];
            return v == 0;  // available
        }
        return true;
    }
#endif
    return false;
}

bool generate(const std::string& prompt, std::string& out_text) {
#if MATE_HAS_FM
    if (!isAvailable()) { out_text = "Apple Intelligence not available"; return false; }
    if (@available(macOS 26.0, *)) {
        @autoreleasepool {
            NSString* p = [NSString stringWithUTF8String:prompt.c_str()];
            Class SC = NSClassFromString(@"LanguageModelSession");
            if (!SC) { out_text = "LanguageModelSession class missing"; return false; }
            id session = [[SC alloc] init];
            if (!session) { out_text = "Could not create session"; return false; }

            __block NSString* result = nil;
            __block NSError* gerr = nil;
            dispatch_semaphore_t sem = dispatch_semaphore_create(0);

            // session respondTo:(NSString*) completionHandler:^(NSString*,NSError*)
            SEL respSel = NSSelectorFromString(@"respondTo:completionHandler:");
            if ([session respondsToSelector:respSel]) {
                void (^cb)(NSString*, NSError*) = ^(NSString* s, NSError* e) {
                    result = s; gerr = e; dispatch_semaphore_signal(sem);
                };
                NSMethodSignature* sig = [session methodSignatureForSelector:respSel];
                NSInvocation* inv = [NSInvocation invocationWithMethodSignature:sig];
                inv.target = session; inv.selector = respSel;
                [inv setArgument:&p  atIndex:2];
                [inv setArgument:&cb atIndex:3];
                [inv invoke];
                dispatch_semaphore_wait(sem, DISPATCH_TIME_FOREVER);
                if (gerr) { out_text = std::string("FM error: ") + gerr.localizedDescription.UTF8String; return false; }
                if (!result) { out_text = "FM returned no text"; return false; }
                out_text = result.UTF8String ? result.UTF8String : "";
                return true;
            }
            out_text = "FoundationModels API shape changed";
            return false;
        }
    }
#endif
    out_text = "FoundationModels not compiled in";
    return false;
}

}  // namespace mate::apple_ai
