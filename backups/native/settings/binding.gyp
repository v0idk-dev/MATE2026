{
  "targets": [{
    "target_name": "settings",
    "sources": ["settings.mm"],
    "include_dirs": ["<!@(node -p \"require('node-addon-api').include\")"],
    "defines": ["NAPI_DISABLE_CPP_EXCEPTIONS"],
    "xcode_settings": {
      "CLANG_CXX_LANGUAGE_STANDARD": "c++17",
      "OTHER_LDFLAGS": [
        "-L<(module_root_dir)/build/Release",
        "-lSettingsUI",
        "-rpath", "@loader_path"
      ]
    },
    "libraries": [
      "-L<(module_root_dir)/build/Release",
      "-lSettingsUI"
    ]
  }]
}
