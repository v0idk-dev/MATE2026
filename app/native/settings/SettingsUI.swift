import AVFoundation
import AppKit
import CoreImage
import Foundation
import FoundationModels
import Security
import SwiftUI
import Vision

// ─── Model ────────────────────────────────────────────────────────────────────

struct RobotNetwork: Codable, Identifiable, Hashable {
    var id: String { ssid }
    var ssid: String
    var pass: String
}

// PhotoDefaults — old fields (focal, sensorW, baseline, plateW, plateH) are
// retained for backward compatibility with existing settings.json files; the
// Photogrammetry pane no longer surfaces focal/sensorW (those come from the
// per-camera calibrations now), and baseline/plateW/plateH defaults follow
// the MATE 2026 spec (10 cm plates, baseline measured from rig).
struct PhotoDefaults: Codable {
    // Legacy — still decoded so old settings.json files round-trip cleanly.
    var focal: String = "35.0"
    var sensorW: String = "36.0"
    // Currently used.
    var baseline: String = "0.15"  // meters, default placeholder
    var plateW: String = "0.1"  // meters — 10 cm per spec
    var plateH: String = "0.1"  // meters — 10 cm per spec
    // New fields (Codable defaults make them optional in old json).
    var plateColorR: Double = 128  // Plate-color picker RGB (0–255).
    var plateColorG: Double = 0  // Default purple-ish (R128 G0 B255).
    var plateColorB: Double = 255
    var plateColorTol: Double = 25  // HSV-hue tolerance in degrees.
    var expectedPlateCount: Int = 8
    var underwater: Bool = false
    // Default AI model for the Enhance button.
    // Format: "<provider>:<model>", e.g. "openai:gpt-4o", "apple:on-device".
    var defaultAIModel: String = ""
}

struct UndistortModel: Codable, Identifiable, Hashable {
    var id: String
    var name: String
}

// Stereo extrinsics file — produced by the standalone stereo_calibrate.py
// tool, imported into the Photogrammetry pane. Stored on disk in
// stereoExtrinsicsDir alongside the per-camera pkls (different extension).
struct StereoExtrinsics: Codable, Identifiable, Hashable {
    var id: String
    var name: String
}

// AI provider availability flags. True iff a key is configured for that
// provider in the keychain. Apple Intelligence has no key; its flag tracks
// "user has enabled Apple Intelligence as a fallback" instead.
struct AIProvidersFlags: Codable {
    var openai: Bool = false
    var anthropic: Bool = false
    var google: Bool = false
    var appleIntelligenceEnabled: Bool = false
}

struct CameraSlotDefaults: Codable {
    var TL: String? = nil
    var TR: String? = nil
    var BL: String? = nil
    var BR: String? = nil
}

struct ScreenshotCameras: Codable {
    var crabChannel: String = "CH01"
    var photogrammetryLeft: String = "CH03"
    var photogrammetryRight: String = "CH04"
}

struct Prefs: Codable {
    var networks: [RobotNetwork] = []
    var selectedSsid: String? = nil
    var autoJoinEnabled: Bool = false
    var autoJoinSsid: String? = nil
    var photo: PhotoDefaults = PhotoDefaults()
    var undistortModels: [UndistortModel] = []
    var activeUndistortId: String? = nil  // existing — single-feed undistort
    // NEW — Photogrammetry-specific calibration role assignments:
    var photogrammetryLeftCalibId: String? = nil
    var photogrammetryRightCalibId: String? = nil
    var stereoExtrinsicsFiles: [StereoExtrinsics] = []
    var activeStereoExtrinsicsId: String? = nil
    // NEW — AI provider config flags (keys themselves live in Keychain):
    var aiProviders: AIProvidersFlags = AIProvidersFlags()
    // Existing camera/screenshot fields:
    var defaultCameraName: String? = nil
    var defaultCameraSlots: CameraSlotDefaults = CameraSlotDefaults()
    var screenshotCameras: ScreenshotCameras = ScreenshotCameras()
    var taskFiles: [TaskFile] = []
    var activeTaskFileId: String? = nil
}

@MainActor
final class Store: ObservableObject {
    static let shared = Store()
    @Published var p = Prefs()

    var appSupportDir: URL {
        let base = FileManager.default.urls(for: .applicationSupportDirectory, in: .userDomainMask)
            .first!
        let dir = base.appendingPathComponent("mate-2026-robot-controller")
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        return dir
    }
    var undistortDir: URL {
        let dir = appSupportDir.appendingPathComponent("undistort")
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        return dir
    }
    // Stereo extrinsics yaml files live alongside the per-camera pkls.
    var stereoExtrinsicsDir: URL {
        let dir = appSupportDir.appendingPathComponent("stereo_extrinsics")
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        return dir
    }
    private var fileURL: URL { appSupportDir.appendingPathComponent("settings.json") }

    func load() {
        guard let data = try? Data(contentsOf: fileURL),
            let dec = try? JSONDecoder().decode(Prefs.self, from: data)
        else { return }
        p = dec
    }
    func save() {
        guard let data = try? JSONEncoder().encode(p),
            let obj = try? JSONSerialization.jsonObject(with: data),
            let pretty = try? JSONSerialization.data(withJSONObject: obj, options: .prettyPrinted)
        else { return }
        try? pretty.write(to: fileURL)
        NotificationCenter.default.post(name: .settingsDidChange, object: nil)
    }
}

extension Notification.Name {
    static let settingsDidChange = Notification.Name("SettingsDidChange")
    static let settingsTabChanged = Notification.Name("MATESettingsTabChanged")
}

// ─── Keychain helpers ────────────────────────────────────────────────────────
// AI provider keys are stored in the macOS Keychain under a single service
// name with provider as the account. This keeps them out of settings.json
// and gives the user a single place (Keychain Access.app) to audit/revoke.

private let _kKeychainService = "dev.v0idk.mate2026.app.ai"

@discardableResult
func keychainSet(provider: String, key: String) -> Bool {
    let val = key.data(using: .utf8) ?? Data()
    let query: [String: Any] = [
        kSecClass as String: kSecClassGenericPassword,
        kSecAttrService as String: _kKeychainService,
        kSecAttrAccount as String: provider,
    ]
    let attrs: [String: Any] = [kSecValueData as String: val]
    var status = SecItemUpdate(query as CFDictionary, attrs as CFDictionary)
    if status == errSecItemNotFound {
        var add = query
        add[kSecValueData as String] = val
        status = SecItemAdd(add as CFDictionary, nil)
    }
    return status == errSecSuccess
}

func keychainGet(provider: String) -> String? {
    let query: [String: Any] = [
        kSecClass as String: kSecClassGenericPassword,
        kSecAttrService as String: _kKeychainService,
        kSecAttrAccount as String: provider,
        kSecReturnData as String: true,
        kSecMatchLimit as String: kSecMatchLimitOne,
    ]
    var item: CFTypeRef?
    let status = SecItemCopyMatching(query as CFDictionary, &item)
    guard status == errSecSuccess, let d = item as? Data,
        let s = String(data: d, encoding: .utf8)
    else { return nil }
    return s
}

@discardableResult
func keychainDelete(provider: String) -> Bool {
    let query: [String: Any] = [
        kSecClass as String: kSecClassGenericPassword,
        kSecAttrService as String: _kKeychainService,
        kSecAttrAccount as String: provider,
    ]
    let status = SecItemDelete(query as CFDictionary)
    return status == errSecSuccess || status == errSecItemNotFound
}

func keychainHas(provider: String) -> Bool {
    return keychainGet(provider: provider) != nil
}

// Apple Intelligence availability check. Returns true on macOS 26+ if
// FoundationModels.framework is present. Conservative — actual model
// availability is checked at AI-call time in step 9.
func appleIntelligenceAvailable() -> Bool {
    let path = "/System/Library/Frameworks/FoundationModels.framework/Resources"
    return FileManager.default.fileExists(atPath: path)
}

// ─── Networking pane (UNCHANGED) ─────────────────────────────────────────────

struct NetworkingPane: View {
    @ObservedObject var store = Store.shared
    @State private var newSSID = ""
    @State private var newPass = ""
    @State private var liveSSID: String? = nil
    let timer = Timer.publish(every: 5, on: .main, in: .common).autoconnect()

    private var onRobotNet: Bool {
        store.p.autoJoinEnabled && liveSSID == store.p.autoJoinSsid && liveSSID != nil
    }

    var body: some View {
        Form {
            Section {
                Toggle(
                    isOn: Binding(
                        get: { store.p.autoJoinEnabled },
                        set: { on in
                            store.p.autoJoinEnabled = on
                            if on, let sel = store.p.selectedSsid {
                                store.p.autoJoinSsid = sel
                                attemptJoin(ssid: sel)
                            } else {
                                store.p.autoJoinSsid = nil
                            }
                            store.save()
                        }
                    )
                ) {
                    VStack(alignment: .leading, spacing: 2) {
                        Text("Auto-Join Robot Network")
                        Text(
                            store.p.autoJoinEnabled
                                ? "Rejoins \"\(store.p.autoJoinSsid ?? "—")\" when in range"
                                : "Select a network below, then enable"
                        )
                        .font(.caption).foregroundStyle(.secondary)
                    }
                }
                HStack(spacing: 6) {
                    Circle().fill(onRobotNet ? Color.green : Color.secondary.opacity(0.4))
                        .frame(width: 8, height: 8)
                    Text(onRobotNet ? "Connected — \"\(liveSSID!)\"" : "Not on robot network")
                        .font(.caption).foregroundStyle(.secondary)
                }
            } header: {
                Text("Robot Network")
            }

            Section(header: Text("Saved Networks")) {
                if store.p.networks.isEmpty {
                    Text("No saved networks").foregroundStyle(.secondary)
                } else {
                    ForEach(store.p.networks, id: \.ssid) { networkRow($0) }
                    Button("Delete All", role: .destructive) {
                        store.p.networks = []
                        store.p.selectedSsid = nil
                        store.save()
                    }
                }
            }
            Section(header: Text("Add Network")) {
                TextField("Network Name (SSID)", text: $newSSID)
                SecureField("Password (leave blank if open)", text: $newPass)
                Button("Add Network") {
                    guard !newSSID.isEmpty else { return }
                    store.p.networks.removeAll { $0.ssid == newSSID }
                    store.p.networks.append(RobotNetwork(ssid: newSSID, pass: newPass))
                    newSSID = ""
                    newPass = ""
                    store.save()
                }.disabled(newSSID.isEmpty)
            }
        }
        .formStyle(.grouped)
        .scrollContentBackground(.hidden)
        .background(.clear)
        .onAppear { pollSSID() }
        .onReceive(timer) { _ in pollSSID() }
    }

    @ViewBuilder func networkRow(_ net: RobotNetwork) -> some View {
        HStack {
            VStack(alignment: .leading, spacing: 1) {
                Text(net.ssid)
                if !net.pass.isEmpty {
                    Text("Password saved").font(.caption2).foregroundStyle(.secondary)
                }
            }
            Spacer()
            if store.p.selectedSsid == net.ssid {
                Text("Selected").font(.caption2).bold()
                    .padding(.horizontal, 6).padding(.vertical, 2)
                    .background(Color.accentColor.opacity(0.15))
                    .foregroundStyle(Color.accentColor).clipShape(Capsule())
            } else {
                Button("Select") {
                    store.p.selectedSsid = net.ssid
                    store.save()
                }.buttonStyle(.borderless)
            }
            Button {
                store.p.networks.removeAll { $0.ssid == net.ssid }
                if store.p.selectedSsid == net.ssid { store.p.selectedSsid = nil }
                store.save()
            } label: {
                Image(systemName: "minus.circle.fill").foregroundStyle(.red)
            }.buttonStyle(.borderless)
        }
    }

    func pollSSID() {
        Task.detached {
            let ssid = await readSSID()
            await MainActor.run { liveSSID = ssid }
        }
    }
    func attemptJoin(ssid: String) {
        let pass = store.p.networks.first { $0.ssid == ssid }?.pass ?? ""
        Task.detached {
            let t = Process()
            t.executableURL = URL(fileURLWithPath: "/usr/sbin/networksetup")
            t.arguments =
                pass.isEmpty
                ? ["-setairportnetwork", "en0", ssid] : ["-setairportnetwork", "en0", ssid, pass]
            try? t.run()
            t.waitUntilExit()
        }
    }
}

func readSSID() async -> String? {
    let t = Process()
    let p = Pipe()
    t.executableURL = URL(fileURLWithPath: "/usr/sbin/networksetup")
    t.arguments = ["-getairportnetwork", "en0"]
    t.standardOutput = p
    try? t.run()
    t.waitUntilExit()
    let out = String(data: p.fileHandleForReading.readDataToEndOfFile(), encoding: .utf8) ?? ""
    guard let r = out.range(of: "Current Wi-Fi Network: ") else { return nil }
    return String(out[r.upperBound...]).trimmingCharacters(in: .whitespacesAndNewlines)
}

// ─── Photogrammetry pane (rewritten) ─────────────────────────────────────────

struct PhotogrammetryPane: View {
    @ObservedObject var store = Store.shared
    @State private var isImportingExtrinsics = false
    @State private var renamingExtrinsics: StereoExtrinsics? = nil
    @State private var errorMsg: String? = nil

    @State private var plateColor: Color = .purple
    @State private var didInit = false

    var body: some View {
        Form {
            // ── Per-camera calibration role assignment ──────────────────────
            Section {
                if store.p.undistortModels.isEmpty {
                    Text("Import calibration files in the Cameras pane first.")
                        .foregroundStyle(.secondary).font(.caption)
                } else {
                    HStack {
                        Text("Left Camera Calibration")
                        Spacer()
                        Picker("", selection: leftCalibBinding) {
                            Text("None").tag("")
                            ForEach(store.p.undistortModels) { m in
                                Text(m.name).tag(m.id)
                            }
                        }.frame(width: 160)
                    }
                    HStack {
                        Text("Right Camera Calibration")
                        Spacer()
                        Picker("", selection: rightCalibBinding) {
                            Text("None").tag("")
                            ForEach(store.p.undistortModels) { m in
                                Text(m.name).tag(m.id)
                            }
                        }.frame(width: 160)
                    }
                }
            } header: {
                Text("Per-Camera Calibrations")
            } footer: {
                Text(
                    "Each rig camera needs its own .pkl from the standalone calibration tool. Import them in the Cameras pane, then assign roles here."
                )
                .font(.caption).foregroundStyle(.secondary)
            }

            // ── Stereo extrinsics (rig geometry) ─────────────────────────────
            Section {
                if store.p.stereoExtrinsicsFiles.isEmpty {
                    Text("No stereo extrinsics imported.")
                        .foregroundStyle(.secondary).font(.caption)
                } else {
                    ForEach(store.p.stereoExtrinsicsFiles) { f in extrinsicsRow(f) }
                }
                HStack {
                    Button {
                        isImportingExtrinsics = true
                    } label: {
                        Label("Import .yaml File…", systemImage: "plus.circle")
                    }
                    Spacer()
                    if !store.p.stereoExtrinsicsFiles.isEmpty {
                        Button("Reset", role: .destructive) {
                            for f in store.p.stereoExtrinsicsFiles {
                                try? FileManager.default.removeItem(
                                    at: store.stereoExtrinsicsDir
                                        .appendingPathComponent(f.id + ".yaml"))
                            }
                            store.p.stereoExtrinsicsFiles = []
                            store.p.activeStereoExtrinsicsId = nil
                            store.save()
                        }
                    }
                }
            } header: {
                Text("Stereo Extrinsics (rig geometry)")
            } footer: {
                Text(
                    "Produced by the standalone stereo_calibrate.py tool. Provides the rigid R, T transform between the two cameras."
                )
                .font(.caption).foregroundStyle(.secondary)
            }

            // ── Plate appearance ─────────────────────────────────────────────
            Section {
                LabeledContent("Plate Color") {
                    ColorPicker("", selection: $plateColor, supportsOpacity: false)
                        .labelsHidden()
                        .onChange(of: plateColor) { _, newValue in
                            let (r, g, b) = colorToRGB(newValue)
                            store.p.photo.plateColorR = r
                            store.p.photo.plateColorG = g
                            store.p.photo.plateColorB = b
                            store.save()
                        }
                }
                LabeledContent("Hue Tolerance (deg)") {
                    HStack {
                        Slider(
                            value: Binding(
                                get: { store.p.photo.plateColorTol },
                                set: {
                                    store.p.photo.plateColorTol = $0
                                    store.save()
                                }
                            ), in: 5...60, step: 1
                        )
                        .frame(width: 160)
                        Text("\(Int(store.p.photo.plateColorTol))")
                            .font(.caption.monospacedDigit())
                            .frame(width: 28, alignment: .trailing)
                    }
                }
            } header: {
                Text("Plate Appearance")
            } footer: {
                Text(
                    "Detection runs in HSV. The picker sets a target hue; tolerance widens the matching band."
                )
                .font(.caption).foregroundStyle(.secondary)
            }

            // ── Geometry defaults ────────────────────────────────────────────
            Section {
                field("Plate Width (m)", "0.10", $store.p.photo.plateW)
                field("Plate Height (m)", "0.10", $store.p.photo.plateH)
                field("Baseline (m)", "0.15", $store.p.photo.baseline)
                HStack {
                    Text("Expected Plate Count")
                    Spacer()
                    Text("\(store.p.photo.expectedPlateCount)")
                        .foregroundStyle(.primary)
                        .monospacedDigit()
                    Stepper(
                        "",
                        value: Binding(
                            get: { store.p.photo.expectedPlateCount },
                            set: {
                                store.p.photo.expectedPlateCount = max(1, min(32, $0))
                                store.save()
                            }
                        ), in: 1...32
                    )
                    .labelsHidden()
                }
                Toggle(
                    "Underwater (apply refraction correction)",
                    isOn: Binding(
                        get: { store.p.photo.underwater },
                        set: {
                            store.p.photo.underwater = $0
                            store.save()
                        }
                    ))
            } header: {
                Text("Geometry Defaults")
            } footer: {
                Text(
                    "Pre-fills the Task 1.2 page. Plate size = 0.1 m per the MATE 2026 spec. Baseline is auto-read from stereo extrinsics when one is selected; this is the manual override."
                )
                .font(.caption).foregroundStyle(.secondary)
            }

            // ── Default AI model for Enhance button ──────────────────────────
            Section {
                HStack {
                    Text("Default Model")
                    Spacer()
                    Picker(
                        "",
                        selection: Binding(
                            get: { store.p.photo.defaultAIModel },
                            set: {
                                store.p.photo.defaultAIModel = $0
                                store.save()
                            }
                        )
                    ) {
                        Text("(none)").tag("")
                        if store.p.aiProviders.openai {
                            Text("OpenAI · gpt-5.5-pro").tag("openai:gpt-5.5-pro")
                            Text("OpenAI · gpt-5.5").tag("openai:gpt-5.5")
                            Text("OpenAI · gpt-5.4").tag("openai:gpt-5.4")
                            Text("OpenAI · gpt-5.4-mini").tag("openai:gpt-5.4-mini")
                        }
                        if store.p.aiProviders.anthropic {
                            Text("Anthropic · claude-opus-4-7").tag("anthropic:claude-opus-4-7")
                            Text("Anthropic · claude-sonnet-4-6").tag("anthropic:claude-sonnet-4-6")
                            Text("Anthropic · claude-haiku-4-5").tag("anthropic:claude-haiku-4-5")
                        }
                        if store.p.aiProviders.google {
                            Text("Google · gemini-3.1-pro").tag("google:gemini-3.1-pro-preview")
                            Text("Google · gemini-3-flash").tag("google:gemini-3-flash-preview")
                            Text("Google · gemini-2.5-pro").tag("google:gemini-2.5-pro")
                            Text("Google · gemini-2.5-flash").tag("google:gemini-2.5-flash")
                        }
                        if store.p.aiProviders.appleIntelligenceEnabled {
                            Text("Apple Intelligence (on-device)").tag("apple:on-device")
                        }
                    }.frame(width: 200)
                        .disabled(!hasAnyAIProvider)
                }
                if !hasAnyAIProvider {
                    Text(
                        "Configure at least one provider in the Models pane to enable model selection."
                    )
                    .font(.caption).foregroundStyle(.secondary)
                }
            } header: {
                Text("Enhancement Defaults")
            }

            if let err = errorMsg {
                Section { Text(err).foregroundStyle(.red).font(.caption) }
            }
        }
        .formStyle(.grouped)
        .scrollContentBackground(.hidden)
        .background(.clear)
        .onAppear {
            if !didInit {
                plateColor = Color(
                    red: store.p.photo.plateColorR / 255.0,
                    green: store.p.photo.plateColorG / 255.0,
                    blue: store.p.photo.plateColorB / 255.0)
                didInit = true
            }
        }
        .fileImporter(
            isPresented: $isImportingExtrinsics,
            allowedContentTypes: [
                .init(filenameExtension: "yaml")!,
                .init(filenameExtension: "yml")!,
            ],
            allowsMultipleSelection: true
        ) { result in
            switch result {
            case .success(let urls): urls.forEach { importExtrinsics($0) }
            case .failure(let e): errorMsg = e.localizedDescription
            }
        }
        .sheet(item: $renamingExtrinsics) { f in
            ExtrinsicsRenameSheet(
                file: f,
                onSave: { name in
                    if let i = store.p.stereoExtrinsicsFiles.firstIndex(where: { $0.id == f.id }) {
                        store.p.stereoExtrinsicsFiles[i].name = name
                        store.save()
                    }
                    renamingExtrinsics = nil
                },
                onCancel: { renamingExtrinsics = nil })
        }
    }

    // ── Helpers ──────────────────────────────────────────────────────────────

    private var hasAnyAIProvider: Bool {
        store.p.aiProviders.openai
            || store.p.aiProviders.anthropic
            || store.p.aiProviders.google
            || store.p.aiProviders.appleIntelligenceEnabled
    }

    private var leftCalibBinding: Binding<String> {
        Binding(
            get: { store.p.photogrammetryLeftCalibId ?? "" },
            set: {
                store.p.photogrammetryLeftCalibId = $0.isEmpty ? nil : $0
                store.save()
            })
    }
    private var rightCalibBinding: Binding<String> {
        Binding(
            get: { store.p.photogrammetryRightCalibId ?? "" },
            set: {
                store.p.photogrammetryRightCalibId = $0.isEmpty ? nil : $0
                store.save()
            })
    }

    @ViewBuilder
    private func extrinsicsRow(_ f: StereoExtrinsics) -> some View {
        HStack {
            VStack(alignment: .leading, spacing: 2) {
                Text(f.name).fontWeight(
                    store.p.activeStereoExtrinsicsId == f.id ? .semibold : .regular)
                Text(f.id + ".yaml").font(.caption2).foregroundStyle(.secondary)
            }
            Spacer()
            if store.p.activeStereoExtrinsicsId == f.id {
                Text("Active").font(.caption2).bold()
                    .padding(.horizontal, 6).padding(.vertical, 2)
                    .background(Color.accentColor.opacity(0.15))
                    .foregroundStyle(Color.accentColor).clipShape(Capsule())
                Button("Deactivate") {
                    store.p.activeStereoExtrinsicsId = nil
                    store.save()
                }.buttonStyle(.borderless).font(.caption)
            } else {
                Button("Set Active") {
                    store.p.activeStereoExtrinsicsId = f.id
                    store.save()
                }.buttonStyle(.borderless)
            }
            Button {
                renamingExtrinsics = f
            } label: {
                Image(systemName: "pencil").foregroundStyle(.secondary)
            }.buttonStyle(.borderless)
            Button {
                try? FileManager.default.removeItem(
                    at: store.stereoExtrinsicsDir.appendingPathComponent(f.id + ".yaml"))
                store.p.stereoExtrinsicsFiles.removeAll { $0.id == f.id }
                if store.p.activeStereoExtrinsicsId == f.id {
                    store.p.activeStereoExtrinsicsId = nil
                }
                store.save()
            } label: {
                Image(systemName: "minus.circle.fill").foregroundStyle(.red)
            }.buttonStyle(.borderless)
        }
    }

    private func importExtrinsics(_ url: URL) {
        let ok = url.startAccessingSecurityScopedResource()
        defer { if ok { url.stopAccessingSecurityScopedResource() } }
        let id = UUID().uuidString
        let dest = store.stereoExtrinsicsDir.appendingPathComponent(id + ".yaml")
        do {
            try FileManager.default.copyItem(at: url, to: dest)
            store.p.stereoExtrinsicsFiles.append(
                StereoExtrinsics(
                    id: id,
                    name: url.deletingPathExtension().lastPathComponent))
            store.save()
            errorMsg = nil
        } catch { errorMsg = "Failed: \(error.localizedDescription)" }
    }

    @ViewBuilder
    func field(_ label: String, _ ph: String, _ text: Binding<String>) -> some View {
        LabeledContent(label) {
            TextField(ph, text: text)
                .frame(width: 80)
                .multilineTextAlignment(.trailing)
                .onSubmit { store.save() }
                .onChange(of: text.wrappedValue) { _, _ in store.save() }
        }
    }

    private func colorToRGB(_ c: Color) -> (Double, Double, Double) {
        let ns = NSColor(c).usingColorSpace(.deviceRGB) ?? NSColor.purple
        return (
            Double(ns.redComponent) * 255.0,
            Double(ns.greenComponent) * 255.0,
            Double(ns.blueComponent) * 255.0
        )
    }
}

struct ExtrinsicsRenameSheet: View {
    let file: StereoExtrinsics
    let onSave: (String) -> Void
    let onCancel: () -> Void
    @State private var text: String
    init(
        file: StereoExtrinsics,
        onSave: @escaping (String) -> Void,
        onCancel: @escaping () -> Void
    ) {
        self.file = file
        self.onSave = onSave
        self.onCancel = onCancel
        _text = State(initialValue: file.name)
    }
    var body: some View {
        VStack(spacing: 16) {
            Text("Rename Stereo Extrinsics").font(.headline)
            TextField("Name", text: $text).textFieldStyle(.roundedBorder).frame(width: 280)
            HStack {
                Button("Cancel", action: onCancel).keyboardShortcut(.cancelAction)
                Button("Save") { onSave(text.trimmingCharacters(in: .whitespaces)) }
                    .keyboardShortcut(.defaultAction)
                    .disabled(text.trimmingCharacters(in: .whitespaces).isEmpty)
            }
        }.padding(24).frame(minWidth: 340)
    }
}

// ─── AI pane (new) ───────────────────────────────────────────────────────────

struct AIPane: View {
    @ObservedObject var store = Store.shared
    @State private var showSetSheet: ProviderID? = nil
    @State private var errorMsg: String? = nil

    enum ProviderID: String, Identifiable {
        case openai, anthropic, google
        var id: String { rawValue }
        var label: String {
            switch self {
            case .openai: return "OpenAI"
            case .anthropic: return "Anthropic"
            case .google: return "Google"
            }
        }
        var helpURL: String {
            switch self {
            case .openai: return "https://platform.openai.com/api-keys"
            case .anthropic: return "https://console.anthropic.com/settings/keys"
            case .google: return "https://aistudio.google.com/app/apikey"
            }
        }
        var keyHint: String {
            switch self {
            case .openai: return "sk-…"
            case .anthropic: return "sk-ant-…"
            case .google: return "AI…"
            }
        }
    }

    var body: some View {
        Form {
            Section {
                providerRow(.openai, configured: store.p.aiProviders.openai)
                providerRow(.anthropic, configured: store.p.aiProviders.anthropic)
                providerRow(.google, configured: store.p.aiProviders.google)
            } header: {
                Text("Cloud Providers")
            } footer: {
                Text(
                    "Keys are stored in your macOS Keychain (service: dev.v0idk.mate2026.app.ai). They are never written to settings.json. Only the renderer is told whether a key exists."
                )
                .font(.caption).foregroundStyle(.secondary)
            }

            Section {
                let avail = appleIntelligenceAvailable()
                HStack {
                    VStack(alignment: .leading, spacing: 2) {
                        Text("Apple Intelligence")
                        Text(
                            avail
                                ? "On-device. Available on this Mac."
                                : "Requires macOS 26 with Apple Intelligence enabled."
                        )
                        .font(.caption).foregroundStyle(.secondary)
                    }
                    Spacer()
                    Toggle(
                        "",
                        isOn: Binding(
                            get: { store.p.aiProviders.appleIntelligenceEnabled },
                            set: {
                                store.p.aiProviders.appleIntelligenceEnabled = $0
                                store.save()
                            }
                        )
                    ).labelsHidden().disabled(!avail)
                }
            } header: {
                Text("On-Device")
            } footer: {
                Text(
                    "Apple Intelligence runs entirely on this Mac via the FoundationModels framework. No key required, no data leaves the device — but the on-device model is smaller than the cloud frontier models, so accuracy is best-effort. Use as a final fallback."
                )
                .font(.caption).foregroundStyle(.secondary)
            }

            if let err = errorMsg {
                Section { Text(err).foregroundStyle(.red).font(.caption) }
            }
        }
        .formStyle(.grouped)
        .scrollContentBackground(.hidden)
        .background(.clear)
        .sheet(item: $showSetSheet) { p in
            SetAPIKeySheet(
                provider: p,
                onSave: { key in
                    if keychainSet(provider: p.rawValue, key: key) {
                        switch p {
                        case .openai: store.p.aiProviders.openai = true
                        case .anthropic: store.p.aiProviders.anthropic = true
                        case .google: store.p.aiProviders.google = true
                        }
                        store.save()
                        errorMsg = nil
                    } else {
                        errorMsg = "Failed to save \(p.label) key to Keychain."
                    }
                    showSetSheet = nil
                },
                onCancel: { showSetSheet = nil })
        }
    }

    @ViewBuilder
    private func providerRow(_ p: ProviderID, configured: Bool) -> some View {
        HStack {
            VStack(alignment: .leading, spacing: 2) {
                Text(p.label)
                Text(configured ? "Key configured" : "No key")
                    .font(.caption)
                    .foregroundStyle(configured ? .green : .secondary)
            }
            Spacer()
            if configured {
                Button("Replace…") { showSetSheet = p }.buttonStyle(.borderless)
                Button(role: .destructive) {
                    keychainDelete(provider: p.rawValue)
                    switch p {
                    case .openai: store.p.aiProviders.openai = false
                    case .anthropic: store.p.aiProviders.anthropic = false
                    case .google: store.p.aiProviders.google = false
                    }
                    let prefix = p.rawValue + ":"
                    if store.p.photo.defaultAIModel.hasPrefix(prefix) {
                        store.p.photo.defaultAIModel = ""
                    }
                    store.save()
                } label: {
                    Image(systemName: "minus.circle.fill").foregroundStyle(.red)
                }.buttonStyle(.borderless)
            } else {
                Button("Set Key…") { showSetSheet = p }.buttonStyle(.borderless)
            }
        }
    }
}

struct SetAPIKeySheet: View {
    let provider: AIPane.ProviderID
    let onSave: (String) -> Void
    let onCancel: () -> Void
    @State private var text: String = ""

    var body: some View {
        VStack(alignment: .leading, spacing: 14) {
            Text("Set \(provider.label) API Key").font(.headline)
            Text(
                "Pasting a key stores it in your macOS Keychain. The app will use it to call \(provider.label) on your behalf — the key never leaves this Mac except as part of authenticated requests to \(provider.label)."
            )
            .font(.caption).foregroundStyle(.secondary)
            .fixedSize(horizontal: false, vertical: true)
            SecureField(provider.keyHint, text: $text)
                .textFieldStyle(.roundedBorder)
            HStack {
                Link("Where to get a key", destination: URL(string: provider.helpURL)!)
                    .font(.caption)
                Spacer()
                Button("Cancel", action: onCancel).keyboardShortcut(.cancelAction)
                Button("Save") {
                    let trimmed = text.trimmingCharacters(in: .whitespacesAndNewlines)
                    guard !trimmed.isEmpty else { return }
                    onSave(trimmed)
                }
                .keyboardShortcut(.defaultAction)
                .disabled(text.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty)
            }
        }.padding(24).frame(minWidth: 420)
    }
}

// ─── Cameras pane (UNCHANGED behaviour; one cleanup for orphaned roles) ──────

struct CamerasPane: View {
    @ObservedObject var store = Store.shared
    @State private var isImporting = false
    @State private var renamingModel: UndistortModel? = nil
    @State private var errorMsg: String? = nil
    @State private var availableCameras: [String] = []
    @State private var defaultPickerSel: String = ""

    var body: some View {
        Form {
            Section {
                if availableCameras.isEmpty {
                    Text("No cameras detected.").foregroundStyle(.secondary)
                } else {
                    Picker("Default Camera Feed", selection: $defaultPickerSel) {
                        Text("None").tag("")
                        ForEach(availableCameras, id: \.self) { Text($0).tag($0) }
                    }
                    .onChange(of: defaultPickerSel) { _, newValue in
                        store.p.defaultCameraName = newValue.isEmpty ? nil : newValue
                        store.save()
                    }
                }
            } header: {
                Text("Default Camera Feed")
            }
            Section {
                if store.p.undistortModels.isEmpty {
                    Text("No calibration files uploaded yet.").foregroundStyle(.secondary)
                } else {
                    ForEach(store.p.undistortModels) { modelRow($0) }
                }
                HStack {
                    Button {
                        isImporting = true
                    } label: {
                        Label("Import .pkl File…", systemImage: "plus.circle")
                    }
                    Spacer()
                    Button("Reset", role: .destructive) {
                        store.p.undistortModels = []
                        store.p.activeUndistortId = nil
                        // Also clear photogrammetry role assignments, since the
                        // referenced ids no longer exist.
                        store.p.photogrammetryLeftCalibId = nil
                        store.p.photogrammetryRightCalibId = nil
                        store.save()
                    }
                }
            } header: {
                Text("Undistort Calibration Files")
            }
            if let err = errorMsg { Section { Text(err).foregroundStyle(.red).font(.caption) } }
        }
        .formStyle(.grouped)
        .scrollContentBackground(.hidden)
        .background(.clear)
        .onAppear {
            defaultPickerSel = store.p.defaultCameraName ?? ""
            fetchCameras()
        }
        .fileImporter(
            isPresented: $isImporting,
            allowedContentTypes: [.init(filenameExtension: "pkl")!],
            allowsMultipleSelection: true
        ) { result in
            switch result {
            case .success(let urls): urls.forEach { importPkl($0) }
            case .failure(let e): errorMsg = e.localizedDescription
            }
        }
        .sheet(item: $renamingModel) { model in
            RenameSheet(
                model: model,
                onSave: { name in
                    if let i = store.p.undistortModels.firstIndex(where: { $0.id == model.id }) {
                        store.p.undistortModels[i].name = name
                        store.save()
                    }
                    renamingModel = nil
                },
                onCancel: { renamingModel = nil })
        }
    }

    @ViewBuilder func modelRow(_ model: UndistortModel) -> some View {
        HStack {
            VStack(alignment: .leading, spacing: 2) {
                Text(model.name).fontWeight(
                    store.p.activeUndistortId == model.id ? .semibold : .regular)
                Text(model.id + ".pkl").font(.caption2).foregroundStyle(.secondary)
            }
            Spacer()
            if store.p.activeUndistortId == model.id {
                Text("Active").font(.caption2).bold()
                    .padding(.horizontal, 6).padding(.vertical, 2)
                    .background(Color.accentColor.opacity(0.15))
                    .foregroundStyle(Color.accentColor).clipShape(Capsule())
                Button("Deactivate") {
                    store.p.activeUndistortId = nil
                    store.save()
                }
                .buttonStyle(.borderless).font(.caption)
            } else {
                Button("Set Active") {
                    store.p.activeUndistortId = model.id
                    store.save()
                }.buttonStyle(.borderless)
            }
            Button {
                renamingModel = model
            } label: {
                Image(systemName: "pencil").foregroundStyle(.secondary)
            }.buttonStyle(.borderless)
            Button {
                try? FileManager.default.removeItem(
                    at: store.undistortDir.appendingPathComponent(model.id + ".pkl"))
                store.p.undistortModels.removeAll { $0.id == model.id }
                if store.p.activeUndistortId == model.id { store.p.activeUndistortId = nil }
                if store.p.photogrammetryLeftCalibId == model.id {
                    store.p.photogrammetryLeftCalibId = nil
                }
                if store.p.photogrammetryRightCalibId == model.id {
                    store.p.photogrammetryRightCalibId = nil
                }
                store.save()
            } label: {
                Image(systemName: "minus.circle.fill").foregroundStyle(.red)
            }.buttonStyle(.borderless)
        }
    }

    func fetchCameras() {
        Task {
            guard let url = URL(string: "http://localhost:5001/api/cameras"),
                let (data, _) = try? await URLSession.shared.data(from: url),
                let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
                let cams = json["cameras"] as? [[String: Any]]
            else { return }
            let names = cams.compactMap { $0["name"] as? String }
            await MainActor.run {
                availableCameras = names
                if defaultPickerSel.isEmpty, let saved = store.p.defaultCameraName {
                    defaultPickerSel = saved
                }
            }
        }
    }

    func importPkl(_ url: URL) {
        let ok = url.startAccessingSecurityScopedResource()
        defer { if ok { url.stopAccessingSecurityScopedResource() } }
        let id = UUID().uuidString
        let dest = store.undistortDir.appendingPathComponent(id + ".pkl")
        do {
            try FileManager.default.copyItem(at: url, to: dest)
            store.p.undistortModels.append(
                UndistortModel(id: id, name: url.deletingPathExtension().lastPathComponent))
            store.save()
            errorMsg = nil
        } catch { errorMsg = "Failed: \(error.localizedDescription)" }
    }
}

struct RenameSheet: View {
    let model: UndistortModel
    let onSave: (String) -> Void
    let onCancel: () -> Void
    @State private var text: String
    init(model: UndistortModel, onSave: @escaping (String) -> Void, onCancel: @escaping () -> Void)
    {
        self.model = model
        self.onSave = onSave
        self.onCancel = onCancel
        _text = State(initialValue: model.name)
    }
    var body: some View {
        VStack(spacing: 16) {
            Text("Rename Calibration File").font(.headline)
            TextField("Name", text: $text).textFieldStyle(.roundedBorder).frame(width: 280)
            HStack {
                Button("Cancel", action: onCancel).keyboardShortcut(.cancelAction)
                Button("Save") { onSave(text.trimmingCharacters(in: .whitespaces)) }
                    .keyboardShortcut(.defaultAction)
                    .disabled(text.trimmingCharacters(in: .whitespaces).isEmpty)
            }
        }.padding(24).frame(minWidth: 340)
    }
}

// ─── General pane (UNCHANGED) ────────────────────────────────────────────────

struct GeneralPane: View {
    @ObservedObject var store = Store.shared

    private let channels = ["", "CH01", "CH02", "CH03", "CH04"]
    private func channelLabel(_ ch: String) -> String { ch.isEmpty ? "None" : ch }

    var body: some View {
        Form {
            Section {
                Text("Which channel appears in each grid corner when the app launches.")
                    .font(.caption).foregroundStyle(.secondary)
                slotPicker("Top Left", binding: slotBinding(\.TL))
                slotPicker("Top Right", binding: slotBinding(\.TR))
                slotPicker("Bottom Left", binding: slotBinding(\.BL))
                slotPicker("Bottom Right", binding: slotBinding(\.BR))
                Button("Reset", role: .destructive) { Task { await resetSlotsFromDefaults() } }
            } header: {
                Text("Default Video Channels")
            }

            Section {
                Text("Channels grabbed when the Screenshot button is pressed in each task.")
                    .font(.caption).foregroundStyle(.secondary)
                HStack {
                    Text("Crab Detection (split)")
                    Spacer()
                    Picker("", selection: $store.p.screenshotCameras.crabChannel) {
                        ForEach(["CH01", "CH02", "CH03", "CH04"], id: \.self) { Text($0).tag($0) }
                    }
                    .onChange(of: store.p.screenshotCameras.crabChannel) { _, _ in store.save() }
                    .frame(width: 90)
                }
                HStack {
                    Text("Photogrammetry Left")
                    Spacer()
                    Picker("", selection: $store.p.screenshotCameras.photogrammetryLeft) {
                        ForEach(["CH01", "CH02", "CH03", "CH04"], id: \.self) { Text($0).tag($0) }
                    }
                    .onChange(of: store.p.screenshotCameras.photogrammetryLeft) { _, _ in
                        store.save()
                    }
                    .frame(width: 90)
                }
                HStack {
                    Text("Photogrammetry Right")
                    Spacer()
                    Picker("", selection: $store.p.screenshotCameras.photogrammetryRight) {
                        ForEach(["CH01", "CH02", "CH03", "CH04"], id: \.self) { Text($0).tag($0) }
                    }
                    .onChange(of: store.p.screenshotCameras.photogrammetryRight) { _, _ in
                        store.save()
                    }
                    .frame(width: 90)
                }
                Button("Reset", role: .destructive) {
                    Task { await resetScreenshotsFromDefaults() }
                }
            } header: {
                Text("Screenshot Cameras")
            }
        }
        .formStyle(.grouped)
        .scrollContentBackground(.hidden)
        .background(.clear)
    }

    func resetSlotsFromDefaults() async {
        guard let url = URL(string: "http://localhost:5001/api/cameras_defaults"),
            let (data, _) = try? await URLSession.shared.data(from: url),
            let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
            let dv = json["defaultView"] as? [String: Any?]
        else {
            store.p.defaultCameraSlots = CameraSlotDefaults()
            store.save()
            return
        }
        var slots = CameraSlotDefaults()
        slots.TL = dv["TL"] as? String
        slots.TR = dv["TR"] as? String
        slots.BL = dv["BL"] as? String
        slots.BR = dv["BR"] as? String
        await MainActor.run {
            store.p.defaultCameraSlots = slots
            store.save()
        }
    }

    func resetScreenshotsFromDefaults() async {
        guard let url = URL(string: "http://localhost:5001/api/cameras_defaults"),
            let (data, _) = try? await URLSession.shared.data(from: url),
            let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
            let sc = json["screenshots"] as? [String: Any]
        else {
            store.p.screenshotCameras = ScreenshotCameras()
            store.save()
            return
        }
        var cams = ScreenshotCameras()
        if let cd = sc["crabDetection"] as? [String: Any], let ch = cd["splitChannel"] as? String {
            cams.crabChannel = ch
        }
        if let pl = sc["photogrammetryLeft"] as? [String: Any], let ch = pl["channel"] as? String {
            cams.photogrammetryLeft = ch
        }
        if let pr = sc["photogrammetryRight"] as? [String: Any], let ch = pr["channel"] as? String {
            cams.photogrammetryRight = ch
        }
        await MainActor.run {
            store.p.screenshotCameras = cams
            store.save()
        }
    }

    @ViewBuilder
    func slotPicker(_ label: String, binding: Binding<String>) -> some View {
        HStack {
            Text(label)
            Spacer()
            Picker("", selection: binding) {
                ForEach(channels, id: \.self) { Text(channelLabel($0)).tag($0) }
            }.frame(width: 110)
        }
    }

    func slotBinding(_ kp: WritableKeyPath<CameraSlotDefaults, String?>) -> Binding<String> {
        Binding(
            get: { store.p.defaultCameraSlots[keyPath: kp] ?? "" },
            set: { store.p.defaultCameraSlots[keyPath: kp] = $0.isEmpty ? nil : $0 }
        )
    }
}

// ─── Tasks pane (UNCHANGED) ──────────────────────────────────────────────────

struct TaskFile: Codable, Identifiable, Hashable {
    var id: String
    var name: String
}

struct TasksPane: View {
    @ObservedObject var store = Store.shared
    @State private var isImporting = false
    @State private var renamingFile: TaskFile? = nil
    @State private var errorMsg: String? = nil

    var body: some View {
        Form {
            Section {
                if store.p.taskFiles.isEmpty {
                    Text("No task files uploaded yet.").foregroundStyle(.secondary)
                } else {
                    ForEach(store.p.taskFiles) { fileRow($0) }
                }
                HStack {
                    Button {
                        isImporting = true
                    } label: {
                        Label("Import .m26tl File…", systemImage: "plus.circle")
                    }
                    Spacer()
                    Button("Remove All", role: .destructive) {
                        store.p.taskFiles = []
                        store.p.activeTaskFileId = nil
                        store.save()
                    }.disabled(store.p.taskFiles.isEmpty)
                }
            } header: {
                Text("Task Order Files")
            } footer: {
                Text(
                    "Each .m26tl file defines the task run order. Select one as active to use it in the Task Counter window."
                )
                .font(.caption).foregroundStyle(.secondary)
            }
            if let err = errorMsg { Section { Text(err).foregroundStyle(.red).font(.caption) } }
        }
        .formStyle(.grouped)
        .scrollContentBackground(.hidden)
        .background(.clear)
        .fileImporter(
            isPresented: $isImporting,
            allowedContentTypes: [.init(filenameExtension: "m26tl")!],
            allowsMultipleSelection: true
        ) { result in
            switch result {
            case .success(let urls): urls.forEach { importM26tl($0) }
            case .failure(let e): errorMsg = e.localizedDescription
            }
        }
        .sheet(item: $renamingFile) { file in
            TaskRenameSheet(
                file: file,
                onSave: { name in
                    if let i = store.p.taskFiles.firstIndex(where: { $0.id == file.id }) {
                        store.p.taskFiles[i].name = name
                        store.save()
                    }
                    renamingFile = nil
                },
                onCancel: { renamingFile = nil })
        }
    }

    @ViewBuilder func fileRow(_ file: TaskFile) -> some View {
        HStack {
            VStack(alignment: .leading, spacing: 2) {
                Text(file.name).fontWeight(
                    store.p.activeTaskFileId == file.id ? .semibold : .regular)
                Text(file.id + ".m26tl").font(.caption2).foregroundStyle(.secondary)
            }
            Spacer()
            if store.p.activeTaskFileId == file.id {
                Text("Active").font(.caption2).bold()
                    .padding(.horizontal, 6).padding(.vertical, 2)
                    .background(Color.accentColor.opacity(0.15))
                    .foregroundStyle(Color.accentColor).clipShape(Capsule())
                Button("Deactivate") {
                    store.p.activeTaskFileId = nil
                    store.save()
                }.buttonStyle(.borderless).font(.caption)
            } else {
                Button("Set Active") {
                    store.p.activeTaskFileId = file.id
                    store.save()
                }.buttonStyle(.borderless)
            }
            Button {
                renamingFile = file
            } label: {
                Image(systemName: "pencil").foregroundStyle(.secondary)
            }.buttonStyle(.borderless)
            Button {
                let tasksDir = store.appSupportDir.appendingPathComponent("tasks")
                try? FileManager.default.removeItem(
                    at: tasksDir.appendingPathComponent(file.id + ".m26tl"))
                store.p.taskFiles.removeAll { $0.id == file.id }
                if store.p.activeTaskFileId == file.id { store.p.activeTaskFileId = nil }
                store.save()
            } label: {
                Image(systemName: "minus.circle.fill").foregroundStyle(.red)
            }.buttonStyle(.borderless)
        }
    }

    func importM26tl(_ url: URL) {
        let ok = url.startAccessingSecurityScopedResource()
        defer { if ok { url.stopAccessingSecurityScopedResource() } }
        let id = UUID().uuidString
        let tasksDir = store.appSupportDir.appendingPathComponent("tasks")
        try? FileManager.default.createDirectory(at: tasksDir, withIntermediateDirectories: true)
        let dest = tasksDir.appendingPathComponent(id + ".m26tl")
        do {
            try FileManager.default.copyItem(at: url, to: dest)
            let name = url.deletingPathExtension().lastPathComponent
            store.p.taskFiles.append(TaskFile(id: id, name: name))
            store.save()
            errorMsg = nil
        } catch { errorMsg = "Failed: \(error.localizedDescription)" }
    }
}

struct TaskRenameSheet: View {
    let file: TaskFile
    let onSave: (String) -> Void
    let onCancel: () -> Void
    @State private var text: String
    init(file: TaskFile, onSave: @escaping (String) -> Void, onCancel: @escaping () -> Void) {
        self.file = file
        self.onSave = onSave
        self.onCancel = onCancel
        _text = State(initialValue: file.name)
    }
    var body: some View {
        VStack(spacing: 16) {
            Text("Rename Task File").font(.headline)
            TextField("Name", text: $text).textFieldStyle(.roundedBorder).frame(width: 280)
            HStack {
                Button("Cancel", action: onCancel).keyboardShortcut(.cancelAction)
                Button("Save") { onSave(text.trimmingCharacters(in: .whitespaces)) }
                    .keyboardShortcut(.defaultAction)
                    .disabled(text.trimmingCharacters(in: .whitespaces).isEmpty)
            }
        }.padding(24).frame(minWidth: 340)
    }
}

// ─── Pane enum (added .ai) ───────────────────────────────────────────────────

enum SettingsPane: String, CaseIterable {
    case general = "general"
    case networking = "networking"
    case cameras = "cameras"
    case photogrammetry = "photogrammetry"
    case ai = "ai"
    case tasks = "tasks"

    var label: String {
        switch self {
        case .general: return "General"
        case .networking: return "Networking"
        case .cameras: return "Cameras"
        case .photogrammetry: return "Photogrammetry"
        case .ai: return "Models"
        case .tasks: return "Tasks"
        }
    }
    var icon: String {
        switch self {
        case .general: return "gearshape"
        case .networking: return "wifi"
        case .cameras: return "video"
        case .photogrammetry: return "camera.aperture"
        case .ai: return "sparkle"
        case .tasks: return "checklist"
        }
    }
}

// ─── Settings window manager ──────────────────────────────────────────────────

@MainActor
final class SettingsWindowManager: NSObject, NSToolbarDelegate {
    static let shared = SettingsWindowManager()

    private var window: NSWindow?
    private var currentPane: SettingsPane = .networking

    // Slightly taller content area to accommodate the rewritten Photogrammetry
    // pane. The window remains non-resizable.
    private let contentSize = NSSize(width: 580, height: 560)

    func show() {
        if let win = window, win.isVisible {
            win.makeKeyAndOrderFront(nil)
            NSApp.activate(ignoringOtherApps: false)
            return
        }
        let win = buildWindow()
        showPane(.general, in: win)
        win.center()
        win.makeKeyAndOrderFront(nil)
        NSApp.activate(ignoringOtherApps: false)
        self.window = win
    }

    func hide() { window?.orderOut(nil) }

    // MARK: - Window construction

    private func buildWindow() -> NSWindow {
        let win = NSWindow(
            contentRect: NSRect(origin: .zero, size: contentSize),
            styleMask: [.titled, .closable, .miniaturizable, .resizable],
            backing: .buffered,
            defer: false
        )
        win.title = "Settings"
        win.toolbarStyle = .preference
        win.setFrameAutosaveName("")  // disable autosave so size is always fixed
        win.isReleasedWhenClosed = false
        win.collectionBehavior = [.managed, .participatesInCycle]
        win.styleMask.remove(.resizable)
        // Close on Esc
        NSEvent.addLocalMonitorForEvents(matching: .keyDown) { [weak win] event in
            if event.keyCode == 53, let w = win, w.isKeyWindow {
                w.orderOut(nil)
                return nil
            }
            return event
        }

        let toolbar = NSToolbar(identifier: "MATESettingsToolbar")
        toolbar.delegate = self
        toolbar.allowsUserCustomization = false
        toolbar.displayMode = .iconAndLabel
        toolbar.selectedItemIdentifier = NSToolbarItem.Identifier(SettingsPane.general.rawValue)
        win.toolbar = toolbar

        return win
    }

    // MARK: - Pane switching

    @objc func toolbarItemClicked(_ sender: NSToolbarItem) {
        guard let pane = SettingsPane(rawValue: sender.itemIdentifier.rawValue),
            let win = window
        else { return }
        showPane(pane, in: win)
    }

    private var currentVC: NSViewController?

    private func showPane(_ pane: SettingsPane, in win: NSWindow) {
        currentPane = pane
        win.toolbar?.selectedItemIdentifier = NSToolbarItem.Identifier(pane.rawValue)

        guard let contentView = win.contentView else { return }

        currentVC?.view.removeFromSuperview()

        let vc: NSViewController
        switch pane {
        case .general: vc = NSHostingController(rootView: GeneralPane())
        case .networking: vc = NSHostingController(rootView: NetworkingPane())
        case .cameras: vc = NSHostingController(rootView: CamerasPane())
        case .photogrammetry: vc = NSHostingController(rootView: PhotogrammetryPane())
        case .ai: vc = NSHostingController(rootView: AIPane())
        case .tasks: vc = NSHostingController(rootView: TasksPane())
        }

        vc.view.translatesAutoresizingMaskIntoConstraints = false
        contentView.addSubview(vc.view)
        NSLayoutConstraint.activate([
            vc.view.topAnchor.constraint(equalTo: contentView.topAnchor),
            vc.view.bottomAnchor.constraint(equalTo: contentView.bottomAnchor),
            vc.view.leadingAnchor.constraint(equalTo: contentView.leadingAnchor),
            vc.view.trailingAnchor.constraint(equalTo: contentView.trailingAnchor),
        ])
        currentVC = vc
    }

    // MARK: - NSToolbarDelegate

    func toolbarAllowedItemIdentifiers(_ toolbar: NSToolbar) -> [NSToolbarItem.Identifier] {
        SettingsPane.allCases.map { NSToolbarItem.Identifier($0.rawValue) }
    }

    func toolbarDefaultItemIdentifiers(_ toolbar: NSToolbar) -> [NSToolbarItem.Identifier] {
        SettingsPane.allCases.map { NSToolbarItem.Identifier($0.rawValue) }
    }

    func toolbarSelectableItemIdentifiers(_ toolbar: NSToolbar) -> [NSToolbarItem.Identifier] {
        SettingsPane.allCases.map { NSToolbarItem.Identifier($0.rawValue) }
    }

    func toolbar(
        _ toolbar: NSToolbar,
        itemForItemIdentifier itemIdentifier: NSToolbarItem.Identifier,
        willBeInsertedIntoToolbar flag: Bool
    ) -> NSToolbarItem? {
        guard let pane = SettingsPane(rawValue: itemIdentifier.rawValue) else { return nil }
        let item = NSToolbarItem(itemIdentifier: itemIdentifier)
        item.label = pane.label
        item.image = NSImage(systemSymbolName: pane.icon, accessibilityDescription: pane.label)
        item.target = self
        item.action = #selector(toolbarItemClicked(_:))
        return item
    }
}

// ─── C entry points called from the .mm ObjC++ file ─────────────────────────

@_cdecl("settings_show")
public func settings_show() {
    DispatchQueue.main.async {
        Store.shared.load()
        SettingsWindowManager.shared.show()
    }
}

@_cdecl("settings_hide")
public func settings_hide() {
    DispatchQueue.main.async {
        SettingsWindowManager.shared.hide()
    }
}

// Returns a JSON array of AVFoundation camera devices:
// [{"uniqueID": "...", "name": "...", "builtin": true}, ...]
// Called synchronously from the .mm binding on a background thread — safe
// because AVCaptureDevice.DiscoverySession is thread-safe.
@_cdecl("list_cameras_json")
public func list_cameras_json() -> UnsafeMutablePointer<CChar>? {
    let all = AVCaptureDevice.devices(for: .video)
    var entries: [[String: Any]] = []
    var opencvIndex = 0
    for device in all {
        let isDesk = device.deviceType == .deskViewCamera
        entries.append([
            "uniqueID": device.uniqueID,
            "name": device.localizedName,
            "builtin": device.deviceType == .builtInWideAngleCamera,
            "index": opencvIndex,
            "hidden": isDesk,
        ])
        opencvIndex += 1
    }
    guard let data = try? JSONSerialization.data(withJSONObject: entries),
        let string = String(data: data, encoding: .utf8)
    else { return strdup("[]") }
    return strdup(string)  // caller must free()
}

// ─── AI key C entry points (called from the .mm binding) ─────────────────────

@_cdecl("ai_key_set")
public func ai_key_set(
    _ provider: UnsafePointer<CChar>?,
    _ key: UnsafePointer<CChar>?
) -> Int32 {
    guard let provider = provider, let key = key else { return 0 }
    let p = String(cString: provider)
    let k = String(cString: key)
    return keychainSet(provider: p, key: k) ? 1 : 0
}

@_cdecl("ai_key_get")
public func ai_key_get(_ provider: UnsafePointer<CChar>?) -> UnsafeMutablePointer<CChar>? {
    guard let provider = provider else { return nil }
    let p = String(cString: provider)
    guard let v = keychainGet(provider: p) else { return nil }
    return strdup(v)
}

@_cdecl("ai_key_delete")
public func ai_key_delete(_ provider: UnsafePointer<CChar>?) -> Int32 {
    guard let provider = provider else { return 0 }
    let p = String(cString: provider)
    return keychainDelete(provider: p) ? 1 : 0
}

@_cdecl("ai_key_has")
public func ai_key_has(_ provider: UnsafePointer<CChar>?) -> Int32 {
    guard let provider = provider else { return 0 }
    let p = String(cString: provider)
    return keychainHas(provider: p) ? 1 : 0
}

@_cdecl("apple_intelligence_available")
public func apple_intelligence_available() -> Int32 {
    return appleIntelligenceAvailable() ? 1 : 0
}

// Returns a JSON object describing which providers have keys configured:
// {"openai": true, "anthropic": false, "google": false, "appleIntelligence": true}
// Caller must free() the returned string.
@_cdecl("ai_providers_json")
public func ai_providers_json() -> UnsafeMutablePointer<CChar>? {
    let dict: [String: Any] = [
        "openai": keychainHas(provider: "openai"),
        "anthropic": keychainHas(provider: "anthropic"),
        "google": keychainHas(provider: "google"),
        "appleIntelligence": appleIntelligenceAvailable(),
    ]
    guard let data = try? JSONSerialization.data(withJSONObject: dict),
        let s = String(data: data, encoding: .utf8)
    else { return strdup("{}") }
    return strdup(s)
}

// Describe one image (as JPEG/PNG base64) using Vision — returns a short text
// summary of detected rectangles and dominant regions to give the LLM spatial context.
private func visionDescribe(base64: String, mimeType: String) -> String {
    guard let data = Data(base64Encoded: base64, options: .ignoreUnknownCharacters),
        let cgSrc = CGImageSourceCreateWithData(data as CFData, nil),
        let cgImg = CGImageSourceCreateImageAtIndex(cgSrc, 0, nil)
    else { return "(image decode failed)" }

    let sem2 = DispatchSemaphore(value: 0)
    var desc = "(no regions)"

    // Rectangle detection — finds plate and pipe outlines
    let rectReq = VNDetectRectanglesRequest { req, _ in
        guard let results = req.results as? [VNRectangleObservation], !results.isEmpty else {
            return
        }
        let h = CGFloat(cgImg.height)
        let w = CGFloat(cgImg.width)
        let parts = results.prefix(8).map { r -> String in
            let x = Int(r.boundingBox.minX * w)
            let y = Int((1 - r.boundingBox.maxY) * h)
            let rw = Int(r.boundingBox.width * w)
            let rh = Int(r.boundingBox.height * h)
            return "rect@(\(x),\(y)) \(rw)×\(rh)px"
        }
        desc = "Detected \(results.count) rectangle(s): " + parts.joined(separator: "; ")
        sem2.signal()
    }
    rectReq.maximumObservations = 8
    rectReq.minimumConfidence = 0.4
    rectReq.minimumAspectRatio = 0.1

    let handler = VNImageRequestHandler(cgImage: cgImg, options: [:])
    DispatchQueue.global().async {
        _ = try? handler.perform([rectReq])
        sem2.signal()  // signal even if already signalled — semaphore count just goes to 1
    }
    _ = sem2.wait(timeout: .now() + 5)
    return desc
}

@_cdecl("apple_intelligence_generate")
public func apple_intelligence_generate(_ promptC: UnsafePointer<CChar>?) -> UnsafeMutablePointer<
    CChar
>? {
    guard let promptC = promptC else { return strdup("{\"error\":\"no prompt\"}") }

    // Accept either a plain string prompt OR a JSON object {prompt, images:[{mime,b64}]}
    let raw = String(cString: promptC)
    var textPrompt = raw
    var imageDescs: [String] = []

    if let jsonData = raw.data(using: .utf8),
        let obj = (try? JSONSerialization.jsonObject(with: jsonData)) as? [String: Any],
        let p = obj["prompt"] as? String
    {
        textPrompt = p
        if let imgs = obj["images"] as? [[String: String]] {
            for (i, img) in imgs.prefix(4).enumerated() {
                let mime = img["mime"] ?? "image/jpeg"
                let b64 = img["b64"] ?? ""
                if !b64.isEmpty {
                    let d = visionDescribe(base64: b64, mimeType: mime)
                    imageDescs.append("Image \(i+1): \(d)")
                }
            }
        }
    }

    var fullPrompt = textPrompt
    if !imageDescs.isEmpty {
        fullPrompt +=
            "\n\nVision analysis of provided images:\n" + imageDescs.joined(separator: "\n")
    }

    let sem = DispatchSemaphore(value: 0)
    var outText = "{\"error\":\"timeout\"}"
    if #available(macOS 15.1, *) {
        Task {
            do {
                let session = LanguageModelSession(
                    instructions:
                        "You are a precise photogrammetry measurement assistant. Always respond with valid JSON only — no explanations, no markdown, no code blocks."
                )
                let r = try await session.respond(to: fullPrompt)
                outText = "{\"text\":" + jsonEncode(r.content) + "}"
            } catch {
                outText = "{\"error\":" + jsonEncode("\(error)") + "}"
            }
            sem.signal()
        }
    } else {
        outText = "{\"error\":\"requires macOS 15.1+\"}"
        sem.signal()
    }
    _ = sem.wait(timeout: .now() + 90)
    return strdup(outText)
}

private func jsonEncode(_ s: String) -> String {
    let d = try? JSONSerialization.data(withJSONObject: [s])
    if let d = d, var t = String(data: d, encoding: .utf8) {
        // Strip the [ and ] to leave just the encoded string.
        t.removeFirst()
        t.removeLast()
        return t
    }
    return "\"\""
}
