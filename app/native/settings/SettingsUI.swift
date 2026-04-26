import AppKit
import Foundation
import SwiftUI

// ─── Model ────────────────────────────────────────────────────────────────────

struct RobotNetwork: Codable, Identifiable, Hashable {
    var id: String { ssid }
    var ssid: String
    var pass: String
}

struct PhotoDefaults: Codable {
    var focal: String = "35.0"
    var sensorW: String = "36.0"
    var baseline: String = "0.1"
    var plateW: String = "0.3"
    var plateH: String = "0.2"
}

struct UndistortModel: Codable, Identifiable, Hashable {
    var id: String
    var name: String
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
    var activeUndistortId: String? = nil
    var defaultCameraName: String? = nil
    var defaultCameraSlots: CameraSlotDefaults = CameraSlotDefaults()
    var screenshotCameras: ScreenshotCameras = ScreenshotCameras()
}

@MainActor
final class Store: ObservableObject {
    static let shared = Store()
    @Published var p = Prefs()

    private var appSupportDir: URL {
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

// ─── Networking pane ─────────────────────────────────────────────────────────

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

// ─── Photogrammetry pane ──────────────────────────────────────────────────────

struct PhotogrammetryPane: View {
    @ObservedObject var store = Store.shared
    var body: some View {
        Form {
            Section {
                field("Focal Length (mm)", "35.0", $store.p.photo.focal)
                field("Sensor Width (mm)", "36.0", $store.p.photo.sensorW)
                field("Baseline (m)", "0.1", $store.p.photo.baseline)
                field("Plate Width (m)", "0.3", $store.p.photo.plateW)
                field("Plate Height (m)", "0.2", $store.p.photo.plateH)
            } header: {
                Text("Defaults")
            }
            Section {
                Text("Pre-fills the photogrammetry parameters panel in Task 1.2.")
                    .font(.caption).foregroundStyle(.secondary)
                HStack {
                    Button("Save") { store.save() }
                    Button("Reset", role: .destructive) {
                        store.p.photo = PhotoDefaults()
                        store.save()
                    }
                }
            }
        }
        .formStyle(.grouped)
        .scrollContentBackground(.hidden)
        .background(.clear)
    }
    @ViewBuilder func field(_ label: String, _ ph: String, _ text: Binding<String>) -> some View {
        LabeledContent(label) {
            TextField(ph, text: text).frame(width: 80).multilineTextAlignment(.trailing)
        }
    }
}

// ─── Cameras pane ─────────────────────────────────────────────────────────────

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
                }
                Button("Save") {
                    store.p.defaultCameraName = defaultPickerSel.isEmpty ? nil : defaultPickerSel
                    store.save()
                }.disabled(availableCameras.isEmpty)
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

// ─── General pane ─────────────────────────────────────────────────────────────

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
                HStack {
                    Button("Save") { store.save() }
                    Button("Reset", role: .destructive) { Task { await resetSlotsFromDefaults() } }
                }
            } header: {
                Text("Default Video Channels")
            }

            Section {
                Text("Channels grabbed when the Screenshot button is pressed in each task.")
                    .font(.caption).foregroundStyle(.secondary)
                LabeledContent("Crab Detection (split)") {
                    Picker("", selection: $store.p.screenshotCameras.crabChannel) {
                        ForEach(["CH01", "CH02", "CH03", "CH04"], id: \.self) { Text($0).tag($0) }
                    }.frame(width: 90)
                }
                LabeledContent("Photogrammetry Left") {
                    Picker("", selection: $store.p.screenshotCameras.photogrammetryLeft) {
                        ForEach(["CH01", "CH02", "CH03", "CH04"], id: \.self) { Text($0).tag($0) }
                    }.frame(width: 90)
                }
                LabeledContent("Photogrammetry Right") {
                    Picker("", selection: $store.p.screenshotCameras.photogrammetryRight) {
                        ForEach(["CH01", "CH02", "CH03", "CH04"], id: \.self) { Text($0).tag($0) }
                    }.frame(width: 90)
                }
                HStack {
                    Button("Save") { store.save() }
                    Button("Reset", role: .destructive) {
                        Task { await resetScreenshotsFromDefaults() }
                    }
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
        LabeledContent(label) {
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

// ─── Pane enum ────────────────────────────────────────────────────────────────

enum SettingsPane: String, CaseIterable {
    case general = "general"
    case networking = "networking"
    case cameras = "cameras"
    case photogrammetry = "photogrammetry"

    var label: String {
        switch self {
        case .general: return "General"
        case .networking: return "Networking"
        case .cameras: return "Cameras"
        case .photogrammetry: return "Photogrammetry"
        }
    }
    var icon: String {
        switch self {
        case .general: return "gearshape"
        case .networking: return "wifi"
        case .cameras: return "video"
        case .photogrammetry: return "camera.aperture"
        }
    }
}

// ─── Settings window manager ──────────────────────────────────────────────────

@MainActor
final class SettingsWindowManager: NSObject, NSToolbarDelegate {
    static let shared = SettingsWindowManager()

    private var window: NSWindow?
    private var currentPane: SettingsPane = .networking

    private let contentSize = NSSize(width: 580, height: 440)

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
