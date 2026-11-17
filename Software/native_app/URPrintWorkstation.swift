import Cocoa
import WebKit

private enum HealthState {
    case unavailable
    case ready
    case conflict(String)
}

final class AppDelegate: NSObject, NSApplicationDelegate, WKNavigationDelegate, WKUIDelegate {
    private let port = 8876
    private var window: NSWindow!
    private var webView: WKWebView!
    private var loadingPanel: NSView!
    private var statusLabel: NSTextField!
    private var spinner: NSProgressIndicator!
    private var retryButton: NSButton!
    private var backendProcess: Process?
    private var backendLogHandle: FileHandle?
    private var isTerminating = false
    private var shutdownRequested = false
    private var shutdownComplete = false

    private var softwareDirectory: URL {
        let appFolder =
            Bundle.main.bundleURL.deletingLastPathComponent().standardizedFileURL
        var candidates: [URL] = []
        if
            let configured = Bundle.main.object(
                forInfoDictionaryKey: "URPrintSoftwareDirectory"
            ) as? String,
            !configured.isEmpty
        {
            candidates.append(
                URL(fileURLWithPath: configured, isDirectory: true)
                    .standardizedFileURL
            )
        }
        candidates.append(contentsOf: [
            appFolder,
            appFolder.deletingLastPathComponent().standardizedFileURL,
        ])
        return candidates.first {
            FileManager.default.fileExists(
                atPath: $0.appendingPathComponent("control_station.py").path
            )
        } ?? appFolder
    }

    private var projectRoot: URL {
        softwareDirectory.deletingLastPathComponent().standardizedFileURL
    }

    private var applicationDataDirectory: URL {
        let base = FileManager.default.urls(
            for: .applicationSupportDirectory,
            in: .userDomainMask
        ).first!
        return base.appendingPathComponent("UR Print Workstation", isDirectory: true)
    }

    private var baseURL: URL {
        URL(string: "http://127.0.0.1:\(port)/")!
    }

    private var healthURL: URL {
        URL(string: "http://127.0.0.1:\(port)/api/health")!
    }

    func applicationDidFinishLaunching(_ notification: Notification) {
        NSApp.setActivationPolicy(.regular)
        buildMenu()
        buildWindow()
        window.makeKeyAndOrderFront(nil)
        NSApp.activate(ignoringOtherApps: true)
        connectToWorkstation()
    }

    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool {
        true
    }

    func applicationShouldTerminate(_ sender: NSApplication) -> NSApplication.TerminateReply {
        if shutdownComplete {
            return .terminateNow
        }
        if shutdownRequested {
            return .terminateLater
        }
        shutdownRequested = true
        showLoading("Stopping this workstation's ROS and local runtime…", retry: false)
        requestRuntimeShutdown { [weak self] in
            DispatchQueue.main.async {
                guard let self else {
                    sender.reply(toApplicationShouldTerminate: true)
                    return
                }
                self.shutdownComplete = true
                sender.reply(toApplicationShouldTerminate: true)
            }
        }
        return .terminateLater
    }

    func applicationWillTerminate(_ notification: Notification) {
        isTerminating = true
        if let process = backendProcess, process.isRunning {
            process.terminate()
            process.waitUntilExit()
        }
        try? backendLogHandle?.close()
    }

    private func requestRuntimeShutdown(completion: @escaping () -> Void) {
        let url = baseURL.appendingPathComponent("api/runtime/shutdown")
        var request = URLRequest(url: url)
        request.httpMethod = "POST"
        request.timeoutInterval = 150
        request.setValue("application/json", forHTTPHeaderField: "Content-Type")
        request.httpBody = try? JSONSerialization.data(
            withJSONObject: ["reason": "application-exit"]
        )
        URLSession.shared.dataTask(with: request) { [weak self] _, response, error in
            let success = (response as? HTTPURLResponse).map { 200..<300 ~= $0.statusCode } ?? false
            if success {
                completion()
                return
            }
            self?.appendNativeLog(
                "Lifecycle endpoint unavailable during quit: \(error?.localizedDescription ?? "HTTP failure"). Running project stop fallback."
            )
            self?.runRuntimeStopFallback(completion: completion) ?? completion()
        }.resume()
    }

    private func runRuntimeStopFallback(completion: @escaping () -> Void) {
        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            defer { completion() }
            guard let self else { return }
            let script = self.projectRoot.appendingPathComponent("scripts/stop.sh")
            guard FileManager.default.isExecutableFile(atPath: script.path) else { return }
            let process = Process()
            process.executableURL = script
            process.currentDirectoryURL = self.projectRoot
            process.environment = self.augmentedEnvironment()
            do {
                try process.run()
                process.waitUntilExit()
            } catch {
                self.appendNativeLog("Project stop fallback failed: \(error.localizedDescription)")
            }
        }
    }

    private func appendNativeLog(_ message: String) {
        guard let data = ("\(Date()) \(message)\n").data(using: .utf8) else { return }
        try? backendLogHandle?.write(contentsOf: data)
    }

    func applicationShouldHandleReopen(_ sender: NSApplication, hasVisibleWindows flag: Bool) -> Bool {
        if !flag {
            window.makeKeyAndOrderFront(nil)
        }
        NSApp.activate(ignoringOtherApps: true)
        return true
    }

    private func buildMenu() {
        let mainMenu = NSMenu()

        let applicationItem = NSMenuItem()
        let applicationMenu = NSMenu()
        applicationMenu.addItem(
            withTitle: "About UR Print Workstation",
            action: #selector(NSApplication.orderFrontStandardAboutPanel(_:)),
            keyEquivalent: ""
        )
        applicationMenu.addItem(.separator())
        applicationMenu.addItem(
            withTitle: "Quit UR Print Workstation",
            action: #selector(NSApplication.terminate(_:)),
            keyEquivalent: "q"
        )
        applicationItem.submenu = applicationMenu
        mainMenu.addItem(applicationItem)

        let viewItem = NSMenuItem()
        let viewMenu = NSMenu(title: "View")
        viewMenu.addItem(withTitle: "Reload", action: #selector(reloadPage), keyEquivalent: "r")
        viewItem.submenu = viewMenu
        mainMenu.addItem(viewItem)

        NSApp.mainMenu = mainMenu
    }

    private func buildWindow() {
        let configuration = WKWebViewConfiguration()
        configuration.websiteDataStore = .default()
        configuration.defaultWebpagePreferences.allowsContentJavaScript = true

        webView = WKWebView(frame: .zero, configuration: configuration)
        webView.translatesAutoresizingMaskIntoConstraints = false
        webView.navigationDelegate = self
        webView.uiDelegate = self
        webView.setValue(false, forKey: "drawsBackground")

        let contentView = NSView()
        contentView.wantsLayer = true
        contentView.layer?.backgroundColor = NSColor.windowBackgroundColor.cgColor
        contentView.addSubview(webView)

        loadingPanel = NSView()
        loadingPanel.translatesAutoresizingMaskIntoConstraints = false
        loadingPanel.wantsLayer = true
        loadingPanel.layer?.backgroundColor = NSColor.windowBackgroundColor.cgColor
        contentView.addSubview(loadingPanel)

        spinner = NSProgressIndicator()
        spinner.translatesAutoresizingMaskIntoConstraints = false
        spinner.style = .spinning
        spinner.controlSize = .large
        spinner.startAnimation(nil)
        loadingPanel.addSubview(spinner)

        statusLabel = NSTextField(labelWithString: "Starting the local control service…")
        statusLabel.translatesAutoresizingMaskIntoConstraints = false
        statusLabel.alignment = .center
        statusLabel.font = .systemFont(ofSize: 15, weight: .medium)
        statusLabel.textColor = .secondaryLabelColor
        statusLabel.maximumNumberOfLines = 3
        loadingPanel.addSubview(statusLabel)

        retryButton = NSButton(title: "Try Again", target: self, action: #selector(retryConnection))
        retryButton.translatesAutoresizingMaskIntoConstraints = false
        retryButton.bezelStyle = .rounded
        retryButton.isHidden = true
        loadingPanel.addSubview(retryButton)

        NSLayoutConstraint.activate([
            webView.leadingAnchor.constraint(equalTo: contentView.leadingAnchor),
            webView.trailingAnchor.constraint(equalTo: contentView.trailingAnchor),
            webView.topAnchor.constraint(equalTo: contentView.topAnchor),
            webView.bottomAnchor.constraint(equalTo: contentView.bottomAnchor),

            loadingPanel.leadingAnchor.constraint(equalTo: contentView.leadingAnchor),
            loadingPanel.trailingAnchor.constraint(equalTo: contentView.trailingAnchor),
            loadingPanel.topAnchor.constraint(equalTo: contentView.topAnchor),
            loadingPanel.bottomAnchor.constraint(equalTo: contentView.bottomAnchor),

            spinner.centerXAnchor.constraint(equalTo: loadingPanel.centerXAnchor),
            spinner.centerYAnchor.constraint(equalTo: loadingPanel.centerYAnchor, constant: -28),
            statusLabel.topAnchor.constraint(equalTo: spinner.bottomAnchor, constant: 18),
            statusLabel.centerXAnchor.constraint(equalTo: loadingPanel.centerXAnchor),
            statusLabel.widthAnchor.constraint(lessThanOrEqualToConstant: 520),
            retryButton.topAnchor.constraint(equalTo: statusLabel.bottomAnchor, constant: 18),
            retryButton.centerXAnchor.constraint(equalTo: loadingPanel.centerXAnchor),
        ])

        let screenFrame = NSScreen.main?.visibleFrame ?? NSRect(x: 0, y: 0, width: 1440, height: 900)
        let initialSize = NSSize(
            width: min(1487, max(1100, screenFrame.width - 100)),
            height: min(980, max(760, screenFrame.height - 80))
        )
        window = NSWindow(
            contentRect: NSRect(origin: .zero, size: initialSize),
            styleMask: [.titled, .closable, .miniaturizable, .resizable, .fullSizeContentView],
            backing: .buffered,
            defer: false
        )
        window.title = "UR Print Workstation"
        window.titlebarAppearsTransparent = false
        window.minSize = NSSize(width: 1024, height: 720)
        window.contentView = contentView
        window.center()
        window.setFrameAutosaveName("URPrintWorkstationMainWindow")
    }

    @objc private func reloadPage() {
        if webView.url == nil {
            connectToWorkstation()
        } else {
            webView.reload()
        }
    }

    @objc private func retryConnection() {
        connectToWorkstation()
    }

    private func connectToWorkstation() {
        showLoading(
            "Starting the local control service… If macOS asks for Desktop access, choose Allow.",
            retry: false
        )
        checkHealth { [weak self] health in
            guard let self else { return }
            switch health {
            case .ready:
                self.loadWorkstation()
            case .unavailable:
                self.startBackend()
            case .conflict(let message):
                self.showLoading(message, retry: true)
            }
        }
    }

    private func checkHealth(completion: @escaping (HealthState) -> Void) {
        var request = URLRequest(url: healthURL)
        request.timeoutInterval = 1.0
        request.cachePolicy = .reloadIgnoringLocalCacheData
        URLSession.shared.dataTask(with: request) { [weak self] data, response, _ in
            guard let self else { return }
            let state: HealthState
            if
                let http = response as? HTTPURLResponse,
                http.statusCode == 200,
                let data,
                let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
                let reportedRoot = json["project_root"] as? String,
                let reportedVersion = json["version"] as? String,
                let demo = json["demo"] as? Bool
            {
                let expected = self.projectRoot.path
                let expectedVersion =
                    Bundle.main.object(forInfoDictionaryKey: "CFBundleShortVersionString")
                    as? String
                if reportedRoot == expected && reportedVersion == expectedVersion && !demo {
                    state = .ready
                } else if demo {
                    state = .conflict("Demo mode is already using port \(self.port). Close it, then try again.")
                } else if reportedRoot == expected {
                    state = .conflict(
                        "An older UR Print Workstation service is still running. "
                        + "Close the other workstation window, then try again."
                    )
                } else {
                    state = .conflict("Port \(self.port) is being used by another local program. Close it, then try again.")
                }
            } else {
                state = .unavailable
            }
            DispatchQueue.main.async {
                completion(state)
            }
        }.resume()
    }

    private func startBackend() {
        guard backendProcess == nil else {
            pollUntilReady(attempt: 0)
            return
        }

        let script = softwareDirectory.appendingPathComponent("control_station.py")
        guard FileManager.default.fileExists(atPath: script.path) else {
            showLoading(
                "control_station.py was not found. Rebuild the app from this project's Software folder.",
                retry: true
            )
            return
        }

        guard let python = compatiblePython() else {
            showLoading("Python 3.9 or newer is required. Install Python, then try again.", retry: true)
            return
        }

        do {
            let runtimeDirectory = applicationDataDirectory.appendingPathComponent(
                "runtime",
                isDirectory: true
            )
            try FileManager.default.createDirectory(at: runtimeDirectory, withIntermediateDirectories: true)
            let logURL = runtimeDirectory.appendingPathComponent("native-app.log")
            let backupURL = runtimeDirectory.appendingPathComponent("native-app.log.1")
            if
                let attributes = try? FileManager.default.attributesOfItem(atPath: logURL.path),
                let size = attributes[.size] as? NSNumber,
                size.int64Value > 5 * 1024 * 1024
            {
                try? FileManager.default.removeItem(at: backupURL)
                try FileManager.default.moveItem(at: logURL, to: backupURL)
            }
            if !FileManager.default.fileExists(atPath: logURL.path) {
                FileManager.default.createFile(atPath: logURL.path, contents: nil)
            }
            let logHandle = try FileHandle(forWritingTo: logURL)
            try logHandle.seekToEnd()

            let process = Process()
            process.executableURL = python
            process.arguments = [script.path, "--no-browser", "--port", String(port)]
            process.currentDirectoryURL = softwareDirectory
            process.standardOutput = logHandle
            process.standardError = logHandle
            process.environment = augmentedEnvironment()
            process.terminationHandler = { [weak self] process in
                DispatchQueue.main.async {
                    guard let self, !self.isTerminating else { return }
                    if process.terminationStatus != 0 {
                        self.backendProcess = nil
                        self.showLoading(
                            "The local control service stopped unexpectedly. See runtime/native-app.log for details.",
                            retry: true
                        )
                    }
                }
            }

            try process.run()
            backendProcess = process
            backendLogHandle = logHandle
            pollUntilReady(attempt: 0)
        } catch {
            showLoading("The local control service could not start: \(error.localizedDescription)", retry: true)
        }
    }

    private func compatiblePython() -> URL? {
        let environmentCandidate = ProcessInfo.processInfo.environment["UR_PRINT_PYTHON"]
        let candidates = [
            environmentCandidate,
            "/usr/bin/python3",
            "/opt/homebrew/bin/python3",
            "/usr/local/bin/python3",
            "/opt/anaconda3/bin/python3",
        ].compactMap { $0 }

        return candidates
            .map { URL(fileURLWithPath: $0) }
            .first { FileManager.default.isExecutableFile(atPath: $0.path) && pythonIsCompatible($0) }
    }

    private func pythonIsCompatible(_ executable: URL) -> Bool {
        let process = Process()
        let pipe = Pipe()
        process.executableURL = executable
        process.arguments = ["-c", "import sys; raise SystemExit(0 if sys.version_info >= (3, 9) else 1)"]
        process.standardOutput = pipe
        process.standardError = pipe
        do {
            try process.run()
            process.waitUntilExit()
            return process.terminationStatus == 0
        } catch {
            return false
        }
    }

    private func augmentedEnvironment() -> [String: String] {
        var environment = ProcessInfo.processInfo.environment
        let standardPaths = [
            "/opt/homebrew/bin",
            "/usr/local/bin",
            "/opt/anaconda3/bin",
            "/usr/bin",
            "/bin",
            "/usr/sbin",
            "/sbin",
        ]
        let existing = environment["PATH", default: ""]
        environment["PATH"] = (standardPaths + [existing]).joined(separator: ":")
        environment["UR_PRINT_DATA_DIR"] = applicationDataDirectory.path
        environment["UR_PRINT_PROJECT_ROOT"] = projectRoot.path
        return environment
    }

    private func pollUntilReady(attempt: Int) {
        guard attempt < 600 else {
            showLoading("The local control service did not become ready. See runtime/native-app.log for details.", retry: true)
            return
        }

        DispatchQueue.main.asyncAfter(deadline: .now() + 0.2) { [weak self] in
            guard let self else { return }
            self.checkHealth { health in
                switch health {
                case .ready:
                    self.loadWorkstation()
                case .unavailable:
                    self.pollUntilReady(attempt: attempt + 1)
                case .conflict(let message):
                    self.showLoading(message, retry: true)
                }
            }
        }
    }

    private func loadWorkstation() {
        retryButton.isHidden = true
        spinner.startAnimation(nil)
        statusLabel.stringValue = "Loading workstation…"
        webView.load(URLRequest(url: baseURL, cachePolicy: .reloadIgnoringLocalCacheData))
    }

    private func showLoading(_ message: String, retry: Bool) {
        loadingPanel.isHidden = false
        statusLabel.stringValue = message
        retryButton.isHidden = !retry
        if retry {
            spinner.stopAnimation(nil)
            spinner.isHidden = true
        } else {
            spinner.isHidden = false
            spinner.startAnimation(nil)
        }
    }

    func webView(_ webView: WKWebView, didFinish navigation: WKNavigation!) {
        loadingPanel.isHidden = true
    }

    func webView(
        _ webView: WKWebView,
        didFail navigation: WKNavigation!,
        withError error: Error
    ) {
        showLoading("The workstation could not load: \(error.localizedDescription)", retry: true)
    }

    func webView(
        _ webView: WKWebView,
        didFailProvisionalNavigation navigation: WKNavigation!,
        withError error: Error
    ) {
        showLoading("The workstation could not load: \(error.localizedDescription)", retry: true)
    }

    func webView(
        _ webView: WKWebView,
        decidePolicyFor navigationAction: WKNavigationAction,
        decisionHandler: @escaping (WKNavigationActionPolicy) -> Void
    ) {
        guard let url = navigationAction.request.url else {
            decisionHandler(.cancel)
            return
        }

        let isLocal = ["127.0.0.1", "localhost"].contains(url.host ?? "") && url.port == port
        if isLocal || url.scheme == "about" {
            decisionHandler(.allow)
            return
        }

        if ["http", "https"].contains(url.scheme ?? "") {
            NSWorkspace.shared.open(url)
        }
        decisionHandler(.cancel)
    }

    func webView(
        _ webView: WKWebView,
        createWebViewWith configuration: WKWebViewConfiguration,
        for navigationAction: WKNavigationAction,
        windowFeatures: WKWindowFeatures
    ) -> WKWebView? {
        if let url = navigationAction.request.url {
            NSWorkspace.shared.open(url)
        }
        return nil
    }

    func webView(
        _ webView: WKWebView,
        runOpenPanelWith parameters: WKOpenPanelParameters,
        initiatedByFrame frame: WKFrameInfo,
        completionHandler: @escaping ([URL]?) -> Void
    ) {
        let panel = NSOpenPanel()
        panel.title = "Import G-code File"
        panel.prompt = "Import"
        panel.message = "Choose a .gcode, .gc, .ngc, or .nc file to parse locally."
        panel.canChooseFiles = true
        panel.canChooseDirectories = parameters.allowsDirectories
        panel.allowsMultipleSelection = parameters.allowsMultipleSelection

        panel.beginSheetModal(for: window) { response in
            completionHandler(response == .OK ? panel.urls : nil)
        }
    }
}
