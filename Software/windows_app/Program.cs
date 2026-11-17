using System.Diagnostics;
using System.Net;
using System.Net.Http.Json;
using System.Text.Json;
using Microsoft.Web.WebView2.Core;
using Microsoft.Web.WebView2.WinForms;

namespace URPrintWorkstation.Windows;

internal static class Program
{
    private const string SingleInstanceName = @"Local\URPrintWorkstation-6F8A5CB4-304E-45EE-8B62-5991959D297E";

    [STAThread]
    private static void Main()
    {
        ApplicationConfiguration.Initialize();
        using var mutex = new Mutex(true, SingleInstanceName, out var isFirstInstance);
        if (!isFirstInstance)
        {
            MessageBox.Show(
                "UR Print Workstation is already running.",
                "UR Print Workstation",
                MessageBoxButtons.OK,
                MessageBoxIcon.Information
            );
            return;
        }

        Application.Run(new WorkstationForm());
        GC.KeepAlive(mutex);
    }
}

internal sealed class WorkstationForm : Form
{
    private const int Port = 8876;
    private static readonly Uri BaseUri = new($"http://127.0.0.1:{Port}/");
    private static readonly Uri HealthUri = new($"http://127.0.0.1:{Port}/api/health");

    private readonly HttpClient _http = new() { Timeout = TimeSpan.FromSeconds(2) };
    private readonly CancellationTokenSource _shutdown = new();
    private readonly object _logLock = new();
    private readonly WebView2 _webView = new() { Dock = DockStyle.Fill };
    private readonly Panel _loadingPanel = new() { Dock = DockStyle.Fill, BackColor = Color.FromArgb(247, 249, 252) };
    private readonly Label _status = new()
    {
        AutoSize = false,
        TextAlign = ContentAlignment.MiddleCenter,
        Font = new Font("Segoe UI", 11F, FontStyle.Regular),
        ForeColor = Color.FromArgb(74, 85, 104),
        Width = 620,
        Height = 76,
    };
    private readonly ProgressBar _progress = new()
    {
        Style = ProgressBarStyle.Marquee,
        MarqueeAnimationSpeed = 24,
        Width = 320,
        Height = 8,
    };
    private readonly Button _retry = new()
    {
        Text = "Try Again",
        AutoSize = true,
        MinimumSize = new Size(118, 38),
        Visible = false,
    };
    private readonly Button _browser = new()
    {
        Text = "Open in Browser",
        AutoSize = true,
        MinimumSize = new Size(142, 38),
        Visible = false,
    };

    private Process? _backend;
    private bool _startedBackend;
    private bool _connectedToPackage;
    private bool _connecting;
    private bool _closing;
    private bool _shutdownComplete;

    private string InstallRoot => AppContext.BaseDirectory.TrimEnd(Path.DirectorySeparatorChar);
    private string PackagedProjectRoot => Path.Combine(InstallRoot, "project");
    private string ProjectRoot
    {
        get
        {
            var configured = Environment.GetEnvironmentVariable("UR_PRINT_PROJECT_ROOT");
            if (!string.IsNullOrWhiteSpace(configured))
                return Path.GetFullPath(configured);
            var marker = ReadPayloadMarker();
            var safeMarker = string.Concat(
                marker.Select(character =>
                    char.IsAsciiLetterOrDigit(character) || character is '-' or '_'
                        ? character
                        : '-'
                )
            );
            return Path.Combine(DataRoot, "projects", safeMarker);
        }
    }
    private string SoftwareRoot =>
        Environment.GetEnvironmentVariable("UR_PRINT_SOFTWARE_DIR")
        ?? Path.Combine(PackagedProjectRoot, "Software");
    private string DataRoot => Path.Combine(
        Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
        "UR Print Workstation"
    );
    private string LogPath => Path.Combine(DataRoot, "windows-launcher.log");
    private static string? PackageVersion => typeof(Program).Assembly.GetName().Version?.ToString(3);

    public WorkstationForm()
    {
        Text = "UR Print Workstation";
        StartPosition = FormStartPosition.CenterScreen;
        Width = 1487;
        Height = 980;
        MinimumSize = new Size(1024, 720);
        BackColor = Color.White;
        Icon = Icon.ExtractAssociatedIcon(Application.ExecutablePath);

        var menu = BuildMenu();
        MainMenuStrip = menu;
        Controls.Add(_webView);
        Controls.Add(_loadingPanel);
        Controls.Add(menu);

        BuildLoadingPanel();
        _retry.Click += async (_, _) => await ConnectAsync();
        _browser.Click += (_, _) => OpenBrowser();
        Shown += async (_, _) => await ConnectAsync();
        FormClosing += OnFormClosing;
    }

    private MenuStrip BuildMenu()
    {
        var menu = new MenuStrip();
        var file = new ToolStripMenuItem("&File");
        var openBrowser = new ToolStripMenuItem("Open in &Browser", null, (_, _) => OpenBrowser());
        var diagnostics = new ToolStripMenuItem(
            "Open &Diagnostics Folder",
            null,
            (_, _) => OpenFolder(DataRoot)
        );
        var exit = new ToolStripMenuItem("E&xit", null, (_, _) => Close());
        file.DropDownItems.AddRange([openBrowser, diagnostics, new ToolStripSeparator(), exit]);

        var view = new ToolStripMenuItem("&View");
        var reload = new ToolStripMenuItem(
            "&Reload",
            null,
            (_, _) =>
            {
                if (_webView.CoreWebView2 is not null)
                    _webView.Reload();
            }
        )
        {
            ShortcutKeys = Keys.Control | Keys.R,
        };
        view.DropDownItems.Add(reload);
        menu.Items.AddRange([file, view]);
        return menu;
    }

    private void BuildLoadingPanel()
    {
        var content = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 5,
            BackColor = _loadingPanel.BackColor,
        };
        content.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
        content.RowStyles.Add(new RowStyle(SizeType.Percent, 45F));
        content.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        content.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        content.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        content.RowStyles.Add(new RowStyle(SizeType.Percent, 55F));

        var title = new Label
        {
            Text = "UR Print Workstation",
            AutoSize = true,
            Anchor = AnchorStyles.None,
            Font = new Font("Segoe UI", 20F, FontStyle.Bold),
            ForeColor = Color.FromArgb(18, 24, 38),
            Margin = new Padding(0, 0, 0, 12),
        };
        _status.Anchor = AnchorStyles.None;
        _progress.Anchor = AnchorStyles.None;

        var buttons = new FlowLayoutPanel
        {
            AutoSize = true,
            Anchor = AnchorStyles.None,
            FlowDirection = FlowDirection.LeftToRight,
            WrapContents = false,
            Margin = new Padding(0, 12, 0, 0),
        };
        buttons.Controls.Add(_retry);
        buttons.Controls.Add(_browser);

        content.Controls.Add(new Panel(), 0, 0);
        content.Controls.Add(title, 0, 1);
        content.Controls.Add(_status, 0, 2);
        content.Controls.Add(_progress, 0, 3);
        content.Controls.Add(buttons, 0, 4);
        _loadingPanel.Controls.Add(content);
        _loadingPanel.BringToFront();
    }

    private async Task ConnectAsync()
    {
        if (_connecting || _closing)
            return;
        _connecting = true;
        ShowLoading("Starting the local control service…", retry: false, browser: false);
        try
        {
            Directory.CreateDirectory(DataRoot);
            EnsureWritableProject();
            if (!File.Exists(Path.Combine(SoftwareRoot, "control_station.py")))
                throw new FileNotFoundException(
                    "The packaged workstation backend is missing.",
                    Path.Combine(SoftwareRoot, "control_station.py")
                );

            var health = await GetHealthAsync(_shutdown.Token);
            if (health is not null)
            {
                if (!HealthBelongsToThisPackage(health.Value))
                    throw new InvalidOperationException(
                        $"Port {Port} is already used by a different workstation service. "
                        + "Close that service, then choose Try Again."
                    );
            }
            else
            {
                StartBackend();
                health = await WaitForHealthAsync(_shutdown.Token);
            }

            if (health is null)
                throw new InvalidOperationException(
                    "The local control service did not become ready. "
                    + $"Open the diagnostics folder and review {Path.GetFileName(LogPath)}."
                );

            // Validate again after startup: another process may have acquired
            // the port while this launcher's backend was starting.
            if (!HealthBelongsToThisPackage(health.Value))
                throw new InvalidOperationException(
                    $"Port {Port} is already used by a different workstation service. "
                    + "Close that service, then choose Try Again."
                );
            _connectedToPackage = true;

            await LoadWebViewAsync();
        }
        catch (OperationCanceledException) when (_closing)
        {
            // Normal application shutdown.
        }
        catch (Exception ex)
        {
            AppendLog($"Launcher error: {ex}");
            ShowLoading(ex.Message, retry: true, browser: await IsHealthyAsync());
        }
        finally
        {
            _connecting = false;
        }
    }

    private string ReadPayloadMarker()
    {
        var markerPath = Path.Combine(PackagedProjectRoot, "PAYLOAD_VERSION.txt");
        if (!File.Exists(markerPath))
            throw new FileNotFoundException(
                "The packaged project version marker is missing.",
                markerPath
            );
        var marker = File.ReadAllText(markerPath).Trim();
        if (string.IsNullOrWhiteSpace(marker) || marker.Length > 160)
            throw new InvalidDataException("The packaged project version marker is invalid.");
        return marker;
    }

    private void EnsureWritableProject()
    {
        if (!string.IsNullOrWhiteSpace(Environment.GetEnvironmentVariable("UR_PRINT_PROJECT_ROOT")))
            return;

        var source = PackagedProjectRoot;
        var destination = ProjectRoot;
        var expectedMarker = ReadPayloadMarker();
        var destinationMarker = Path.Combine(destination, "PAYLOAD_VERSION.txt");
        if (
            File.Exists(destinationMarker)
            && string.Equals(
                File.ReadAllText(destinationMarker).Trim(),
                expectedMarker,
                StringComparison.Ordinal
            )
        )
        {
            return;
        }

        var parent = Directory.GetParent(destination)?.FullName
            ?? throw new InvalidOperationException("The writable project path is invalid.");
        Directory.CreateDirectory(parent);
        var staging = destination + $".staging-{Environment.ProcessId}";
        if (Directory.Exists(staging))
            Directory.Delete(staging, recursive: true);
        Directory.CreateDirectory(staging);
        try
        {
            CopyProjectDirectory(source, staging);
            File.WriteAllText(Path.Combine(staging, "PAYLOAD_VERSION.txt"), expectedMarker + Environment.NewLine);
            if (Directory.Exists(destination))
                throw new InvalidOperationException(
                    "The writable project payload is incomplete. Close Docker Desktop, remove "
                    + $"\"{destination}\", then start the workstation again."
                );
            Directory.Move(staging, destination);
        }
        catch
        {
            if (Directory.Exists(staging))
                Directory.Delete(staging, recursive: true);
            throw;
        }
    }

    private static void CopyProjectDirectory(string sourceRoot, string destinationRoot)
    {
        foreach (
            var directory in Directory.EnumerateDirectories(
                sourceRoot,
                "*",
                SearchOption.AllDirectories
            )
        )
        {
            var relative = Path.GetRelativePath(sourceRoot, directory);
            if (
                relative.Equals("Software", StringComparison.OrdinalIgnoreCase)
                || relative.StartsWith($"Software{Path.DirectorySeparatorChar}", StringComparison.OrdinalIgnoreCase)
            )
            {
                continue;
            }
            Directory.CreateDirectory(Path.Combine(destinationRoot, relative));
        }
        foreach (
            var file in Directory.EnumerateFiles(
                sourceRoot,
                "*",
                SearchOption.AllDirectories
            )
        )
        {
            var relative = Path.GetRelativePath(sourceRoot, file);
            if (
                relative.Equals("PAYLOAD_VERSION.txt", StringComparison.OrdinalIgnoreCase)
                || relative.StartsWith($"Software{Path.DirectorySeparatorChar}", StringComparison.OrdinalIgnoreCase)
            )
            {
                continue;
            }
            var destination = Path.Combine(destinationRoot, relative);
            Directory.CreateDirectory(Path.GetDirectoryName(destination)!);
            File.Copy(file, destination, overwrite: true);
        }
    }

    private void StartBackend()
    {
        if (_backend is { HasExited: false })
            return;

        var python = Path.Combine(InstallRoot, "python", "python.exe");
        var script = Path.Combine(SoftwareRoot, "control_station.py");
        if (!File.Exists(python))
            throw new FileNotFoundException("The bundled Python runtime is missing.", python);

        var startInfo = new ProcessStartInfo
        {
            FileName = python,
            WorkingDirectory = SoftwareRoot,
            UseShellExecute = false,
            CreateNoWindow = true,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
        };
        startInfo.ArgumentList.Add(script);
        startInfo.ArgumentList.Add("--no-browser");
        startInfo.ArgumentList.Add("--port");
        startInfo.ArgumentList.Add(Port.ToString());
        startInfo.Environment["PYTHONUTF8"] = "1";
        startInfo.Environment["PYTHONDONTWRITEBYTECODE"] = "1";
        startInfo.Environment["UR_PRINT_DATA_DIR"] = DataRoot;
        startInfo.Environment["UR_PRINT_PROJECT_ROOT"] = ProjectRoot;
        startInfo.Environment["UR_PRINT_SOFTWARE_DIR"] = SoftwareRoot;

        _backend = new Process { StartInfo = startInfo, EnableRaisingEvents = true };
        _backend.OutputDataReceived += (_, eventArgs) =>
        {
            if (eventArgs.Data is not null)
                AppendLog(eventArgs.Data);
        };
        _backend.ErrorDataReceived += (_, eventArgs) =>
        {
            if (eventArgs.Data is not null)
                AppendLog(eventArgs.Data);
        };
        _backend.Exited += (_, _) =>
        {
            if (_closing)
                return;
            BeginInvoke(() =>
            {
                ShowLoading(
                    "The local control service stopped unexpectedly. Review the diagnostics log, then try again.",
                    retry: true,
                    browser: false
                );
            });
        };

        if (!_backend.Start())
            throw new InvalidOperationException("Windows could not start the bundled backend.");
        _backend.BeginOutputReadLine();
        _backend.BeginErrorReadLine();
        _startedBackend = true;
        AppendLog($"Started backend PID {_backend.Id} with {python}");
    }

    private async Task<JsonElement?> WaitForHealthAsync(CancellationToken cancellationToken)
    {
        for (var attempt = 0; attempt < 600; attempt++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            if (_backend is { HasExited: true })
                throw new InvalidOperationException(
                    $"The local control service exited with code {_backend.ExitCode}. "
                    + "Open the diagnostics folder for details."
                );
            var health = await GetHealthAsync(cancellationToken);
            if (health is not null)
                return health;
            await Task.Delay(200, cancellationToken);
        }
        return null;
    }

    private async Task<JsonElement?> GetHealthAsync(CancellationToken cancellationToken)
    {
        try
        {
            using var response = await _http.GetAsync(HealthUri, cancellationToken);
            if (response.StatusCode != HttpStatusCode.OK)
                return null;
            await using var stream = await response.Content.ReadAsStreamAsync(cancellationToken);
            using var document = await JsonDocument.ParseAsync(stream, cancellationToken: cancellationToken);
            return document.RootElement.Clone();
        }
        catch (HttpRequestException)
        {
            return null;
        }
        catch (JsonException)
        {
            return null;
        }
        catch (TaskCanceledException) when (!cancellationToken.IsCancellationRequested)
        {
            return null;
        }
    }

    private async Task<bool> IsHealthyAsync()
    {
        var health = await GetHealthAsync(_shutdown.Token);
        return health is not null && HealthBelongsToThisPackage(health.Value);
    }

    private bool HealthBelongsToThisPackage(JsonElement health) =>
        WorkstationServiceIdentity.Matches(health, ProjectRoot, PackageVersion);

    private async Task LoadWebViewAsync()
    {
        ShowLoading("Loading workstation…", retry: false, browser: false);
        try
        {
            var webViewData = Path.Combine(DataRoot, "WebView2");
            var environment = await CoreWebView2Environment.CreateAsync(
                browserExecutableFolder: null,
                userDataFolder: webViewData
            );
            await _webView.EnsureCoreWebView2Async(environment);
            ConfigureWebView(_webView.CoreWebView2);
            _webView.Source = BaseUri;
        }
        catch (WebView2RuntimeNotFoundException ex)
        {
            AppendLog($"WebView2 Runtime unavailable: {ex}");
            OpenBrowser();
            ShowLoading(
                "The embedded Windows web runtime is unavailable, so the workstation opened in your default browser. "
                + "Keep this window open while using it.",
                retry: false,
                browser: true
            );
        }
    }

    private void ConfigureWebView(CoreWebView2 core)
    {
        core.Settings.AreDefaultContextMenusEnabled = false;
        core.Settings.AreDevToolsEnabled = false;
        core.Settings.IsStatusBarEnabled = false;
        core.Settings.IsZoomControlEnabled = true;
        core.NewWindowRequested += (_, eventArgs) =>
        {
            eventArgs.Handled = true;
            if (Uri.TryCreate(eventArgs.Uri, UriKind.Absolute, out var uri))
                OpenExternalUri(uri);
        };
        core.NavigationStarting += (_, eventArgs) =>
        {
            if (!Uri.TryCreate(eventArgs.Uri, UriKind.Absolute, out var uri))
            {
                eventArgs.Cancel = true;
                return;
            }
            if (IsLocalUri(uri))
                return;
            eventArgs.Cancel = true;
            OpenExternalUri(uri);
        };
        core.NavigationCompleted += (_, eventArgs) =>
        {
            if (eventArgs.IsSuccess)
                _loadingPanel.Visible = false;
            else
                ShowLoading(
                    $"The workstation page could not load ({eventArgs.WebErrorStatus}).",
                    retry: true,
                    browser: true
                );
        };
        core.ProcessFailed += (_, eventArgs) =>
        {
            AppendLog($"WebView process failure: {eventArgs.ProcessFailedKind}");
            ShowLoading(
                "The embedded workstation view stopped. Reload it or continue in your browser.",
                retry: true,
                browser: true
            );
        };
    }

    private static bool IsLocalUri(Uri uri) =>
        uri.Port == Port
        && (uri.Host.Equals("127.0.0.1", StringComparison.OrdinalIgnoreCase)
            || uri.Host.Equals("localhost", StringComparison.OrdinalIgnoreCase));

    private void OpenBrowser() => OpenExternalUri(BaseUri);

    private static void OpenExternalUri(Uri uri)
    {
        if (uri.Scheme is not ("http" or "https"))
            return;
        Process.Start(new ProcessStartInfo(uri.AbsoluteUri) { UseShellExecute = true });
    }

    private static void OpenFolder(string path)
    {
        Directory.CreateDirectory(path);
        Process.Start(new ProcessStartInfo("explorer.exe", $"\"{path}\"") { UseShellExecute = true });
    }

    private void ShowLoading(string message, bool retry, bool browser)
    {
        if (InvokeRequired)
        {
            BeginInvoke(() => ShowLoading(message, retry, browser));
            return;
        }
        _status.Text = message;
        _retry.Visible = retry;
        _browser.Visible = browser;
        _progress.Visible = !retry && !browser;
        _loadingPanel.Visible = true;
        _loadingPanel.BringToFront();
    }

    private void AppendLog(string line)
    {
        try
        {
            lock (_logLock)
            {
                Directory.CreateDirectory(DataRoot);
                File.AppendAllText(
                    LogPath,
                    $"{DateTimeOffset.Now:O} {line}{Environment.NewLine}"
                );
            }
        }
        catch
        {
            // Logging must never crash the operator interface.
        }
    }

    private async Task StopProjectRuntimeFallbackAsync()
    {
        var compose = Path.Combine(ProjectRoot, "docker-compose.real.yml");
        if (!File.Exists(compose))
            return;
        var startInfo = new ProcessStartInfo("docker")
        {
            WorkingDirectory = ProjectRoot,
            UseShellExecute = false,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            CreateNoWindow = true,
        };
        foreach (var argument in new[] { "compose", "-f", compose, "stop", "ros2" })
            startInfo.ArgumentList.Add(argument);
        try
        {
            using var process = new Process { StartInfo = startInfo };
            if (!process.Start())
                return;
            using var timeout = new CancellationTokenSource(TimeSpan.FromSeconds(90));
            await process.WaitForExitAsync(timeout.Token);
            var output = await process.StandardOutput.ReadToEndAsync();
            var error = await process.StandardError.ReadToEndAsync();
            if (!string.IsNullOrWhiteSpace(output))
                AppendLog(output.Trim());
            if (!string.IsNullOrWhiteSpace(error))
                AppendLog(error.Trim());
        }
        catch (Exception ex)
        {
            AppendLog($"Project stop fallback failed: {ex.Message}");
        }
    }

    private async void OnFormClosing(object? sender, FormClosingEventArgs eventArgs)
    {
        if (!_shutdownComplete)
        {
            eventArgs.Cancel = true;
            if (_closing)
                return;
            _closing = true;
            ShowLoading(
                "Stopping this workstation's ROS and local runtime…",
                retry: false,
                browser: false
            );
            try
            {
                // A rejected old/demo/foreign service must never receive this
                // launcher's shutdown command merely because it uses our port.
                if (!_connectedToPackage && !_startedBackend)
                {
                    _shutdownComplete = true;
                    BeginInvoke(Close);
                    return;
                }
                var health = await GetHealthAsync(CancellationToken.None);
                var shutdownTarget = WorkstationServiceIdentity.SelectShutdownTarget(
                    _connectedToPackage,
                    _startedBackend,
                    health,
                    ProjectRoot,
                    PackageVersion
                );
                if (shutdownTarget == WorkstationShutdownTarget.None)
                {
                    AppendLog("Skipped shutdown: the port belongs to another workstation service.");
                    _shutdownComplete = true;
                    BeginInvoke(Close);
                    return;
                }
                if (shutdownTarget == WorkstationShutdownTarget.ProjectFallback)
                    throw new InvalidOperationException("The owned backend health endpoint is unavailable.");
                using var shutdownHttp = new HttpClient
                {
                    Timeout = TimeSpan.FromSeconds(150),
                };
                using var content = JsonContent.Create(new { reason = "application-exit" });
                using var response = await shutdownHttp.PostAsync(
                    new Uri(BaseUri, "/api/runtime/shutdown"),
                    content
                );
                response.EnsureSuccessStatusCode();
            }
            catch (Exception ex)
            {
                AppendLog($"Lifecycle shutdown endpoint failed: {ex.Message}");
                await StopProjectRuntimeFallbackAsync();
            }
            _shutdownComplete = true;
            BeginInvoke(Close);
            return;
        }

        _closing = true;
        _shutdown.Cancel();
        _http.Dispose();
        if (!_startedBackend || _backend is null)
            return;
        try
        {
            if (!_backend.HasExited)
                _backend.Kill(entireProcessTree: true);
            _backend.WaitForExit(4000);
        }
        catch (Exception ex)
        {
            AppendLog($"Could not stop backend cleanly: {ex.Message}");
        }
        finally
        {
            _backend.Dispose();
        }
    }
}
