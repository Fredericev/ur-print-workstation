#requires -Version 5.1
[CmdletBinding()]
param(
    [string]$DotnetVersion = "10.0.302",
    [string]$PythonVersion = "3.13.14"
)

$ErrorActionPreference = "Stop"
$SoftwareDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $SoftwareDir
$WindowsApp = Join-Path $SoftwareDir "windows_app"
$BuildRoot = Join-Path $SoftwareDir "runtime\windows-build"
$CacheDir = Join-Path $BuildRoot "cache"
$PublishRoot = Join-Path $BuildRoot "publish"
$StageRoot = Join-Path $BuildRoot "stage"
$PayloadSnapshot = Join-Path $BuildRoot "project-payload"
$DistRoot = Join-Path $SoftwareDir "Windows"
$ProjectFile = Join-Path $WindowsApp "URPrintWorkstation.Windows.csproj"
$BackendSource = Get-Content (Join-Path $SoftwareDir "control_station.py") -Raw
$VersionMatch = [regex]::Match($BackendSource, '(?m)^VERSION = "(\d+\.\d+\.\d+)"$')
if (-not $VersionMatch.Success) {
    throw "Could not determine the backend version before packaging."
}
$AppVersion = $VersionMatch.Groups[1].Value
$LauncherProject = [xml](Get-Content $ProjectFile -Raw)
if (
    [string]$LauncherProject.Project.PropertyGroup.Version -ne $AppVersion -or
    [string]$LauncherProject.Project.PropertyGroup.FileVersion -ne "$AppVersion.0" -or
    [string]$LauncherProject.Project.PropertyGroup.AssemblyVersion -ne "$AppVersion.0"
) {
    throw "Backend and Windows launcher versions do not match; update the launcher before packaging."
}
$PackageGuide = Get-Content (Join-Path $WindowsApp "README-WINDOWS.txt") -Raw
if (
    -not $PackageGuide.Contains("UR-Print-Workstation-$AppVersion-win-x64.zip") -or
    -not $PackageGuide.Contains("UR-Print-Workstation-$AppVersion-win-arm64.zip")
) {
    throw "The Windows package guide version does not match the backend."
}
$PythonChecksums = @{
    "amd64" = "2dcbfaca0c0742b6201209e5dbfc129ef5602e44544d6313b09eabc3931cdf65"
    "arm64" = "ba81a3db4de19a305bfdf67caaefa088db4241a51de7d120b1aa95f8d8cf2a7d"
}

New-Item -ItemType Directory -Force -Path $BuildRoot, $CacheDir, $PublishRoot, $StageRoot, $DistRoot | Out-Null

$DotnetRoot = Join-Path $BuildRoot "dotnet"
$Dotnet = Join-Path $DotnetRoot "dotnet.exe"
$env:DOTNET_ROOT = $DotnetRoot
$env:DOTNET_MULTILEVEL_LOOKUP = "0"

function Test-PinnedDotnetSdk {
    if (-not (Test-Path $Dotnet)) { return $false }
    try {
        $ReportedVersion = & $Dotnet --version 2>$null | Select-Object -Last 1
        return $LASTEXITCODE -eq 0 -and $ReportedVersion -eq $DotnetVersion
    } catch {
        return $false
    }
}

if (-not (Test-PinnedDotnetSdk)) {
    $InstallScript = Join-Path $CacheDir "dotnet-install.ps1"
    $HostArchitecture = [Environment]::GetEnvironmentVariable("PROCESSOR_ARCHITEW6432")
    if (-not $HostArchitecture) {
        $HostArchitecture = [Environment]::GetEnvironmentVariable("PROCESSOR_ARCHITECTURE")
    }
    switch ($HostArchitecture.ToUpperInvariant()) {
        "AMD64" { $SdkArchitecture = "x64" }
        "ARM64" { $SdkArchitecture = "arm64" }
        default { throw "A 64-bit x64 or ARM64 Windows build computer is required." }
    }
    Invoke-WebRequest "https://dot.net/v1/dotnet-install.ps1" -UseBasicParsing -OutFile $InstallScript
    & $InstallScript -Version $DotnetVersion -Architecture $SdkArchitecture -InstallDir $DotnetRoot -NoPath
    if (-not (Test-PinnedDotnetSdk)) {
        throw "Pinned .NET SDK $DotnetVersion is unavailable at $Dotnet."
    }
}

function Get-PayloadRelativePath {
    param([string]$Root, [string]$Path)
    # The newer .NET relative-path API is unavailable in Windows PowerShell 5.1.
    $RootPrefix = [IO.Path]::GetFullPath($Root).TrimEnd([char[]]@('\', '/')) + [IO.Path]::DirectorySeparatorChar
    $FullPath = [IO.Path]::GetFullPath($Path)
    if (-not $FullPath.StartsWith($RootPrefix, [StringComparison]::OrdinalIgnoreCase)) {
        throw "A file outside the package payload cannot be included: $Path"
    }
    return $FullPath.Substring($RootPrefix.Length).Replace("\", "/")
}

function Get-CanonicalHashLines {
    param([string]$Root, [string]$ExcludedRelativePath)
    $RelativeFiles = [System.Collections.Generic.List[string]]::new()
    Get-ChildItem -LiteralPath $Root -File -Recurse | ForEach-Object {
        $Relative = Get-PayloadRelativePath $Root $_.FullName
        if ($Relative -ne $ExcludedRelativePath) { $RelativeFiles.Add($Relative) }
    }
    $RelativeFiles.Sort([StringComparer]::Ordinal)
    foreach ($Relative in $RelativeFiles) {
        $Hash = (Get-FileHash -LiteralPath (Join-Path $Root $Relative) -Algorithm SHA256).Hash.ToLowerInvariant()
        "$Hash  $Relative"
    }
}

function Get-PythonArchive {
    param([string]$Architecture)
    $Archive = Join-Path $CacheDir "python-$PythonVersion-embeddable-$Architecture.zip"
    $Url = "https://www.python.org/ftp/python/$PythonVersion/python-$PythonVersion-embeddable-$Architecture.zip"
    $Expected = $PythonChecksums[$Architecture]
    if ((-not (Test-Path $Archive)) -or ((Get-FileHash $Archive -Algorithm SHA256).Hash.ToLowerInvariant() -ne $Expected)) {
        Remove-Item $Archive -Force -ErrorAction SilentlyContinue
        Invoke-WebRequest $Url -OutFile $Archive
    }
    $Actual = (Get-FileHash $Archive -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($Actual -ne $Expected) {
        throw "Python archive checksum mismatch for $Architecture. Expected $Expected; got $Actual."
    }
    return $Archive
}

function Copy-DirectoryClean {
    param([string]$Source, [string]$Destination)
    Remove-Item $Destination -Recurse -Force -ErrorAction SilentlyContinue
    New-Item -ItemType Directory -Force -Path $Destination | Out-Null
    # Match the Mac snapshot exclusions, including nested vendor Git data.
    Get-ChildItem -LiteralPath $Source -Force |
        Where-Object {
            $_.Name -notin @(".git", ".github", ".DS_Store", "__pycache__", ".pytest_cache", "verify_team_release.py") -and
            $_.Extension -ne ".pyc"
        } | ForEach-Object {
            $Target = Join-Path $Destination $_.Name
            if ($_.PSIsContainer) {
                Copy-DirectoryClean $_.FullName $Target
            } else {
                Copy-Item -LiteralPath $_.FullName -Destination $Target -Force
            }
        }
}

function Copy-ProjectPayload {
    param([string]$Destination)
    $SoftwarePayload = Join-Path $Destination "Software"
    $WorkspacePayload = Join-Path $Destination "workspaces\ur_ros2_ws"
    New-Item -ItemType Directory -Force -Path $SoftwarePayload, $WorkspacePayload | Out-Null

    Copy-Item (Join-Path $ProjectRoot "docker-compose.real.yml") $Destination
    Copy-DirectoryClean (Join-Path $ProjectRoot "docker") (Join-Path $Destination "docker")
    Copy-DirectoryClean (Join-Path $ProjectRoot "scripts") (Join-Path $Destination "scripts")
    Copy-DirectoryClean (Join-Path $ProjectRoot "real_robot_files") (Join-Path $Destination "real_robot_files")
    Copy-DirectoryClean (Join-Path $ProjectRoot "workspaces\ur_ros2_ws\src") (Join-Path $WorkspacePayload "src")
    Copy-DirectoryClean (Join-Path $ProjectRoot "workspaces\ur_ros2_ws\config") (Join-Path $WorkspacePayload "config")
    Copy-DirectoryClean (Join-Path $ProjectRoot "workspaces\ur_ros2_ws\scripts") (Join-Path $WorkspacePayload "scripts")

    # Include future shared modules automatically, as the Mac package does.
    Get-ChildItem -LiteralPath $SoftwareDir -Filter "*.py" -File |
        ForEach-Object { Copy-Item -LiteralPath $_.FullName -Destination $SoftwarePayload }
    Copy-Item (Join-Path $SoftwareDir "README.md") $SoftwarePayload
    Copy-DirectoryClean (Join-Path $SoftwareDir "app") (Join-Path $SoftwarePayload "app")

    # Every source, UI, and ROS payload change needs a fresh writable project.
    $HashLines = Get-CanonicalHashLines $Destination "PAYLOAD_VERSION.txt"
    $HashText = ($HashLines -join "`n") + "`n"
    $Sha256 = [Security.Cryptography.SHA256]::Create()
    try {
        $PayloadHash = [BitConverter]::ToString(
            $Sha256.ComputeHash([Text.Encoding]::UTF8.GetBytes($HashText))
        ).Replace("-", "").ToLowerInvariant().Substring(0, 20)
    } finally {
        $Sha256.Dispose()
    }
    [IO.File]::WriteAllText((Join-Path $Destination "PAYLOAD_VERSION.txt"), "$AppVersion-$PayloadHash`n", [Text.Encoding]::ASCII)
}

function Build-Architecture {
    param(
        [string]$RuntimeIdentifier,
        [string]$PythonArchitecture
    )
    $PythonArchive = Get-PythonArchive $PythonArchitecture
    $PublishDir = Join-Path $PublishRoot $RuntimeIdentifier
    $PackageName = "UR-Print-Workstation-$AppVersion-$RuntimeIdentifier"
    $StageDir = Join-Path $StageRoot $PackageName
    Remove-Item $PublishDir, $StageDir -Recurse -Force -ErrorAction SilentlyContinue
    New-Item -ItemType Directory -Force -Path $PublishDir, (Join-Path $StageDir "python"), (Join-Path $StageDir "project") | Out-Null

    & $Dotnet publish $ProjectFile --configuration Release --runtime $RuntimeIdentifier --self-contained true --output $PublishDir -p:EnableWindowsTargeting=true -p:PublishSingleFile=false -p:DebugType=None -p:DebugSymbols=false
    if ($LASTEXITCODE -ne 0) { throw "dotnet publish failed for $RuntimeIdentifier." }

    Copy-Item (Join-Path $PublishDir "*") $StageDir -Recurse -Force
    Get-ChildItem $StageDir -Filter "*.pdb" -Recurse | Remove-Item -Force
    Expand-Archive $PythonArchive (Join-Path $StageDir "python") -Force
    "python313.zip`n.`n../project/Software`n" | Set-Content (Join-Path $StageDir "python\python313._pth") -Encoding Ascii
    Copy-DirectoryClean $PayloadSnapshot (Join-Path $StageDir "project")
    Copy-Item (Join-Path $WindowsApp "README-WINDOWS.txt"), (Join-Path $WindowsApp "THIRD-PARTY-NOTICES.txt"), (Join-Path $WindowsApp "Replay Offline Print.cmd") $StageDir

    $Manifest = Join-Path $StageDir "SHA256SUMS.txt"
    # Canonical lowercase SHA-256, POSIX paths, ordinal ordering, and LF
    # produce the same manifest records as the macOS cross-builder.
    $ManifestText = ((Get-CanonicalHashLines $StageDir "SHA256SUMS.txt") -join "`n") + "`n"
    [IO.File]::WriteAllText($Manifest, $ManifestText, [Text.Encoding]::ASCII)

    $Archive = Join-Path $DistRoot "$PackageName.zip"
    Remove-Item $Archive, "$Archive.sha256" -Force -ErrorAction SilentlyContinue
    Compress-Archive $StageDir $Archive -CompressionLevel Optimal
    $ArchiveHash = (Get-FileHash $Archive -Algorithm SHA256).Hash.ToLowerInvariant()
    [IO.File]::WriteAllText("$Archive.sha256", "$ArchiveHash  $([IO.Path]::GetFileName($Archive))`n", [Text.Encoding]::ASCII)
    Write-Host "Created $Archive"
}

# Published version names are immutable, like the Mac portable package.
foreach ($RuntimeIdentifier in @("win-x64", "win-arm64")) {
    $Archive = Join-Path $DistRoot "UR-Print-Workstation-$AppVersion-$RuntimeIdentifier.zip"
    if ((Test-Path $Archive) -or (Test-Path "$Archive.sha256")) {
        throw "Release $Archive already exists; use a new version for a new release."
    }
}

# Capture the project exactly once. Both architecture packages must contain the
# same application sources even if a developer edits the working tree while a
# long cross-platform build is running.
Remove-Item $PayloadSnapshot -Recurse -Force -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force -Path $PayloadSnapshot | Out-Null
Copy-ProjectPayload $PayloadSnapshot

Build-Architecture "win-x64" "amd64"
Build-Architecture "win-arm64" "arm64"
Copy-Item (Join-Path $WindowsApp "README-WINDOWS.txt") (Join-Path $DistRoot "README-WINDOWS.txt") -Force
Write-Host "Windows packages are ready in $DistRoot"
