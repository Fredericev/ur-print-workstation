using System.Text.Json;

namespace URPrintWorkstation.Windows;

internal enum WorkstationShutdownTarget
{
    None,
    Service,
    ProjectFallback,
}

// Keep service ownership independent of the Windows UI so it can be exercised
// on every build host before a portable native package is published.
internal static class WorkstationServiceIdentity
{
    internal static bool Matches(JsonElement health, string expectedRoot, string? expectedVersion)
    {
        if (
            health.ValueKind != JsonValueKind.Object
            || !health.TryGetProperty("project_root", out var projectRoot)
            || projectRoot.ValueKind != JsonValueKind.String
            || !health.TryGetProperty("version", out var version)
            || version.ValueKind != JsonValueKind.String
            || !health.TryGetProperty("demo", out var demo)
            || demo.ValueKind != JsonValueKind.False
        )
            return false;
        var reportedRoot = projectRoot.GetString();
        if (
            string.IsNullOrWhiteSpace(reportedRoot)
            || string.IsNullOrWhiteSpace(expectedVersion)
            || !string.Equals(version.GetString(), expectedVersion, StringComparison.Ordinal)
        )
            return false;
        try
        {
            var reported = Path.GetFullPath(reportedRoot)
                .TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar);
            var expected = Path.GetFullPath(expectedRoot)
                .TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar);
            return string.Equals(reported, expected, StringComparison.OrdinalIgnoreCase);
        }
        catch (Exception ex) when (ex is ArgumentException or NotSupportedException or PathTooLongException)
        {
            return false;
        }
    }

    internal static WorkstationShutdownTarget SelectShutdownTarget(
        bool connectedToPackage,
        bool startedBackend,
        JsonElement? health,
        string expectedRoot,
        string? expectedVersion
    )
    {
        if (!connectedToPackage && !startedBackend)
            return WorkstationShutdownTarget.None;
        if (health is null)
            return WorkstationShutdownTarget.ProjectFallback;
        return Matches(health.Value, expectedRoot, expectedVersion)
            ? WorkstationShutdownTarget.Service
            : WorkstationShutdownTarget.None;
    }
}
