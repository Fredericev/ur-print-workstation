using System.Text.Json;
using URPrintWorkstation.Windows;

var project = Path.Combine(Path.GetTempPath(), "ur-native-contract", "current");
const string version = "0.14.0";
JsonElement Health(string? root = null, string release = version, bool demo = false) =>
    JsonSerializer.SerializeToElement(new { project_root = root ?? project, version = release, demo });
void Check(bool condition, string scenario)
{
    if (!condition)
        throw new Exception($"Windows service ownership regression: {scenario}");
}
bool Matches(JsonElement health) => WorkstationServiceIdentity.Matches(health, project, version);
WorkstationShutdownTarget Target(bool connected, bool started, JsonElement? health) =>
    WorkstationServiceIdentity.SelectShutdownTarget(connected, started, health, project, version);

Check(Matches(Health()), "current live backend should be usable");
Check(Matches(Health(project + Path.DirectorySeparatorChar)), "trailing directory separator should be harmless");
Check(!Matches(Health(release: "0.13.0")), "an old backend from the same project must not replace current functionality");
Check(!Matches(Health(demo: true)), "demo mode must not masquerade as the real workstation");
Check(!Matches(Health(Path.Combine(Path.GetTempPath(), "other-project"))), "another project must not be reused");
Check(!Matches(Health("\0")), "a malformed project root should be rejected without throwing");
foreach (var malformed in new[] { "{}", "null", "[]", "{\"project_root\":42}", "{\"version\":\"0.14.0\",\"demo\":false}", "{\"project_root\":\"\",\"version\":\"0.14.0\",\"demo\":false}" })
{
    using var document = JsonDocument.Parse(malformed);
    Check(!Matches(document.RootElement), $"malformed service identity {malformed}");
}
Check(Target(false, false, Health()) == WorkstationShutdownTarget.None, "an unconnected launcher must not stop a service");
Check(Target(true, false, Health()) == WorkstationShutdownTarget.Service, "a verified reused backend should stop on application exit");
Check(Target(false, true, Health()) == WorkstationShutdownTarget.Service, "a started verified backend should stop on application exit");
Check(Target(true, true, Health(release: "0.13.0")) == WorkstationShutdownTarget.None, "a foreign replacement on the port must never receive shutdown");
Check(Target(true, false, Health(demo: true)) == WorkstationShutdownTarget.None, "a demo replacement on the port must never receive shutdown");
Check(Target(false, false, null) == WorkstationShutdownTarget.None, "startup failure without an owned backend requires no cleanup");
Check(Target(false, true, null) == WorkstationShutdownTarget.ProjectFallback, "an unavailable owned backend requires local project cleanup, never an unverified HTTP POST");
Console.WriteLine("Windows native service identity and shutdown ownership contracts passed.");
