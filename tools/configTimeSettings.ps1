# configTimeSettings.ps1 v1.1 for ESP32 Time Server
# Copyright Rob Latour, 2026
# License: MIT
# Website: https://github.com/roblatour/ESP32TimeServer

$ErrorActionPreference = 'Stop'
$script:Version = '1.1'
$script:LegacyBackupVersion = '1.0'
$script:RegistryRoot = 'SYSTEM\CurrentControlSet\Services\W32Time'
$script:OwnedRuleNames = @(
    'ESP32TimeServer-configTimeSettings-PTP-In',
    'ESP32TimeServer-configTimeSettings-PTP-Out'
)
$script:PtpMulticastAddress = '224.0.1.129'
$script:PtpOperationalLog = 'Microsoft-Windows-Time-Service-PTP-Provider/PTP-Operational'
$script:ExpectedNtpPeers = @()
$script:ExpectedPtpMaster = $null
$script:PtpCandidate = $null
$script:KnownValues = [ordered]@{
    'Parameters\Type' = $null
    'Parameters\NtpServer' = $null
    'TimeProviders\NtpClient\Enabled' = $null
    'TimeProviders\NtpClient\InputProvider' = $null
    'TimeProviders\VMICTimeProvider\Enabled' = $null
    'TimeProviders\PtpClient\PtpMasters' = $null
    'TimeProviders\PtpClient\DllName' = $null
    'TimeProviders\PtpClient\Enabled' = $null
    'TimeProviders\PtpClient\InputProvider' = $null
    'TimeProviders\PtpClient\DelayPollInterval' = $null
    'TimeProviders\PtpClient\AnnounceInterval' = $null
    'TimeProviders\PtpClient\EnableMulticastRx' = $null
    'TimeProviders\PtpClient\EnableMulticastTx' = $null
    'Config\MinPollInterval' = $null
    'Config\MaxPollInterval' = $null
    'Config\UpdateInterval' = $null
}
$script:ClockDisciplineKeys = @('Config\MinPollInterval','Config\MaxPollInterval','Config\UpdateInterval')
# PTP follows Microsoft's high-accuracy profile; NTP uses the documented standalone-computer defaults.
$script:ClockDisciplineProfiles = @{
    PTP = [ordered]@{ MinPollInterval = 6; MaxPollInterval = 6; UpdateInterval = 100 }
    NTP = [ordered]@{ MinPollInterval = 10; MaxPollInterval = 15; UpdateInterval = 360000 }
}

function Get-ClockDisciplineStates([string]$Mode) {
    $states = @()
    foreach ($entry in $script:ClockDisciplineProfiles[$Mode].GetEnumerator()) {
        $states += [pscustomobject]@{ path = 'Config'; name = $entry.Key; present = $true; kind = 'DWord'; value = $entry.Value }
    }
    return $states
}

function Get-ClockDisciplineDescription([string]$Mode) {
    $settings = $script:ClockDisciplineProfiles[$Mode]
    return ('MinPollInterval={0} ({1}s), MaxPollInterval={2} ({3}s), UpdateInterval={4} ({5}s)' -f
        $settings.MinPollInterval, [Math]::Pow(2, $settings.MinPollInterval),
        $settings.MaxPollInterval, [Math]::Pow(2, $settings.MaxPollInterval),
        $settings.UpdateInterval, ($settings.UpdateInterval / 100))
}

function Test-FirewallRulesMatch($ExpectedRules) {
    $actualRules = @(Get-OwnedFirewallState)
    if ($actualRules.Count -ne $ExpectedRules.Count) { return $false }
    foreach ($expected in $ExpectedRules) {
        $actual = @($actualRules | Where-Object { $_.name -ceq $expected.name })
        if ($actual.Count -ne 1) { return $false }
        foreach ($field in @('displayName','description','group','direction','action','enabled',
            'profile','protocol','application','service')) {
            if ([string]$actual[0][$field] -cne [string]$expected.$field) { return $false }
        }
        foreach ($field in @('localPort','remotePort','localAddress','remoteAddress','interfaceAlias')) {
            $left = @($actual[0][$field] | ForEach-Object { [string]$_ } | Sort-Object)
            $right = @($expected.$field | ForEach-Object { [string]$_ } | Sort-Object)
            if (($left -join ',') -cne ($right -join ',')) { return $false }
        }
    }
    return $true
}

function Show-Help {
    @'
Usage:
  .\tools\configTimeSettings.ps1 -backup
  .\tools\configTimeSettings.ps1 -restore <filename>
  .\tools\configTimeSettings.ps1 -inventory <IPv4-server-address>
  .\tools\configTimeSettings.ps1 -readiness <IPv4-server-address>
  .\tools\configTimeSettings.ps1 -set NTP -serveraddress1 <IPv4_or_IPv6_address>
      [-serveraddress2 <IPv4_or_IPv6_address>] [-serveraddress3 <IPv4_or_IPv6_address>]
      [-serveraddress4 <IPv4_or_IPv6_address>]
  .\tools\configTimeSettings.ps1 -set PTP <IPv4-server-address>
  .\tools\configTimeSettings.ps1 -help
  .\tools\configTimeSettings.ps1 -?

All commands configure this local computer. Run -backup, -restore, and -set
from an elevated 64-bit Windows PowerShell 5.1 session. Inventory, readiness,
and help do not require elevation. This version supports standalone, non-domain physical
computers only. Managed time policy, domain membership, domain controllers,
and competing clock controllers are safety refusals.
This script is unable to work on a Virtual Machine. Detection runs before any
operation, including help.

Backup files use configtimebackup_YYYY_MM_DD_HH_MM_SS.bak in the current
filesystem directory. Restore accepts any filename with a .bak extension
(case-insensitive). Restore only replays allowlisted settings and owned
firewall rules; it is not a system image. Backup and both -set modes require
read/list/write/delete access to that directory. Every set takes a validated
automatic safety backup and aborts if it cannot be completed. Preserve backups:
they can contain network details.

Native W32Time NTP always uses UDP port 123; no port argument is accepted.
Inventory, readiness, and PTP require a literal IPv4 unicast server address.
NTP accepts literal IPv4 or IPv6 unicast peers, without brackets or ports;
DNS names are not accepted. IPv6 link-local peers require a local scope, e.g.
fe80::1234%23. Adapter names are also accepted as scopes and converted to
numeric interface IDs for probes and W32Time. Mixed IPv4/IPv6 NTP lists are
supported; W32Time peer flags follow each address after a comma.
PTP uses UDP ports 319 and 320. Inventory queries Windows/driver timestamp
capabilities through IP Helper, reports Unknown when a query cannot establish
support, and does not infer support from a model name. It sends three 1000 ms
ICMP probes using each adapter's source IPv4 address only when Windows route
lookup selects that adapter and weak-host sending is disabled. Results establish
reachability only; they do not establish PTP/NTP availability or
synchronization. Only up, physical wired Ethernet NICs can be recommended;
Wi-Fi and virtual adapters are excluded even if probes succeed. Recommendations
require three successful probes, with hardware timestamp support preferred.
The recommendation line is green when a NIC is selected, yellow otherwise.
Active hardware timestamping is queried before selection. Failed queries remain
Unknown with the reason shown; a driver Enabled setting is not proof of active
timestamping. Inventory does not change driver settings.

PTP readiness: Windows PTP listens on every adapter and chooses one itself; it
cannot be told which adapter to use. Hardware timestamping is not required:
Windows falls back to software timestamps. -readiness (also shown at the end of
-inventory, and enforced by -set PTP) lists:
  [PTP adapter - expected] (green): the up, physical wired Ethernet adapter on
      the server's IPv4 subnet that Windows is expected to choose (hardware
      timestamp support Yes first, then the adapter Windows routes the server
      through, then the lowest interface index).
  [PTP adapter - alternate] (yellow): other up, physical wired Ethernet
      adapters on the server's subnet. Windows may choose one of these.
  [Other path] (yellow): Wi-Fi, virtual, or off-subnet adapters that Windows
      routes the server or PTP multicast 224.0.1.129 through.
Alternate adapters and other paths are warnings, not blockers; this script
never disables adapters. Ready requires at least one [PTP adapter]. Ping and
route lookups are reported but do not block PTP. Leaving only one adapter on
the server's network enabled gives the most predictable result.

-set PTP proceeds only when readiness passes. It also requires a Private network
profile on the expected adapter, the Windows PTP provider (ptpprov.dll),
firewall policy that does not block UDP 319/320, and your acknowledgement of
the ESP32 prerequisites (interactive Y; Enter, No, or unavailable input aborts
without changes). It configures the PtpClient provider, disables NtpClient (its
peer list is kept), and creates two script-owned firewall rules scoped to every
[PTP adapter], the server, UDP 319/320, the Private profile, and W32Time. After
the restart it reads PTP-Operational event 513 to report the adapter Windows
chose; a different adapter is a warning only.

Native Windows PTP does not verify this project's authentication TLV and has
no automatic NTP fallback. The PTP source must provide unsigned PTPv2,
default profile, domain 0, two-step E2E UDP/IPv4, multicast Delay_Req support,
and valid UTC metadata. NTP is unsigned; the server must permit it. Ping
confirms reachability only, not time protocol operation or synchronization.
Normal W32Time synchronization can step or slew the system clock. No wall-clock
time, time zone, step/correction limits, or unrelated networking settings are
changed.

Each set mode applies its W32Time clock-discipline profile in Config:
  PTP: MinPollInterval=6, MaxPollInterval=6 (64 s), UpdateInterval=100 (1 s),
       following Microsoft's high-accuracy guidance so PTP offsets converge
       in minutes rather than hours.
  NTP: MinPollInterval=10, MaxPollInterval=15, UpdateInterval=360000, the
       Windows standalone-computer defaults.
These values are backed up and restored. Backups from version 1.0 do not
contain them; restoring one leaves them unchanged.

After a successful set, the script asks "Restart the Windows Time service now?
(Y/n)". Enter means Yes. No, EOF, or unavailable input defers activation and
synchronization verification; it does not start, stop, restart, reload, or
resync the service. A consented restart is observed for up to 180 seconds.
For PTP, verification also re-checks the single network path and W32Time's
UDP 319/320 sockets; sockets bound to another adapter's address roll back.
It then watches until 120 seconds after the restart for Windows Time event 34
(a rejected, implausibly large correction); any such event leaves the
configuration in place but reports synchronization as unverified (exit 6).

Backup files are UTF-8 JSON envelopes (schema configTimeSettingsBackup,
version 1) containing a base64-encoded UTF-8 JSON payload and the lowercase
SHA-256 digest of those exact payload bytes. Restore validates the digest,
schema, types, allowlist, machine identity, and size/depth limits. The digest
detects corruption; it is not an authenticity signature.

Exit codes: 0 success/help/deferred activation/PTP ready; 1 syntax/address/
backup-file validation; 2 permission/policy/environment/prerequisite refusal;
3 reachability, no eligible NIC, or PTP not ready; 4 directory/backup/I/O failure before mutation; 5 apply or
restore verification failure with rollback; 6 synchronization unverified after
the observation period; 7 rollback failure or unknown configuration state.

Examples:
  .\tools\configTimeSettings.ps1 -inventory 192.168.1.24
  .\tools\configTimeSettings.ps1 -readiness 192.168.1.24
  .\tools\configTimeSettings.ps1 -set NTP -serveraddress1 192.168.1.24
  .\tools\configTimeSettings.ps1 -set NTP -serveraddress1 2001:db8::24
  .\tools\configTimeSettings.ps1 -set PTP 192.168.1.24
  .\tools\configTimeSettings.ps1 -restore ".\configtimebackup_2026_11_05_11_20_30.bak"

configTimeSettings v1.1
Copyright Rob Latour, 2026
License: MIT
Website: https://github.com/roblatour/ESP32TimeServer
'@
}

function Stop-Config([int]$Code, [string]$Message) {
    [Console]::Error.WriteLine("ERROR: $Message")
    exit $Code
}

function Parse-Arguments([string[]]$Tokens) {
    $result = [ordered]@{}
    $allowed = @(
        'backup','restore','inventory','readiness','set','serveraddress1',
        'serveraddress2','serveraddress3','serveraddress4','help','?'
    )
    for ($i = 0; $i -lt $Tokens.Count; $i++) {
        $token = [string]$Tokens[$i]
        if (-not $token.StartsWith('-') -or $token -eq '-') {
            Stop-Config 1 "unexpected positional token '$token'"
        }
        $name = $token.Substring(1)
        if ($name -ieq 'nic') {
            Stop-Config 1 "parameter '-nic' is no longer supported: Windows PTP cannot be bound to a chosen NIC, so -set PTP uses the single network path verified by -readiness"
        }
        if ($allowed -notcontains $name.ToLowerInvariant()) {
            Stop-Config 1 "unknown parameter '$token'"
        }
        $key = $name.ToLowerInvariant()
        $logical = if ($key -eq '?') { 'help' } else { $key }
        if ($result.Contains($logical)) {
            Stop-Config 1 "parameter '-$name': command line argument was duplicated"
        }
        if ($logical -in @('backup','help')) {
            $result[$logical] = $true
            continue
        }
        if (($i + 1) -ge $Tokens.Count -or ([string]$Tokens[$i + 1]).StartsWith('-')) {
            Stop-Config 1 "parameter '-$name' requires a non-empty value"
        }
        $i++
        $value = [string]$Tokens[$i]
        if ([string]::IsNullOrWhiteSpace($value)) {
            Stop-Config 1 "parameter '-$name' requires a non-empty value"
        }
        $result[$logical] = $value
        if ($logical -eq 'set' -and $value -ieq 'PTP') {
            if (($i + 1) -ge $Tokens.Count -or ([string]$Tokens[$i + 1]).StartsWith('-') -or
                [string]::IsNullOrWhiteSpace([string]$Tokens[$i + 1])) {
                Stop-Config 1 '-set PTP requires an IPv4 server address immediately after PTP'
            }
            $i++
            $result['serveraddress'] = [string]$Tokens[$i]
        }
    }

    $operations = @('backup','restore','inventory','readiness','set','help') |
        Where-Object { $result.Contains($_) }
    if ($operations.Count -gt 1) {
        Stop-Config 1 'exactly one operation is allowed per invocation'
    }
    if ($operations.Count -eq 0) { $result['help'] = $true }
    if ($result.Contains('help')) {
        if ($result.Count -gt 1) { Stop-Config 1 '-help cannot be combined with another parameter' }
        return $result
    }
    $operation = $operations | Select-Object -First 1
    $valid = switch ($operation) {
        'backup' { @('backup') }
        'restore' { @('restore') }
        'inventory' { @('inventory') }
        'readiness' { @('readiness') }
        'set' { @('set','serveraddress','serveraddress1','serveraddress2',
            'serveraddress3','serveraddress4') }
    }
    foreach ($key in $result.Keys) {
        if ($valid -notcontains $key) {
            Stop-Config 1 "parameter '-$key' is not valid for -$operation"
        }
    }
    if ($operation -eq 'restore' -and -not $result.Contains('restore')) {
        Stop-Config 1 '-restore requires a filename'
    }
    if ($operation -eq 'inventory' -and -not $result.Contains('inventory')) {
        Stop-Config 1 '-inventory requires a server address'
    }
    if ($operation -eq 'set') {
        if (-not $result.Contains('set')) { Stop-Config 1 '-set requires NTP or PTP' }
        $protocol = ([string]$result['set']).ToUpperInvariant()
        if ($protocol -notin @('NTP','PTP')) { Stop-Config 1 "-set value '$($result['set'])' must be NTP or PTP" }
        $result['set'] = $protocol
        if ($protocol -eq 'NTP') {
            if (-not $result.Contains('serveraddress1')) { Stop-Config 1 '-serveraddress1 is required for NTP' }
            for ($n = 2; $n -le 4; $n++) {
                if ($result.Contains("serveraddress$n") -and -not $result.Contains("serveraddress$($n - 1)")) {
                    Stop-Config 1 "-serveraddress$n requires -serveraddress$($n - 1)"
                }
            }
        } else {
            if ($result.Keys | Where-Object { $_ -match '^serveraddress[1-4]$' }) {
                Stop-Config 1 'NTP peer parameters cannot be combined with PTP'
            }
        }
    }
    return $result
}

function Test-Address([string]$Address, [bool]$Ipv4Only = $false) {
    if ($Address -match '[/\[\]]|://') {
        return $null
    }
    if ($Address.Contains(':')) {
        if ($Ipv4Only) { return $null }
        $parts = $Address.Split('%')
        if ($parts.Count -gt 2 -or $parts[0] -match '\s') { return $null }
        $parsed = $null
        if (-not [Net.IPAddress]::TryParse($parts[0], [ref]$parsed) -or
            $parsed.AddressFamily -ne [Net.Sockets.AddressFamily]::InterNetworkV6) { return $null }
        if ($parsed.GetAddressBytes()[0] -eq 255 -or [Net.IPAddress]::IPv6Any.Equals($parsed) -or
            [Net.IPAddress]::IPv6Loopback.Equals($parsed)) { return $null }
        if ($parts[0].Contains('.') -and $null -eq (Test-Address ($parts[0].Substring($parts[0].LastIndexOf(':') + 1)) $true)) {
            return $null
        }
        if ($parsed.IsIPv4MappedToIPv6 -and $null -eq (Test-Address ($parsed.MapToIPv4().ToString()) $true)) {
            return $null
        }
        if ($parsed.IsIPv6LinkLocal) {
            if ($parts.Count -ne 2 -or [string]::IsNullOrWhiteSpace($parts[1])) { return $null }
            if ($parts[1] -notmatch '^[A-Za-z0-9_. {}-]+$') { return $null }
            try {
                $scopeAdapters = @(Get-NetAdapter -IncludeHidden -ErrorAction Stop |
                    Where-Object {
                        ([string]$_.ifIndex -eq $parts[1]) -or
                        ([string]$_.Name -ceq $parts[1]) -or
                        ([string]$_.InterfaceGuid -eq $parts[1])
                    })
            } catch { Stop-Config 2 "IPv6 scope validation failed: $($_.Exception.Message)" }
            if ($scopeAdapters.Count -ne 1 -or $scopeAdapters[0].ifIndex -le 0) { return $null }
            $parsed.ScopeId = [long]$scopeAdapters[0].ifIndex
        } elseif ($parts.Count -eq 2) { return $null }
        $text = $parsed.ToString().ToLowerInvariant()
        return [pscustomobject]@{ Text = $text; Normalized = $text; Family = 'IPv6'; IP = $parsed }
    }
    if ($Address -notmatch '^[0-9]{1,3}(\.[0-9]{1,3}){3}$') { return $null }
    $octets = $Address.Split('.')
    foreach ($octet in $octets) {
        if (($octet.Length -gt 1 -and $octet.StartsWith('0')) -or [int]$octet -gt 255) { return $null }
    }
    $ip4 = [Net.IPAddress]::Parse($Address)
    if ($ip4.Equals([Net.IPAddress]::Any) -or [Net.IPAddress]::IsLoopback($ip4) -or
        [byte]$octets[0] -eq 0 -or [byte]$octets[0] -ge 224) { return $null }
    return [pscustomobject]@{ Text = $Address; Normalized = $ip4.ToString(); Family = 'IPv4'; IP = $ip4 }
}

function Test-Administrator {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($identity)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

function Test-VirtualMachine($Computer) {
    return ([string]$Computer.Model -match 'Virtual|VMware|VirtualBox|KVM|HVM|Xen|Hyper-V|Google Compute Engine' -or
        [string]$Computer.Manufacturer -match 'VMware|VirtualBox|QEMU|Xen|Parallels')
}

function Get-PhysicalComputerSystem {
    try { $computers = @(Get-CimInstance -ClassName Win32_ComputerSystem -ErrorAction Stop) }
    catch { Stop-Config 2 "virtual-machine detection failed: $($_.Exception.Message)" }
    if ($computers.Count -ne 1) {
        Stop-Config 2 'virtual-machine detection failed: Windows did not return one computer system'
    }
    $computer = $computers[0]
    if (Test-VirtualMachine $computer) {
        Stop-Config 2 'A virtual machine was detected. This script is unable to work on a Virtual Machine.'
    }
    if ([string]::IsNullOrWhiteSpace([string]$computer.Manufacturer) -or
        [string]::IsNullOrWhiteSpace([string]$computer.Model)) {
        Stop-Config 2 'virtual-machine detection failed: the computer manufacturer or model is unavailable'
    }
    return $computer
}

function Enter-MachineLock {
    $mutex = New-Object Threading.Mutex($false, 'Global\ESP32TimeServer-configTimeSettings')
    try { $locked = $mutex.WaitOne(0) } catch [Threading.AbandonedMutexException] { $locked = $true }
    if (-not $locked) {
        $mutex.Dispose()
        Stop-Config 2 'another configTimeSettings backup, set, or restore operation is in progress'
    }
    $script:Mutex = $mutex
}

function Exit-MachineLock {
    if ($null -ne $script:Mutex) {
        try { $script:Mutex.ReleaseMutex() } finally { $script:Mutex.Dispose(); $script:Mutex = $null }
    }
}

function Test-CurrentDirectory {
    $location = Get-Location
    if ($location.Provider.Name -ne 'FileSystem' -or -not (Test-Path -LiteralPath $location.Path -PathType Container)) {
        Stop-Config 4 'the current directory must be a readable and writable filesystem directory'
    }
    $probe = Join-Path $location.Path ('.configTimeSettings_probe_' + [Guid]::NewGuid().ToString('N'))
    $bytes = [Text.Encoding]::UTF8.GetBytes('configTimeSettings directory probe')
    $stream = $null
    try {
        Get-ChildItem -LiteralPath $location.Path -Force -ErrorAction Stop | Out-Null
        $stream = New-Object IO.FileStream($probe, [IO.FileMode]::CreateNew, [IO.FileAccess]::ReadWrite, [IO.FileShare]::None)
        $stream.Write($bytes, 0, $bytes.Length)
        $stream.Flush($true)
        $stream.Dispose()
        $stream = $null
        $read = [IO.File]::ReadAllBytes($probe)
        if ([Convert]::ToBase64String($read) -ne [Convert]::ToBase64String($bytes)) {
            throw 'directory probe readback did not match'
        }
        Remove-Item -LiteralPath $probe -Force -ErrorAction Stop
    } catch {
        if ($null -ne $stream) { $stream.Dispose() }
        if (Test-Path -LiteralPath $probe) {
            try { Remove-Item -LiteralPath $probe -Force -ErrorAction Stop }
            catch { Stop-Config 4 "directory access check failed and temporary file could not be removed: '$probe'" }
        }
        Stop-Config 4 "current-directory read/write check failed: $($_.Exception.Message)"
    }
}

function Test-Environment([bool]$WarningsOnly = $false) {
    try { $os = Get-CimInstance -ClassName Win32_OperatingSystem -ErrorAction Stop }
    catch { Stop-Config 2 "Windows operating-system prerequisites could not be queried: $($_.Exception.Message)" }
    if (-not [Environment]::Is64BitOperatingSystem -or -not [Environment]::Is64BitProcess -or
        $PSVersionTable.PSEdition -eq 'Core' -or $PSVersionTable.PSVersion.Major -ne 5) {
        Stop-Config 2 'this operation requires 64-bit Windows PowerShell 5.1'
    }
    $buildNumber = 0
    if (-not [int]::TryParse([string]$os.BuildNumber, [ref]$buildNumber)) {
        Stop-Config 2 'Windows build number could not be established'
    }
    if (($os.ProductType -eq 1 -and $buildNumber -lt 22631) -or
        ($os.ProductType -ne 1 -and $buildNumber -lt 20348)) {
        if (-not $WarningsOnly) { Stop-Config 2 'Windows release is outside the supported Windows 11 23H2-or-newer / Windows Server 2022-or-newer baseline' }
        Write-Warning 'Windows build is below the supported release baseline; inventory is informational only'
    }
    $computer = Get-PhysicalComputerSystem
    $domainController = ($os.ProductType -ne 1)
    $policy = $false
    $policyUnknown = $false
    try {
        $policyParent = [Microsoft.Win32.Registry]::LocalMachine.OpenSubKey('SOFTWARE\Policies\Microsoft', $false)
        if ($null -ne $policyParent) {
            try {
                $policyKey = $policyParent.OpenSubKey('W32Time', $false)
                if ($null -ne $policyKey) { $policy = $true; $policyKey.Dispose() }
            } finally { $policyParent.Dispose() }
        }
    } catch {
        if (-not $WarningsOnly) { Stop-Config 2 "W32Time policy status could not be queried: $($_.Exception.Message)" }
        $policyUnknown = $true
    }
    $warnings = @()
    if ($computer.PartOfDomain) { $warnings += 'Domain-joined computer detected' }
    if ($domainController) { $warnings += 'Domain controller detected' }
    if ($policy) { $warnings += 'managed W32Time policy detected' }
    if ($policyUnknown) { $warnings += 'W32Time policy status is Unknown because the registry query was denied' }
    if ($WarningsOnly) {
        foreach ($warning in $warnings) { Write-Warning $warning }
    } elseif ($warnings.Count -gt 0) {
        Stop-Config 2 ("standalone physical-computer safety refusal: " + ($warnings -join '; ') +
            '. Use an administrator-designed configuration.')
    }
    return $computer
}

function Get-RegistryValueState([string]$RelativePath, [string]$ValueName) {
    $key = [Microsoft.Win32.Registry]::LocalMachine.OpenSubKey("$script:RegistryRoot\$RelativePath", $false)
    if ($null -eq $key) { return [ordered]@{ path = $RelativePath; name = $ValueName; present = $false } }
    try {
        if ($key.GetValueNames() -notcontains $ValueName) {
            return [ordered]@{ path = $RelativePath; name = $ValueName; present = $false }
        }
        $kind = $key.GetValueKind($ValueName).ToString()
        $value = $key.GetValue($ValueName, $null, [Microsoft.Win32.RegistryValueOptions]::DoNotExpandEnvironmentNames)
        if ($kind -eq 'Binary') { $value = [Convert]::ToBase64String([byte[]]$value) }
        return [ordered]@{ path = $RelativePath; name = $ValueName; present = $true; kind = $kind; value = $value }
    } finally { $key.Dispose() }
}

function Set-RegistryValueState($State) {
    $relative = [string]$State.path
    $name = [string]$State.name
    if (-not $script:KnownValues.Contains("$relative\$name")) {
        throw "backup contains a non-allowlisted registry value '$relative\$name'"
    }
    $fullPath = "$script:RegistryRoot\$relative"
    if (-not $State.present) {
        $key = [Microsoft.Win32.Registry]::LocalMachine.OpenSubKey($fullPath, $true)
        if ($null -ne $key) { try { $key.DeleteValue($name, $false) } finally { $key.Dispose() } }
        return
    }
    $kind = [Microsoft.Win32.RegistryValueKind][Enum]::Parse(
        [Microsoft.Win32.RegistryValueKind], [string]$State.kind, $false)
    $value = $State.value
    if ($kind -eq [Microsoft.Win32.RegistryValueKind]::Binary) { $value = [Convert]::FromBase64String([string]$value) }
    $key = [Microsoft.Win32.Registry]::LocalMachine.CreateSubKey($fullPath)
    try { $key.SetValue($name, $value, $kind) } finally { $key.Dispose() }
}

function Get-ServiceState {
    $service = Get-CimInstance -ClassName Win32_Service -Filter "Name='W32Time'" -ErrorAction Stop
    if ($null -eq $service) { throw 'W32Time service does not exist' }
    $until = [DateTime]::UtcNow.AddSeconds(30)
    while ($service.State -in @('Start Pending','Stop Pending','Continue Pending','Pause Pending','Paused')) {
        if ([DateTime]::UtcNow -ge $until) { throw "W32Time state '$($service.State)' did not settle within 30 seconds" }
        Start-Sleep -Milliseconds 500
        $service = Get-CimInstance -ClassName Win32_Service -Filter "Name='W32Time'" -ErrorAction Stop
    }
    $delayedKey = [Microsoft.Win32.Registry]::LocalMachine.OpenSubKey('SYSTEM\CurrentControlSet\Services\W32Time', $false)
    if ($null -eq $delayedKey) { throw 'W32Time startup registry state could not be read' }
    try {
        $names = @()
        if ($null -ne $delayedKey) { $names = $delayedKey.GetValueNames() }
        $delayed = if ($names -contains 'DelayedAutoStart') {
            [ordered]@{ present = $true; kind = $delayedKey.GetValueKind('DelayedAutoStart').ToString(); value = $delayedKey.GetValue('DelayedAutoStart') }
        } else { [ordered]@{ present = $false } }
    } finally { if ($null -ne $delayedKey) { $delayedKey.Dispose() } }
    return [ordered]@{ startMode = [string]$service.StartMode; delayedAutoStart = $delayed; state = [string]$service.State }
}

function Get-OwnedFirewallState {
    $saved = @()
    if (-not (Get-Command Get-NetFirewallRule -ErrorAction SilentlyContinue)) {
        throw 'Windows Defender Firewall rule query API is unavailable; complete backup state cannot be established'
    }
    $allRules = @(Get-NetFirewallRule -ErrorAction Stop)
    foreach ($name in $script:OwnedRuleNames) {
        $rules = @($allRules | Where-Object { [string]$_.Name -ceq $name })
        foreach ($rule in $rules) {
            if ($rule.Description -notmatch 'configTimeSettings ownership marker' -or
                $rule.Group -ne 'ESP32TimeServer-configTimeSettings') {
                throw "firewall rule name collision for '$name'; ownership could not be verified"
            }
            $port = $rule | Get-NetFirewallPortFilter -ErrorAction Stop
            $address = $rule | Get-NetFirewallAddressFilter -ErrorAction Stop
            $interface = $rule | Get-NetFirewallInterfaceFilter -ErrorAction Stop
            $application = $rule | Get-NetFirewallApplicationFilter -ErrorAction Stop
            $service = $rule | Get-NetFirewallServiceFilter -ErrorAction Stop
            $saved += [ordered]@{
                name = [string]$rule.Name; displayName = [string]$rule.DisplayName
                description = [string]$rule.Description; group = [string]$rule.Group
                direction = [string]$rule.Direction; action = [string]$rule.Action
                enabled = [string]$rule.Enabled; profile = [string]$rule.Profile
                protocol = [string]$port.Protocol; localPort = @($port.LocalPort)
                remotePort = @($port.RemotePort); localAddress = @($address.LocalAddress)
                remoteAddress = @($address.RemoteAddress); interfaceAlias = @($interface.InterfaceAlias)
                application = [string]$application.Program; service = [string]$service.Service
            }
        }
    }
    return $saved
}

function Get-BackupPayload {
    $registry = @()
    foreach ($entry in $script:KnownValues.Keys) {
        $separator = $entry.LastIndexOf('\')
        $path = $entry.Substring(0, $separator)
        $name = $entry.Substring($separator + 1)
        $registry += Get-RegistryValueState $path $name
    }
    $identity = Get-CimInstance Win32_ComputerSystem
    $os = Get-CimInstance Win32_OperatingSystem
    $machineGuid = (Get-ItemProperty -LiteralPath 'HKLM:\SOFTWARE\Microsoft\Cryptography' -Name MachineGuid -ErrorAction Stop).MachineGuid
    $nic = $null
    if ($script:SelectedNic) {
        $addresses = @(Get-NetIPAddress -InterfaceIndex $script:SelectedNic.ifIndex -AddressFamily IPv4 -ErrorAction Stop |
            Select-Object -ExpandProperty IPAddress)
        $nic = [ordered]@{
            name = $script:SelectedNic.Name; guid = $script:SelectedNic.InterfaceGuid
            index = $script:SelectedNic.ifIndex; mac = $script:SelectedNic.MacAddress
            ipv4Addresses = $addresses
        }
    }
    return [ordered]@{
        machine = [ordered]@{ name = [Environment]::MachineName; machineGuid = [string]$machineGuid; domain = [string]$identity.Domain; osCaption = [string]$os.Caption; build = [string]$os.BuildNumber; architecture = [string]$os.OSArchitecture }
        scriptVersion = $script:Version; schemaVersion = 1
        capturedUtc = [DateTime]::UtcNow.ToString('o')
        capturedLocalOffset = [DateTimeOffset]::Now.Offset.ToString()
        selectedNic = $nic; registry = $registry
        ptpClientKeyExisted = [bool](Test-Path "HKLM:\$script:RegistryRoot\TimeProviders\PtpClient")
        service = Get-ServiceState
        firewallRules = @(Get-OwnedFirewallState)
    }
}

function Protect-FileForCurrentUser([string]$Path) {
    $acl = New-Object Security.AccessControl.FileSecurity
    $acl.SetAccessRuleProtection($true, $false)
    $currentSid = [Security.Principal.WindowsIdentity]::GetCurrent().User
    $adminSid = New-Object Security.Principal.SecurityIdentifier('S-1-5-32-544')
    $systemSid = New-Object Security.Principal.SecurityIdentifier('S-1-5-18')
    foreach ($sid in @($currentSid, $adminSid, $systemSid)) {
        $rule = New-Object Security.AccessControl.FileSystemAccessRule(
            $sid, [Security.AccessControl.FileSystemRights]::FullControl,
            [Security.AccessControl.AccessControlType]::Allow)
        $acl.AddAccessRule($rule)
    }
    Set-Acl -LiteralPath $Path -AclObject $acl -ErrorAction Stop
    $check = Get-Acl -LiteralPath $Path -ErrorAction Stop
    if (-not $check.AreAccessRulesProtected) { throw 'backup ACL inheritance could not be disabled' }
}

function Write-Backup([bool]$SafetyOnly = $false) {
    $directory = (Get-Location).Path
    $deadline = [DateTime]::UtcNow.AddSeconds(5)
    do {
        $stamp = Get-Date -Format 'yyyy_MM_dd_HH_mm_ss'
        $final = Join-Path $directory "configtimebackup_$stamp.bak"
        if (-not (Test-Path -LiteralPath $final)) { break }
        Start-Sleep -Milliseconds 250
    } while ([DateTime]::UtcNow -lt $deadline)
    if (Test-Path -LiteralPath $final) { Stop-Config 4 'backup filename collision persisted for five seconds' }

    $temp = Join-Path $directory ('.configtimebackup_' + [Guid]::NewGuid().ToString('N') + '.tmp')
    try {
        $payloadObject = Get-BackupPayload
        $payloadJson = ConvertTo-Json -InputObject $payloadObject -Depth 20 -Compress
        $payloadBytes = (New-Object Text.UTF8Encoding($false)).GetBytes($payloadJson)
        $payloadHash = [Security.Cryptography.SHA256]::Create()
        try { $digest = ([BitConverter]::ToString($payloadHash.ComputeHash($payloadBytes))).Replace('-','').ToLowerInvariant() }
        finally { $payloadHash.Dispose() }
        $envelope = [ordered]@{ schema = 'configTimeSettingsBackup'; schemaVersion = 1; payloadBase64 = [Convert]::ToBase64String($payloadBytes); payloadSha256 = $digest }
        $envelopeBytes = (New-Object Text.UTF8Encoding($false)).GetBytes((ConvertTo-Json -InputObject $envelope -Depth 5 -Compress))
        if ($envelopeBytes.Length -gt 10MB) { throw 'backup exceeds the 10 MiB size limit' }
        $stream = New-Object IO.FileStream($temp, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write, [IO.FileShare]::None)
        $stream.Dispose()
        Protect-FileForCurrentUser $temp
        $stream = New-Object IO.FileStream($temp, [IO.FileMode]::Open, [IO.FileAccess]::Write, [IO.FileShare]::None)
        try { $stream.Write($envelopeBytes, 0, $envelopeBytes.Length); $stream.Flush($true) }
        finally { $stream.Dispose() }
        $readback = [IO.File]::ReadAllBytes($temp)
        if ([Convert]::ToBase64String($readback) -ne [Convert]::ToBase64String($envelopeBytes)) { throw 'backup temporary-file readback failed' }
        $null = Read-BackupFile $temp
        if ([IO.File]::Exists($final)) { throw "backup destination already exists: '$final'" }
        [IO.File]::Move($temp, $final)
        [Console]::Out.WriteLine("Backup created: $final")
        return $final
    } catch {
        if (Test-Path -LiteralPath $temp) {
            try { Remove-Item -LiteralPath $temp -Force -ErrorAction Stop }
            catch { Stop-Config 4 "backup failed and temporary file could not be removed: '$temp'; $($_.Exception.Message)" }
        }
        Stop-Config 4 "backup creation failed: $($_.Exception.Message)"
    }
}

function Test-NoDuplicateJsonPropertyNames([string]$Json) {
    # ConvertFrom-Json handles syntax and types; this scanner additionally rejects repeated object keys.
    $stack = New-Object 'System.Collections.Generic.Stack[object]'
    $inString = $false; $escaped = $false; $stringStart = -1; $depth = 0
    for ($i = 0; $i -lt $Json.Length; $i++) {
        $c = $Json[$i]
        if ($inString) {
            if ($escaped) { $escaped = $false; continue }
            if ($c -eq '\') { $escaped = $true; continue }
            if ($c -eq '"') {
                $inString = $false
                if ($stack.Count -gt 0 -and $stack.Peek().kind -eq 'object' -and $stack.Peek().expectKey) {
                    $raw = $Json.Substring($stringStart, $i - $stringStart + 1)
                    $key = ConvertFrom-Json -InputObject $raw -ErrorAction Stop
                    if (-not $stack.Peek().names.Add([string]$key)) { throw "duplicate JSON property '$key'" }
                    $stack.Peek().expectKey = $false
                }
            }
            continue
        }
        if ($c -eq '"') { $inString = $true; $stringStart = $i; continue }
        if ($c -eq '{') {
            $depth++
            if ($depth -gt 32) { throw 'JSON nesting exceeds the maximum depth of 32' }
            $stack.Push([pscustomobject]@{ kind = 'object'; names = (New-Object 'System.Collections.Generic.HashSet[string]' ([StringComparer]::Ordinal)); expectKey = $true })
            continue
        }
        if ($c -eq '[') {
            $depth++
            if ($depth -gt 32) { throw 'JSON nesting exceeds the maximum depth of 32' }
            $stack.Push([pscustomobject]@{ kind = 'array' })
            continue
        }
        if ($c -eq '}' -or $c -eq ']') {
            if ($depth -gt 0) { $depth-- }
            if ($stack.Count -gt 0) { $null = $stack.Pop() }
            continue
        }
        if ($c -eq ',' -and $stack.Count -gt 0 -and $stack.Peek().kind -eq 'object') { $stack.Peek().expectKey = $true }
    }
}

function Read-BackupFile([string]$Path) {
    $bytes = [IO.File]::ReadAllBytes($Path)
    if ($bytes.Length -gt 10MB) { throw 'backup file exceeds the 10 MiB size limit' }
    $utf8 = New-Object Text.UTF8Encoding($false, $true)
    $json = $utf8.GetString($bytes)
    if ($json.Length -gt 0 -and $json[0] -eq [char]0xFEFF) { throw 'backup JSON must not contain a byte-order mark' }
    $null = Test-NoDuplicateJsonPropertyNames $json
    $envelope = ConvertFrom-Json -InputObject $json -ErrorAction Stop
    $envelopeFields = @($envelope.PSObject.Properties.Name | Sort-Object)
    if (($envelopeFields -join ',') -cne 'payloadBase64,payloadSha256,schema,schemaVersion') {
        throw 'backup envelope has missing or unexpected fields'
    }
    if ($envelope.schema -isnot [string] -or $envelope.schema -cne 'configTimeSettingsBackup' -or
        ($envelope.schemaVersion -isnot [int] -and $envelope.schemaVersion -isnot [long]) -or
        $envelope.schemaVersion -ne 1) { throw 'unsupported backup schema or version' }
    if ($envelope.payloadBase64 -isnot [string] -or $envelope.payloadSha256 -notmatch '^[0-9a-f]{64}$') { throw 'invalid backup envelope fields' }
    $payloadBytes = [Convert]::FromBase64String($envelope.payloadBase64)
    $sha = [Security.Cryptography.SHA256]::Create()
    try { $actual = ([BitConverter]::ToString($sha.ComputeHash($payloadBytes))).Replace('-','').ToLowerInvariant() } finally { $sha.Dispose() }
    if ($actual -cne $envelope.payloadSha256) { throw 'backup payload digest does not match' }
    $payloadJson = $utf8.GetString($payloadBytes)
    if ($payloadJson.Length -gt 0 -and $payloadJson[0] -eq [char]0xFEFF) { throw 'backup payload JSON must not contain a byte-order mark' }
    $null = Test-NoDuplicateJsonPropertyNames $payloadJson
    $payload = ConvertFrom-Json -InputObject $payloadJson -ErrorAction Stop
    $payloadFields = @($payload.PSObject.Properties.Name | Sort-Object)
    $expectedPayloadFields = @('capturedLocalOffset','capturedUtc','firewallRules','machine',
        'ptpClientKeyExisted','registry','schemaVersion','scriptVersion','selectedNic','service') | Sort-Object
    if (($payloadFields -join ',') -cne ($expectedPayloadFields -join ',') -or
        ($payload.schemaVersion -isnot [int] -and $payload.schemaVersion -isnot [long]) -or
        $payload.schemaVersion -ne 1 -or
        $payload.registry -isnot [Array] -or
        $payload.firewallRules -isnot [Array] -or
        $payload.ptpClientKeyExisted -isnot [bool] -or
        $payload.scriptVersion -isnot [string] -or
        $payload.machine.name -isnot [string] -or
        $payload.machine.machineGuid -isnot [string] -or
        $payload.machine.osCaption -isnot [string] -or
        $payload.machine.build -isnot [string] -or
        $payload.machine.architecture -isnot [string]) {
        throw 'backup payload is missing required fields or has an unsupported version'
    }
    $machineFields = @('architecture','build','domain','machineGuid','name','osCaption')
    $serviceFields = @('delayedAutoStart','startMode','state')
    if ((@($payload.machine.PSObject.Properties.Name | Sort-Object) -join ',') -cne
        (@($machineFields | Sort-Object) -join ',') -or
        (@($payload.service.PSObject.Properties.Name | Sort-Object) -join ',') -cne
        (@($serviceFields | Sort-Object) -join ',') -or
        $payload.scriptVersion -notin @($script:LegacyBackupVersion, $script:Version)) {
        throw 'backup contains missing or unexpected machine/service fields or an incompatible script version'
    }
    # Version 1.0 backups predate the managed clock-discipline values; restoring them leaves those values unchanged.
    $expectedKeys = @($script:KnownValues.Keys)
    if ($payload.scriptVersion -ceq $script:LegacyBackupVersion) {
        $expectedKeys = @($expectedKeys | Where-Object { $script:ClockDisciplineKeys -notcontains $_ })
    }
    if ($payload.registry.Count -ne $expectedKeys.Count) { throw 'backup does not contain every allowlisted registry state' }
    $registryKeys = New-Object 'System.Collections.Generic.HashSet[string]' ([StringComparer]::OrdinalIgnoreCase)
    foreach ($state in $payload.registry) {
        $stateFields = @($state.PSObject.Properties.Name | Sort-Object)
        if ($stateFields -notcontains 'path' -or $stateFields -notcontains 'name' -or $stateFields -notcontains 'present' -or
            $state.present -isnot [bool]) { throw 'backup contains a malformed registry state' }
        $registryKey = "$($state.path)\$($state.name)"
        if ($expectedKeys -notcontains $registryKey) { throw "backup contains a non-allowlisted registry value '$registryKey'" }
        if (-not $registryKeys.Add($registryKey)) { throw "backup contains duplicate registry state '$registryKey'" }
        if ($state.present) {
            if ($stateFields.Count -ne 5 -or $state.kind -isnot [string] -or
                $state.kind -notin @('String','ExpandString','Binary','DWord','QWord','MultiString')) {
                throw "backup contains an invalid typed registry state '$registryKey'"
            }
            switch ($state.kind) {
                { $_ -in @('String','ExpandString','Binary') } {
                    if ($state.value -isnot [string]) { throw "backup value for '$registryKey' has the wrong type" }
                    if ($state.kind -eq 'Binary') { $null = [Convert]::FromBase64String($state.value) }
                }
                'DWord' { if ($state.value -isnot [int] -and $state.value -isnot [long]) { throw "backup value for '$registryKey' has the wrong type" } }
                'QWord' { if ($state.value -isnot [long]) { throw "backup value for '$registryKey' has the wrong type" } }
                'MultiString' {
                    if ($state.value -isnot [Array] -or @($state.value | Where-Object { $_ -isnot [string] }).Count -gt 0) {
                        throw "backup value for '$registryKey' has the wrong type"
                    }
                }
            }
        } elseif ($stateFields.Count -ne 3) { throw "absent registry state '$registryKey' contains unexpected data" }
    }
    if ($registryKeys.Count -ne $expectedKeys.Count) { throw 'backup does not contain every allowlisted registry state' }
    foreach ($rule in $payload.firewallRules) {
        $firewallFields = @('action','application','description','direction','displayName',
            'enabled','group','interfaceAlias','localAddress','localPort','name','profile',
            'protocol','remoteAddress','remotePort','service')
        if ($rule.name -notin $script:OwnedRuleNames -or
            (@($rule.PSObject.Properties.Name | Sort-Object) -join ',') -cne
            (@($firewallFields | Sort-Object) -join ',')) {
            throw "backup contains malformed or non-allowlisted firewall rule '$($rule.name)'"
        }
    }
    if ($payload.service.startMode -notin @('Auto','Manual','Disabled') -or
        $payload.service.state -notin @('Running','Stopped') -or
        $payload.service.delayedAutoStart.present -isnot [bool] -or
        [string]::IsNullOrWhiteSpace($payload.capturedUtc) -or
        [string]::IsNullOrWhiteSpace($payload.capturedLocalOffset)) {
        throw 'backup payload contains invalid machine or service state'
    }
    $delayedFields = @($payload.service.delayedAutoStart.PSObject.Properties.Name | Sort-Object)
    if ($payload.service.delayedAutoStart.present) {
        if (($delayedFields -join ',') -cne 'kind,present,value' -or
            $payload.service.delayedAutoStart.kind -ne 'DWord' -or
            ($payload.service.delayedAutoStart.value -isnot [int] -and
                $payload.service.delayedAutoStart.value -isnot [long])) {
            throw 'backup contains invalid W32Time delayed-start state'
        }
    } elseif (($delayedFields -join ',') -cne 'present') {
        throw 'absent W32Time delayed-start state contains unexpected data'
    }
    if ($null -ne $payload.selectedNic) {
        $nicFields = @('guid','index','ipv4Addresses','mac','name')
        if ((@($payload.selectedNic.PSObject.Properties.Name | Sort-Object) -join ',') -cne
            (@($nicFields | Sort-Object) -join ',') -or
            $payload.selectedNic.guid -isnot [string] -or
            $payload.selectedNic.ipv4Addresses -isnot [Array]) {
            throw 'backup contains invalid selected NIC identity'
        }
    }
    $captured = [DateTimeOffset]::MinValue
    if (-not [DateTimeOffset]::TryParse([string]$payload.capturedUtc, [ref]$captured)) {
        throw 'backup payload has an invalid UTC capture timestamp'
    }
    $offset = [TimeSpan]::Zero
    if (-not [TimeSpan]::TryParse([string]$payload.capturedLocalOffset, [ref]$offset) -or
        $offset -lt [TimeSpan]::FromHours(-14) -or $offset -gt [TimeSpan]::FromHours(14)) {
        throw 'backup payload has an invalid local UTC offset'
    }
    return $payload
}

function Test-BackupPath([string]$Path) {
    if ([string]::IsNullOrWhiteSpace($Path) -or $Path.IndexOfAny([IO.Path]::GetInvalidPathChars()) -ge 0 -or
        $Path -match '[*?]' -or $Path -match '^(\\\\[.?]\\|\\\\[^\\]+\\[^\\]+)' -or
        $Path -match ':[^\\]*:' -or (Test-Path -LiteralPath $Path -PathType Container)) {
        Stop-Config 1 "restore path '$Path' must be a literal local file path without wildcards, device paths, streams, or directories"
    }
    $leaf = [IO.Path]::GetFileName($Path)
    if ([IO.Path]::GetExtension($leaf) -ine '.bak') {
        Stop-Config 1 "restore filename '$leaf' must have a .bak extension"
    }
    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) { Stop-Config 1 "backup file '$Path' does not exist" }
    $item = Get-Item -LiteralPath $Path -Force -ErrorAction Stop
    if (($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) {
        Stop-Config 1 "restore file '$Path' cannot be a symbolic link or reparse point"
    }
    try {
        $fullPath = [IO.Path]::GetFullPath($Path)
        $root = [IO.Path]::GetPathRoot($fullPath)
        if ($root.StartsWith('\\') -or (New-Object IO.DriveInfo($root)).DriveType -eq [IO.DriveType]::Network) {
            Stop-Config 1 "restore file '$Path' must be on a local filesystem volume"
        }
    } catch { Stop-Config 1 "restore path '$Path' could not be resolved as a local filesystem path" }
}

function Test-Ping([string]$Address) {
    $ping = New-Object Net.NetworkInformation.Ping
    $successes = 0
    try {
        for ($i = 0; $i -lt 3; $i++) {
            try {
                $reply = $ping.Send($Address, 1000)
                if ($reply.Status -eq [Net.NetworkInformation.IPStatus]::Success) { $successes++ }
            } catch { }
        }
    } finally { $ping.Dispose() }
    return ($successes -gt 0)
}

function Initialize-TimestampApi {
    if ('ConfigTimeSettingsTimestampApi' -as [type]) { return }
    $source = @'
using System;
using System.Runtime.InteropServices;

public static class ConfigTimeSettingsTimestampApi
{
    [StructLayout(LayoutKind.Sequential)]
    public struct NetLuid
    {
        public ulong Value;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct HardwareCapabilities
    {
        [MarshalAs(UnmanagedType.U1)] public bool PtpV2OverUdpIPv4EventMessageReceive;
        [MarshalAs(UnmanagedType.U1)] public bool PtpV2OverUdpIPv4AllMessageReceive;
        [MarshalAs(UnmanagedType.U1)] public bool PtpV2OverUdpIPv4EventMessageTransmit;
        [MarshalAs(UnmanagedType.U1)] public bool PtpV2OverUdpIPv4AllMessageTransmit;
        [MarshalAs(UnmanagedType.U1)] public bool PtpV2OverUdpIPv6EventMessageReceive;
        [MarshalAs(UnmanagedType.U1)] public bool PtpV2OverUdpIPv6AllMessageReceive;
        [MarshalAs(UnmanagedType.U1)] public bool PtpV2OverUdpIPv6EventMessageTransmit;
        [MarshalAs(UnmanagedType.U1)] public bool PtpV2OverUdpIPv6AllMessageTransmit;
        [MarshalAs(UnmanagedType.U1)] public bool AllReceive;
        [MarshalAs(UnmanagedType.U1)] public bool AllTransmit;
        [MarshalAs(UnmanagedType.U1)] public bool TaggedTransmit;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SoftwareCapabilities
    {
        [MarshalAs(UnmanagedType.U1)] public bool AllReceive;
        [MarshalAs(UnmanagedType.U1)] public bool AllTransmit;
        [MarshalAs(UnmanagedType.U1)] public bool TaggedTransmit;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct TimestampCapabilities
    {
        public ulong HardwareClockFrequencyHz;
        [MarshalAs(UnmanagedType.U1)] public bool SupportsCrossTimestamp;
        public HardwareCapabilities Hardware;
        public SoftwareCapabilities Software;
    }

    [DllImport("iphlpapi.dll", ExactSpelling = true)]
    public static extern uint ConvertInterfaceIndexToLuid(uint interfaceIndex, out NetLuid interfaceLuid);

    [DllImport("iphlpapi.dll", ExactSpelling = true)]
    public static extern uint GetInterfaceSupportedTimestampCapabilities(
        ref NetLuid interfaceLuid, out TimestampCapabilities capabilities);

    [DllImport("iphlpapi.dll", ExactSpelling = true)]
    public static extern uint GetInterfaceActiveTimestampCapabilities(
        ref NetLuid interfaceLuid, out TimestampCapabilities capabilities);
}
'@
    Add-Type -TypeDefinition $source -Language CSharp -ErrorAction Stop
}

function Test-PhysicalEthernetAdapter($Adapter) {
    # Some wired drivers report the unspecified physical medium (0).
    return ([bool]$Adapter.HardwareInterface -and [string]$Adapter.MediaType -eq '802.3' -and
        $Adapter.NdisPhysicalMedium -in @(0, 14))
}

function Test-PtpV2Ipv4Hardware($Capabilities) {
    $hardware = $Capabilities.Hardware
    $receive = $hardware.PtpV2OverUdpIPv4EventMessageReceive -or
        $hardware.PtpV2OverUdpIPv4AllMessageReceive -or $hardware.AllReceive
    $transmit = $hardware.PtpV2OverUdpIPv4EventMessageTransmit -or
        $hardware.PtpV2OverUdpIPv4AllMessageTransmit -or
        $hardware.TaggedTransmit -or $hardware.AllTransmit
    return ($receive -and $transmit)
}

function Get-AdapterTimestampState($Adapter) {
    try { Initialize-TimestampApi }
    catch {
        return [pscustomobject]@{ Support = 'Unknown'; Active = 'Unknown'; Error = $_.Exception.Message }
    }
    try {
        $luid = New-Object ConfigTimeSettingsTimestampApi+NetLuid
        $result = [ConfigTimeSettingsTimestampApi]::ConvertInterfaceIndexToLuid(
            [uint32]$Adapter.ifIndex, [ref]$luid)
    } catch {
        return [pscustomobject]@{ Support = 'Unknown'; Active = 'Unknown'; Error = $_.Exception.Message }
    }
    if ($result -ne 0) {
        return [pscustomobject]@{ Support = 'Unknown'; Active = 'Unknown'; Error = "interface LUID conversion failed ($result)" }
    }
    $supportError = $null
    $activeError = $null
    $supported = New-Object ConfigTimeSettingsTimestampApi+TimestampCapabilities
    try {
        $result = [ConfigTimeSettingsTimestampApi]::GetInterfaceSupportedTimestampCapabilities(
            [ref]$luid, [ref]$supported)
        if ($result -eq 50) {
            $support = 'No'
        } elseif ($result -eq 0) {
            $support = if (Test-PtpV2Ipv4Hardware $supported) { 'Yes' } else { 'No' }
        } else {
            $support = 'Unknown'
            $supportError = "supported timestamp query failed ($result): $((New-Object ComponentModel.Win32Exception([int]$result)).Message)"
        }
    } catch {
        $support = 'Unknown'
        $supportError = $_.Exception.Message
    }
    $activeCaps = New-Object ConfigTimeSettingsTimestampApi+TimestampCapabilities
    try {
        $activeResult = [ConfigTimeSettingsTimestampApi]::GetInterfaceActiveTimestampCapabilities(
            [ref]$luid, [ref]$activeCaps)
        if ($activeResult -eq 50) {
            $active = 'No'
        } elseif ($activeResult -eq 0) {
            $active = if (Test-PtpV2Ipv4Hardware $activeCaps) { 'Yes' } else { 'No' }
        } else {
            $active = 'Unknown'
            if ($activeResult -eq 2001) {
                $activeError = 'active timestamp query failed (ERROR_BAD_DRIVER, 2001): driver advertises an unsupported timestamp configuration'
            } else {
                $activeError = "active timestamp query failed ($activeResult): $((New-Object ComponentModel.Win32Exception([int]$activeResult)).Message)"
            }
        }
    } catch {
        $active = 'Unknown'
        $activeError = $_.Exception.Message
    }
    $errors = @($supportError, $activeError) | Where-Object { $_ }
    return [pscustomobject]@{ Support = $support; Active = $active; Error = ($errors -join '; ') }
}

function Initialize-IcmpApi {
    if ('ConfigTimeSettingsIcmpApi' -as [type]) { return }
    $source = @'
using System;
using System.Runtime.InteropServices;

public static class ConfigTimeSettingsIcmpApi
{
    [DllImport("iphlpapi.dll", ExactSpelling = true, SetLastError = true)]
    public static extern IntPtr IcmpCreateFile();

    [DllImport("iphlpapi.dll", ExactSpelling = true, SetLastError = true)]
    public static extern bool IcmpCloseHandle(IntPtr icmpHandle);

    [DllImport("iphlpapi.dll", ExactSpelling = true, SetLastError = true)]
    private static extern uint IcmpSendEcho2Ex(
        IntPtr icmpHandle, IntPtr @event, IntPtr apcRoutine, IntPtr apcContext,
        uint sourceAddress, uint destinationAddress, byte[] requestData,
        ushort requestSize, IntPtr requestOptions, byte[] replyBuffer,
        uint replySize, uint timeout);

    public static uint SendEcho(
        IntPtr icmpHandle, uint sourceAddress, uint destinationAddress,
        uint timeout, out uint roundTripTime, out uint status, out int error)
    {
        byte[] requestData = new byte[32];
        byte[] replyBuffer = new byte[1024];
        uint replies = IcmpSendEcho2Ex(
            icmpHandle, IntPtr.Zero, IntPtr.Zero, IntPtr.Zero,
            sourceAddress, destinationAddress, requestData,
            (ushort)requestData.Length, IntPtr.Zero, replyBuffer,
            (uint)replyBuffer.Length, timeout);
        if (replies > 0)
        {
            status = BitConverter.ToUInt32(replyBuffer, 4);
            roundTripTime = BitConverter.ToUInt32(replyBuffer, 8);
            error = 0;
        }
        else
        {
            status = 0;
            roundTripTime = 0;
            error = Marshal.GetLastWin32Error();
        }
        return replies;
    }
}
'@
    Add-Type -TypeDefinition $source -Language CSharp -ErrorAction Stop
}

function Get-InterfaceProbeSource($Adapter, [string[]]$Addresses, [string]$Server) {
    try {
        $ipInterface = @(Get-NetIPInterface -InterfaceIndex $Adapter.ifIndex `
            -AddressFamily IPv4 -ErrorAction Stop)
    } catch {
        return [pscustomobject]@{ Address = $null; Error = "interface routing policy could not be queried: $($_.Exception.Message)" }
    }
    if ($ipInterface.Count -ne 1 -or [string]$ipInterface[0].WeakHostSend -ne 'Disabled') {
        return [pscustomobject]@{ Address = $null; Error = 'interface routing is not verifiably constrained (WeakHostSend must be Disabled)' }
    }
    $routeErrors = @()
    foreach ($address in $Addresses) {
        try {
            $routeResult = @(Find-NetRoute -RemoteIPAddress $Server -LocalIPAddress $address -ErrorAction Stop)
            $routes = @($routeResult | Where-Object {
                $_.PSObject.Properties.Name -contains 'DestinationPrefix'
            })
            $selectedAddress = @($routeResult | Where-Object {
                $_.PSObject.Properties.Name -contains 'IPAddress' -and
                [string]$_.IPAddress -eq $address -and
                $_.InterfaceIndex -eq $Adapter.ifIndex
            })
            if ($routes.Count -eq 1 -and $routes[0].InterfaceIndex -eq $Adapter.ifIndex -and
                $selectedAddress.Count -eq 1) {
                return [pscustomobject]@{ Address = $address; Error = $null }
            }
            $routeErrors += "route lookup did not select interface index $($Adapter.ifIndex) for $address"
        } catch {
            $detail = ($_.Exception.Message -replace '\s+', ' ').Trim()
            $routeErrors += "route lookup failed for $address`: $detail"
        }
    }
    return [pscustomobject]@{ Address = $null; Error = ($routeErrors -join '; ') }
}

function ConvertTo-Ipv4ApiAddress([string]$Address) {
    return [BitConverter]::ToUInt32(([Net.IPAddress]::Parse($Address).GetAddressBytes()), 0)
}

function Invoke-InterfaceIcmpProbe([string]$SourceAddress, [string]$Server) {
    $handle = [IntPtr]::Zero
    try {
        Initialize-IcmpApi
        $handle = [ConfigTimeSettingsIcmpApi]::IcmpCreateFile()
        if ($handle -eq [IntPtr]::Zero -or $handle -eq [IntPtr](-1)) {
            throw "IcmpCreateFile failed ($([Runtime.InteropServices.Marshal]::GetLastWin32Error()))"
        }
        $source = ConvertTo-Ipv4ApiAddress $SourceAddress
        $destination = ConvertTo-Ipv4ApiAddress $Server
        $roundTripTimes = @()
        $statuses = @()
        for ($attempt = 0; $attempt -lt 3; $attempt++) {
            [uint32]$rtt = 0
            [uint32]$status = 0
            [int]$errorCode = 0
            $replyCount = [ConfigTimeSettingsIcmpApi]::SendEcho(
                $handle, $source, $destination, [uint32]1000,
                [ref]$rtt, [ref]$status, [ref]$errorCode)
            if ($replyCount -gt 0 -and $status -eq 0) {
                $roundTripTimes += [double]$rtt
                $statuses += 'Reply'
            } elseif ($replyCount -gt 0) {
                $roundTripTimes += $null
                $statuses += "ICMP status $status"
            } else {
                $roundTripTimes += $null
                $statuses += if ($errorCode -eq 11010) { 'Timeout' } else { "ICMP error $errorCode" }
            }
        }
        return [pscustomobject]@{
            Measurable = $true
            RoundTripTimes = [double[]]@($roundTripTimes | Where-Object { $null -ne $_ })
            Statuses = [string[]]$statuses
            Error = $null
        }
    } catch {
        return [pscustomobject]@{
            Measurable = $false
            RoundTripTimes = [double[]]@()
            Statuses = [string[]]@()
            Error = $_.Exception.Message
        }
    } finally {
        if ($handle -ne [IntPtr]::Zero -and $handle -ne [IntPtr](-1)) {
            if (-not [ConfigTimeSettingsIcmpApi]::IcmpCloseHandle($handle)) {
                Write-Warning "IcmpCloseHandle failed ($([Runtime.InteropServices.Marshal]::GetLastWin32Error()))"
            }
        }
    }
}

function Test-DirectedBroadcast([string]$Address) {
    $target = [Net.IPAddress]::Parse($Address).GetAddressBytes()
    $targetValue = ([uint64]$target[0] -shl 24) -bor ([uint64]$target[1] -shl 16) -bor
        ([uint64]$target[2] -shl 8) -bor [uint64]$target[3]
    try {
        $localAddresses = @(Get-NetIPAddress -AddressFamily IPv4 -ErrorAction Stop |
            Where-Object { $_.PrefixLength -lt 31 })
    } catch { throw "local IPv4 address and subnet information could not be queried: $($_.Exception.Message)" }
    foreach ($local in $localAddresses) {
        $bytes = [Net.IPAddress]::Parse($local.IPAddress).GetAddressBytes()
        $localValue = ([uint64]$bytes[0] -shl 24) -bor ([uint64]$bytes[1] -shl 16) -bor
            ([uint64]$bytes[2] -shl 8) -bor [uint64]$bytes[3]
        $mask = [uint64]::MaxValue -shl (32 - [int]$local.PrefixLength)
        $mask = $mask -band [uint64]4294967295
        $network = $localValue -band $mask
        $broadcast = $network -bor ((-bnot $mask) -band [uint64]4294967295)
        if ($targetValue -eq $broadcast -or $targetValue -eq $network) { return $true }
    }
    return $false
}

function Test-CompetingClockControllers {
    $vmic = Get-RegistryValueState 'TimeProviders\VMICTimeProvider' 'Enabled'
    if ($vmic.present -and [string]$vmic.value -ne '0') {
        # Windows enables this Hyper-V guest provider by default; it only supplies time while vmictimesync runs in a VM.
        try { $vmicService = @(Get-CimInstance -ClassName Win32_Service -Filter "Name='vmictimesync'" -ErrorAction Stop) }
        catch { Stop-Config 2 "Hyper-V time synchronization service state could not be inspected: $($_.Exception.Message)" }
        if (@($vmicService | Where-Object { [string]$_.State -ne 'Stopped' }).Count -gt 0) {
            Stop-Config 2 'VMICTimeProvider is enabled and the Hyper-V Time Synchronization service (vmictimesync) is running; review competing clock-controller policy before changing W32Time'
        }
    }
    $providerRoot = [Microsoft.Win32.Registry]::LocalMachine.OpenSubKey(
        "$script:RegistryRoot\TimeProviders", $false)
    if ($null -ne $providerRoot) {
        try {
            foreach ($providerName in $providerRoot.GetSubKeyNames()) {
                if ($providerName -in @('NtpClient','PtpClient','VMICTimeProvider')) { continue }
                $candidate = $providerRoot.OpenSubKey($providerName, $false)
                try {
                    if ($null -ne $candidate -and
                        $candidate.GetValue('Enabled', 0) -ne 0 -and
                        $candidate.GetValue('InputProvider', 0) -ne 0) {
                        Stop-Config 2 "additional W32Time input provider '$providerName' is enabled; review its clock-control behavior before proceeding"
                    }
                } finally { if ($null -ne $candidate) { $candidate.Dispose() } }
            }
        } finally { $providerRoot.Dispose() }
    }
    try {
        $services = @(Get-CimInstance Win32_Service -ErrorAction Stop |
            Where-Object {
                $_.State -eq 'Running' -and $_.Name -ne 'W32Time' -and
                ($_.Name -match '(?i)chrony|ntpd|meinberg|nettime|ptp' -or
                    $_.DisplayName -match '(?i)chrony|ntp server|meinberg|precision time')
            })
    } catch { Stop-Config 2 "running clock-controller services could not be inspected: $($_.Exception.Message)" }
    if ($services.Count -gt 0) {
        Stop-Config 2 ("competing clock controller detected: " +
            (($services | ForEach-Object { $_.Name }) -join ', ') +
            '. Stop only after an administrator-designed review; this utility will not disable it.')
    }
}

function Test-IpInFirewallRange([string]$Address, [string]$Range) {
    if ($Range -in @('Any','0.0.0.0/0','::/0')) { return $true }
    $parts = $Range.Split('/')
    $network = $null
    $peer = $null
    if (-not [Net.IPAddress]::TryParse($parts[0], [ref]$network) -or
        -not [Net.IPAddress]::TryParse($Address, [ref]$peer) -or
        $network.AddressFamily -ne $peer.AddressFamily) { return $false }
    $bytes = $network.GetAddressBytes()
    $peerBytes = $peer.GetAddressBytes()
    $prefix = if ($parts.Count -eq 2) {
        $number = 0
        if (-not [int]::TryParse($parts[1], [ref]$number)) { return $false }
        $number
    } else { $bytes.Length * 8 }
    if ($prefix -lt 0 -or $prefix -gt ($bytes.Length * 8)) { return $false }
    for ($i = 0; $i -lt $bytes.Length; $i++) {
        $remaining = $prefix - (8 * $i)
        if ($remaining -le 0) { break }
        $mask = if ($remaining -ge 8) { 255 } else { (255 -shl (8 - $remaining)) -band 255 }
        if (($bytes[$i] -band $mask) -ne ($peerBytes[$i] -band $mask)) { return $false }
    }
    return $true
}

function Test-FirewallRuleAppliesToW32Time($Rule) {
    $application = $Rule | Get-NetFirewallApplicationFilter -ErrorAction Stop
    $program = [string]$application.Program
    if ($program -and $program -ne 'Any') {
        $expanded = [Environment]::ExpandEnvironmentVariables($program)
        $svchost = Join-Path $env:SystemRoot 'System32\svchost.exe'
        if ($expanded -ne $svchost -and $expanded -ne (Join-Path $env:SystemRoot 'Sysnative\svchost.exe')) { return $false }
    }
    $serviceFilter = $Rule | Get-NetFirewallServiceFilter -ErrorAction Stop
    $service = [string]$serviceFilter.Service
    return (-not $service -or $service -in @('Any','*') -or $service -eq 'W32Time')
}

function Get-ThirdPartyFirewallProducts {
    try {
        return @(Get-CimInstance -Namespace 'root/SecurityCenter2' -ClassName FirewallProduct -ErrorAction Stop |
            ForEach-Object { [string]$_.displayName } | Where-Object { $_ -and $_ -notmatch '(?i)Windows (Defender )?Firewall' })
    } catch { return @() }
}

function Get-ActiveBlockRules([hashtable]$Filter) {
    # Get-NetFirewallRule reports "no matching rules" as an ObjectNotFound error; that is an empty result.
    try { return @(Get-NetFirewallRule -PolicyStore ActiveStore -Enabled True -Action Block @Filter -ErrorAction Stop) }
    catch {
        if ($_.CategoryInfo.Category -eq [System.Management.Automation.ErrorCategory]::ObjectNotFound) { return @() }
        throw
    }
}

function Test-NtpFirewallPolicy([string[]]$Peers) {
    $profiles = @(Get-NetFirewallProfile -PolicyStore ActiveStore -ErrorAction Stop)
    if ($profiles.Count -eq 0) { Stop-Config 2 'effective firewall policy could not be determined' }
    if ($profiles | Where-Object { $_.DefaultOutboundAction -eq 'Block' }) {
        Stop-Config 2 'effective firewall policy blocks outbound traffic by default; approve a narrowly scoped UDP 123 rule with the administrator'
    }
    $blocks = @(Get-ActiveBlockRules @{ Direction = 'Outbound' })
    foreach ($rule in $blocks) {
        $port = $rule | Get-NetFirewallPortFilter -ErrorAction Stop
        if ($port.Protocol -notin @('UDP','17','Any')) { continue }
        if ($port.RemotePort -notcontains 'Any' -and $port.RemotePort -notcontains '123') { continue }
        if (-not (Test-FirewallRuleAppliesToW32Time $rule)) { continue }
        $address = $rule | Get-NetFirewallAddressFilter -ErrorAction Stop
        $remote = @($address.RemoteAddress)
        $matchesPeer = $false
        foreach ($peer in $Peers) {
            foreach ($range in $remote) {
                if (Test-IpInFirewallRange $peer ([string]$range)) { $matchesPeer = $true; break }
            }
            if ($matchesPeer) { break }
        }
        if ($matchesPeer) {
            Stop-Config 2 "effective outbound firewall block '$($rule.Name)' may block NTP UDP 123; no firewall policy was changed"
        }
    }
}

function Get-Inventory([string]$Server) {
    $valid = Test-Address $Server $true
    if ($null -eq $valid) { Stop-Config 1 "-inventory server address '$Server' must be an IPv4 unicast literal" }
    try {
        if (Test-DirectedBroadcast $valid.Text) { Stop-Config 1 "-inventory server address '$Server' is a local IPv4 network or broadcast address" }
    } catch { Stop-Config 2 "local IPv4 network validation failed: $($_.Exception.Message)" }
    $null = Test-Environment $true
    try { $adapters = @(Get-NetAdapter -IncludeHidden -ErrorAction Stop) }
    catch { Stop-Config 2 "network adapter enumeration failed: $($_.Exception.Message)" }
    try {
        $allIpv4 = @(Get-NetIPAddress -AddressFamily IPv4 -ErrorAction Stop)
    } catch {
        $allIpv4 = @()
        Write-Warning "IPv4 adapter enumeration failed: $($_.Exception.Message)"
    }
    $eligibleAdapters = @()
    foreach ($adapter in $adapters) {
        $addresses = @($allIpv4 | Where-Object {
            $_.InterfaceIndex -eq $adapter.ifIndex -and
            $_.IPAddress -notmatch '^(169\.254\.|0\.|127\.)'
        } | Select-Object -ExpandProperty IPAddress)
        try { $timestamp = Get-AdapterTimestampState $adapter }
        catch {
            $timestamp = [pscustomobject]@{
                Support = 'Unknown'; Active = 'Unknown'; Error = $_.Exception.Message
            }
        }
        $state = [string]$adapter.Status
        $wired = Test-PhysicalEthernetAdapter $adapter
        $probe = $null
        $probeReason = $null
        if ($state -eq 'Up' -and $addresses.Count -gt 0) {
            $source = Get-InterfaceProbeSource $adapter $addresses $valid.Text
            if ($null -eq $source.Address) {
                $probeReason = $source.Error
            } else {
                $probe = Invoke-InterfaceIcmpProbe $source.Address $valid.Text
                if (-not $probe.Measurable) { $probeReason = $probe.Error }
            }
        }
        $replies = if ($null -ne $probe -and $probe.Measurable) {
            @($probe.RoundTripTimes)
        } else { @() }
        $successCount = $replies.Count
        $packetLoss = [Math]::Round((3 - $successCount) * 100.0 / 3.0, 1)
        $meanRtt = if ($successCount -gt 0) {
            [double](($replies | Measure-Object -Average).Average)
        } else { $null }
        if ($state -ne 'Up') {
            $probeResult = "Not tested (not operationally up)"
            $probeReason = 'not operationally up'
        } elseif ($addresses.Count -eq 0) {
            $probeResult = 'Not tested (no usable IPv4 address)'
            $probeReason = 'no usable IPv4 address'
        } elseif ($null -ne $probeReason) {
            $probeResult = "Not measurable ($probeReason)"
        } else {
            $rttText = @()
            $replyIndex = 0
            foreach ($status in $probe.Statuses) {
                if ($status -eq 'Reply') {
                    $rttText += ('{0:F3} ms' -f $replies[$replyIndex])
                    $replyIndex++
                } else {
                    $rttText += [string]$status
                }
            }
            $probeResult = "RTTs: $($rttText -join ', '); successes: $successCount/3; packet loss: $packetLoss%"
            if ($successCount -eq 0) {
                $probeResult = "Unreachable ($probeResult)"
            } elseif ($successCount -lt 3) {
                $probeResult += ('; partial-sample mean RTT: {0:F3} ms' -f $meanRtt)
            } else {
                $probeResult += ('; mean RTT: {0:F3} ms' -f $meanRtt)
            }
        }
        $exclusionReasons = @()
        if ($state -ne 'Up') { $exclusionReasons += 'not operationally up' }
        if (-not $wired) {
            if (-not $adapter.HardwareInterface) { $exclusionReasons += 'virtual/software adapter' }
            else { $exclusionReasons += 'not physical wired Ethernet' }
        }
        if ($addresses.Count -eq 0) { $exclusionReasons += 'no usable IPv4 address' }
        if ($null -eq $probe -or -not $probe.Measurable) {
            if ($state -eq 'Up' -and $addresses.Count -gt 0) {
                $exclusionReasons += "probe not measurable: $probeReason"
            }
        } elseif ($successCount -ne 3) {
            $exclusionReasons += "probe succeeded $successCount/3 (3 required)"
        }
        $eligible = $state -eq 'Up' -and $wired -and $addresses.Count -gt 0 -and
            $null -ne $probe -and $probe.Measurable -and $successCount -eq 3
        if ($eligible) {
            $eligibleAdapters += [pscustomobject]@{
                Adapter = $adapter
                Support = [string]$timestamp.Support
                Active = [string]$timestamp.Active
                TimestampError = [string]$timestamp.Error
                MeanRtt = $meanRtt
                Guid = [string]$adapter.InterfaceGuid
                TimestampRank = if ([string]$timestamp.Support -eq 'Yes') { 0 } else { 1 }
            }
        }
        $eligibility = if ($eligible) {
            'Yes'
        } elseif ($exclusionReasons.Count -gt 0) {
            "No ($($exclusionReasons -join '; '))"
        } else {
            'No'
        }
        $queryDetail = if ($timestamp.Error) { "; timestamp query: $($timestamp.Error)" } else { '' }
        Write-Output ("Adapter: {0}; GUID: {1}; index: {2}; description: {3}; type: {4}; state: {5}; IPv4: {6}; timestamp support: {7}; active hardware timestamping: {8}; probe: {9}; eligibility: {10}{11}" -f
            $adapter.Name, $adapter.InterfaceGuid, $adapter.ifIndex, $adapter.InterfaceDescription,
            ("{0} (physical medium {1}; hardware={2})" -f $adapter.MediaType, $adapter.NdisPhysicalMedium, $adapter.HardwareInterface),
            $state, ($addresses -join ', '), $timestamp.Support, $timestamp.Active, $probeResult, $eligibility, $queryDetail)
    }
    $script:InventoryRecommendation = $null
    if ($eligibleAdapters.Count -eq 0) {
        Write-Host 'Recommendation: No eligible NIC (requires physical wired Ethernet, operationally up, usable IPv4, and three successful interface-verified ICMP replies).' -ForegroundColor Yellow
        return
    }
    $recommendation = $eligibleAdapters |
        Sort-Object @{ Expression = { $_.TimestampRank } },
            @{ Expression = { $_.MeanRtt } },
            @{ Expression = { $_.Guid } } |
        Select-Object -First 1
    $script:InventoryRecommendation = $recommendation
    Write-Host ("Recommendation: {0}; GUID: {1}; index: {2}; mean RTT: {3:F3} ms; timestamp support: {4}; active hardware timestamping: {5}" -f
        $recommendation.Adapter.Name, $recommendation.Guid, $recommendation.Adapter.ifIndex,
        $recommendation.MeanRtt, $recommendation.Support, $recommendation.Active) -ForegroundColor Green
    if ($recommendation.Support -eq 'No') {
        Write-Warning 'The recommended NIC lacks confirmed hardware timestamp support; software timestamping may reduce accuracy.'
    } elseif ($recommendation.Support -eq 'Unknown') {
        Write-Warning 'Hardware timestamp support for the recommended NIC remains unverified.'
    }
    if ($recommendation.Active -eq 'Unknown') {
        $detail = if ($recommendation.TimestampError) { " Query detail: $($recommendation.TimestampError)." } else { '' }
        Write-Warning "Active hardware timestamping for the recommended NIC remains unverified; the recommendation reflects supported capabilities only.$detail"
    } elseif ($recommendation.Active -eq 'No') {
        Write-Warning 'Windows does not report active hardware timestamping for the recommended NIC; the recommendation reflects supported capabilities only.'
    }
}

function Get-RouteInterfaceIndex([string]$Destination) {
    $result = @()
    try { $result = @(Find-NetRoute -RemoteIPAddress $Destination -ErrorAction Stop) } catch { }
    $routes = @($result | Where-Object { $_.PSObject.Properties.Name -contains 'DestinationPrefix' })
    if ($routes.Count -ge 1) { return [int]$routes[0].InterfaceIndex }
    $address = $null
    if (-not [Net.IPAddress]::TryParse($Destination, [ref]$address) -or
        $address.AddressFamily -ne [Net.Sockets.AddressFamily]::InterNetwork -or
        $address.GetAddressBytes()[0] -lt 224 -or $address.GetAddressBytes()[0] -gt 239) { return $null }
    # Find-NetRoute can fail for every multicast destination; mirror Windows route
    # selection (longest prefix, then lowest route + interface metric) instead.
    return Get-MulticastRouteInterfaceIndex $Destination
}

function Get-MulticastRouteInterfaceIndex([string]$Destination) {
    try {
        $interfaces = @(Get-NetIPInterface -AddressFamily IPv4 -ErrorAction Stop |
            Where-Object { [string]$_.ConnectionState -eq 'Connected' })
        $routes = @(Get-NetRoute -AddressFamily IPv4 -ErrorAction Stop)
    } catch { return $null }
    $candidates = @()
    foreach ($route in $routes) {
        $interface = @($interfaces | Where-Object { $_.InterfaceIndex -eq $route.InterfaceIndex })[0]
        if ($null -eq $interface) { continue }
        $prefix = [string]$route.DestinationPrefix
        if (-not (Test-IpInFirewallRange $Destination $prefix)) { continue }
        $candidates += [pscustomobject]@{
            InterfaceIndex = [int]$route.InterfaceIndex
            PrefixLength = [int]($prefix.Split('/')[1])
            Metric = [int]$route.RouteMetric + [int]$interface.InterfaceMetric
        }
    }
    if ($candidates.Count -lt 1) { return $null }
    $longest = ($candidates | Measure-Object -Property PrefixLength -Maximum).Maximum
    $best = @($candidates | Where-Object { $_.PrefixLength -eq $longest })
    $lowest = ($best | Measure-Object -Property Metric -Minimum).Minimum
    $winners = @($best | Where-Object { $_.Metric -eq $lowest } | ForEach-Object { $_.InterfaceIndex } | Sort-Object -Unique)
    if ($winners.Count -ne 1) { return $null }
    return [int]$winners[0]
}

function Get-PtpReadiness([string]$Server, [bool]$Probe = $true) {
    $adapters = @(Get-NetAdapter -IncludeHidden -ErrorAction Stop)
    $allIpv4 = @(Get-NetIPAddress -AddressFamily IPv4 -ErrorAction Stop)
    $serverRoute = Get-RouteInterfaceIndex $Server
    $multicastRoute = Get-RouteInterfaceIndex $script:PtpMulticastAddress
    $defaultRouteIndexes = @()
    try {
        $defaultRouteIndexes = @(Get-NetRoute -DestinationPrefix '0.0.0.0/0' -AddressFamily IPv4 -ErrorAction Stop |
            ForEach-Object { [int]$_.InterfaceIndex })
    } catch { }
    $wiredEntries = @()
    $others = @()
    $problems = @()
    $warnings = @()
    foreach ($adapter in $adapters) {
        if ([string]$adapter.Status -ne 'Up') { continue }
        $ipv4 = @($allIpv4 | Where-Object {
            $_.InterfaceIndex -eq $adapter.ifIndex -and [string]$_.IPAddress -notmatch '^(169\.254\.|0\.|127\.)'
        })
        $onSubnet = $false
        foreach ($address in $ipv4) {
            if (Test-IpInFirewallRange $Server ('{0}/{1}' -f $address.IPAddress, $address.PrefixLength)) { $onSubnet = $true }
        }
        $routesServer = ($null -ne $serverRoute -and $serverRoute -eq [int]$adapter.ifIndex)
        $routesMulticast = ($null -ne $multicastRoute -and $multicastRoute -eq [int]$adapter.ifIndex)
        if (-not ($onSubnet -or $routesServer -or $routesMulticast)) { continue }
        try { $timestamp = Get-AdapterTimestampState $adapter }
        catch { $timestamp = [pscustomobject]@{ Support = 'Unknown'; Active = 'Unknown'; Error = $_.Exception.Message } }
        $wired = Test-PhysicalEthernetAdapter $adapter
        $paths = @()
        if ($onSubnet) { $paths += "on the server's IPv4 subnet" }
        if ($routesServer) { $paths += "Windows routes $Server through it" }
        if ($routesMulticast) { $paths += "Windows routes PTP multicast $($script:PtpMulticastAddress) through it" }
        $entry = [pscustomobject]@{
            Adapter = $adapter
            Addresses = [string[]]@($ipv4 | ForEach-Object { [string]$_.IPAddress })
            Support = [string]$timestamp.Support
            Active = [string]$timestamp.Active
            TimestampError = [string]$timestamp.Error
            Paths = [string[]]$paths
            DefaultRoute = ($defaultRouteIndexes -contains [int]$adapter.ifIndex)
            Reason = $null
        }
        if ($wired -and $onSubnet) {
            $wiredEntries += $entry
            continue
        }
        $entry.Reason = if (-not $wired) {
            if (-not $adapter.HardwareInterface) { 'virtual/software adapter' } else { 'not physical wired Ethernet' }
        } else {
            "not on the server's IPv4 subnet"
        }
        $others += $entry
    }
    # Windows PTP picks its own adapter; prefer hardware timestamp support, then the server route.
    $wiredEntries = @($wiredEntries | Sort-Object `
        @{ Expression = { if ($_.Support -eq 'Yes') { 0 } else { 1 } } },
        @{ Expression = { if ($null -ne $serverRoute -and [int]$_.Adapter.ifIndex -eq $serverRoute) { 0 } else { 1 } } },
        @{ Expression = { [int]$_.Adapter.ifIndex } })
    $candidate = $null
    if ($wiredEntries.Count -eq 0) {
        $problems += "No up, physical wired Ethernet adapter is on the IPv4 subnet of $Server."
    } else {
        $candidate = $wiredEntries[0]
        $wiredIndexes = @($wiredEntries | ForEach-Object { [int]$_.Adapter.ifIndex })
        if ($null -eq $serverRoute) {
            $warnings += "Windows route lookup for $Server failed; PTP setup can still proceed."
        } elseif ($wiredIndexes -notcontains $serverRoute) {
            $warnings += "Windows routes $Server through interface index $serverRoute, which is not a wired Ethernet adapter on the server's subnet; Windows PTP may use that path."
        }
    }
    $ready = ($null -ne $candidate -and $problems.Count -eq 0)
    $reachability = $null
    if ($ready -and $Probe) {
        $source = Get-InterfaceProbeSource $candidate.Adapter $candidate.Addresses $Server
        if ($null -eq $source.Address) {
            $reachability = [pscustomobject]@{ Successes = 0; Text = "not measurable ($($source.Error))" }
        } else {
            $result = Invoke-InterfaceIcmpProbe $source.Address $Server
            if (-not $result.Measurable) {
                $reachability = [pscustomobject]@{ Successes = 0; Text = "not measurable ($($result.Error))" }
            } else {
                $replies = @($result.RoundTripTimes)
                $text = "ping $($replies.Count)/3 replies"
                if ($replies.Count -gt 0) {
                    $text += ('; mean RTT: {0:F3} ms' -f [double](($replies | Measure-Object -Average).Average))
                }
                $reachability = [pscustomobject]@{ Successes = $replies.Count; Text = $text }
            }
        }
    }
    return [pscustomobject]@{
        Server = $Server
        Ready = $ready
        Candidate = $candidate
        Wired = $wiredEntries
        Others = $others
        Problems = $problems
        Warnings = $warnings
        Reachability = $reachability
    }
}

function Write-PtpReadiness($Readiness) {
    $server = $Readiness.Server
    Write-Output ''
    Write-Output "PTP readiness for $server (Windows PTP needs a wired Ethernet adapter on the server's subnet; it chooses the adapter itself):"
    foreach ($entry in $Readiness.Wired) {
        $isCandidate = ([string]$entry.Adapter.InterfaceGuid -eq [string]$Readiness.Candidate.Adapter.InterfaceGuid)
        $tag = if ($isCandidate) { '[PTP adapter - expected]' } else { '[PTP adapter - alternate]' }
        $color = if ($isCandidate) { 'Green' } else { 'Yellow' }
        Write-Host ('  {0} {1}; GUID: {2}; index: {3}; IPv4: {4}; hardware timestamp support: {5}; active hardware timestamping: {6}; path: {7}' -f
            $tag, $entry.Adapter.Name, $entry.Adapter.InterfaceGuid, $entry.Adapter.ifIndex, ($entry.Addresses -join ', '),
            $entry.Support, $entry.Active, ($entry.Paths -join '; ')) -ForegroundColor $color
    }
    foreach ($entry in $Readiness.Others) {
        Write-Host ('  [Other path] {0}; GUID: {1}; index: {2}; IPv4: {3}; hardware timestamp support: {4}; reason: {5}; path: {6}' -f
            $entry.Adapter.Name, $entry.Adapter.InterfaceGuid, $entry.Adapter.ifIndex, ($entry.Addresses -join ', '),
            $entry.Support, $entry.Reason, ($entry.Paths -join '; ')) -ForegroundColor Yellow
    }
    if ($Readiness.Wired.Count -gt 1 -or $Readiness.Others.Count -gt 0) {
        Write-Warning ("More than one adapter connects this PC to $server or to PTP multicast. This is allowed: Windows PTP listens on all adapters " +
            'and picks one itself. The script expects the [PTP adapter - expected] one, allows PTP on every [PTP adapter] in the firewall, ' +
            'and reports the adapter Windows actually chose after setup. Leaving only one adapter enabled on this network gives the most predictable result.')
    }
    foreach ($warning in $Readiness.Warnings) { Write-Warning $warning }
    foreach ($problem in $Readiness.Problems) { Write-Warning $problem }
    if (-not $Readiness.Ready) {
        Write-Host 'PTP readiness: Not ready - resolve the items above, then rerun.' -ForegroundColor Yellow
        return
    }
    $candidate = $Readiness.Candidate
    if ($null -ne $Readiness.Reachability) {
        Write-Output "  Reachability through '$($candidate.Adapter.Name)': $($Readiness.Reachability.Text)"
        if ($Readiness.Reachability.Successes -lt 3) {
            Write-Warning "$server did not answer every ping through '$($candidate.Adapter.Name)'. ICMP may be filtered; PTP setup can still proceed, but reachability is unconfirmed."
        }
    }
    if ($candidate.Active -ne 'Yes') {
        $detail = if ($candidate.TimestampError) { " ($($candidate.TimestampError))" } else { '' }
        Write-Warning "Windows does not confirm that hardware timestamping is active on '$($candidate.Adapter.Name)' (active: $($candidate.Active))$detail. Hardware timestamping is not required; Windows uses software timestamps instead, so PTP accuracy may be close to NTP."
    }
    Write-Host ("PTP readiness: Ready - expected PTP adapter '{0}' for {1}." -f $candidate.Adapter.Name, $server) -ForegroundColor Green
}

function Get-PtpProviderPath {
    return (Join-Path $env:SystemRoot 'System32\ptpprov.dll')
}

function Test-PtpProviderInstalled {
    return (Test-Path -LiteralPath (Get-PtpProviderPath) -PathType Leaf)
}

function Get-PtpRegistryStates([string]$Server) {
    $ptp = 'TimeProviders\PtpClient'
    return @(
        [pscustomobject]@{ path = $ptp; name = 'PtpMasters'; present = $true; kind = 'String'; value = $Server }
        [pscustomobject]@{ path = $ptp; name = 'DllName'; present = $true; kind = 'String'; value = (Get-PtpProviderPath) }
        [pscustomobject]@{ path = $ptp; name = 'Enabled'; present = $true; kind = 'DWord'; value = 1 }
        [pscustomobject]@{ path = $ptp; name = 'InputProvider'; present = $true; kind = 'DWord'; value = 1 }
        [pscustomobject]@{ path = $ptp; name = 'DelayPollInterval'; present = $true; kind = 'DWord'; value = 16000 }
        [pscustomobject]@{ path = $ptp; name = 'AnnounceInterval'; present = $true; kind = 'DWord'; value = 4000 }
        [pscustomobject]@{ path = $ptp; name = 'EnableMulticastRx'; present = $true; kind = 'DWord'; value = 1 }
        [pscustomobject]@{ path = $ptp; name = 'EnableMulticastTx'; present = $true; kind = 'DWord'; value = 1 }
        [pscustomobject]@{ path = 'TimeProviders\NtpClient'; name = 'Enabled'; present = $true; kind = 'DWord'; value = 0 }
    ) + @(Get-ClockDisciplineStates 'PTP')
}

function Get-W32TimeFirewallService {
    try {
        $result = Invoke-External "$env:SystemRoot\System32\sc.exe" @('qsidtype','W32Time')
        if ($result.ExitCode -eq 0 -and $result.Output -match '(?im)SERVICE_SID_TYPE:\s*(UNRESTRICTED|RESTRICTED)\s*$') {
            return 'W32Time'
        }
    } catch { }
    Write-Warning 'W32Time has no service SID; the PTP firewall rules cannot be restricted to the Windows Time service and are scoped by adapter, address, port, and profile only.'
    return ''
}

function Get-PtpFirewallRuleDefinitions([string]$Server, [string[]]$InterfaceAlias, [string]$Service) {
    $description = 'configTimeSettings ownership marker: ESP32TimeServer PTP over UDP 319/320 for Windows Time'
    $common = @{
        description = $description; group = 'ESP32TimeServer-configTimeSettings'; action = 'Allow'
        enabled = 'True'; profile = 'Private'; protocol = 'UDP'; localPort = @('319','320')
        localAddress = @('Any'); interfaceAlias = [string[]]@($InterfaceAlias | Sort-Object -Unique); application = ''; service = $Service
    }
    $inbound = [pscustomobject]($common + @{
        name = $script:OwnedRuleNames[0]; displayName = 'ESP32TimeServer PTP inbound (configTimeSettings)'
        direction = 'Inbound'; remotePort = @('Any'); remoteAddress = @($Server)
    })
    $outbound = [pscustomobject]($common + @{
        name = $script:OwnedRuleNames[1]; displayName = 'ESP32TimeServer PTP outbound (configTimeSettings)'
        direction = 'Outbound'; remotePort = @('319','320'); remoteAddress = @($Server, $script:PtpMulticastAddress)
    })
    return @($inbound, $outbound)
}

function Test-PtpFirewallRules($ExpectedRules) {
    $actualRules = @(Get-OwnedFirewallState)
    if ($actualRules.Count -ne $ExpectedRules.Count) { return $false }
    foreach ($expected in $ExpectedRules) {
        $actual = @($actualRules | Where-Object { $_.name -ceq $expected.name })
        if ($actual.Count -ne 1) { return $false }
        foreach ($field in @('displayName','group','direction','action','enabled','profile','protocol')) {
            if ([string]$actual[0][$field] -ne [string]$expected.$field) { return $false }
        }
        foreach ($field in @('application','service')) {
            $left = if ([string]::IsNullOrEmpty([string]$actual[0][$field])) { 'Any' } else { [string]$actual[0][$field] }
            $right = if ([string]::IsNullOrEmpty([string]$expected.$field)) { 'Any' } else { [string]$expected.$field }
            if ($left -ne $right) { return $false }
        }
        foreach ($field in @('localPort','remotePort','localAddress','remoteAddress','interfaceAlias')) {
            $left = @($actual[0][$field] | ForEach-Object { [string]$_ } | Sort-Object)
            $right = @($expected.$field | ForEach-Object { [string]$_ } | Sort-Object)
            if (($left -join ',') -ne ($right -join ',')) { return $false }
        }
    }
    return $true
}

function Test-PtpFirewallPolicy([string]$Server) {
    $profiles = @(Get-NetFirewallProfile -PolicyStore ActiveStore -ErrorAction Stop)
    $private = @($profiles | Where-Object { [string]$_.Name -eq 'Private' })
    if ($private.Count -ne 1) { throw 'the effective Private firewall profile could not be determined' }
    if ([string]$private[0].AllowLocalFirewallRules -eq 'False') {
        throw 'firewall policy ignores locally created rules on the Private profile; approve PTP UDP 319/320 rules with the administrator'
    }
    $blocks = @(Get-ActiveBlockRules @{})
    foreach ($rule in $blocks) {
        if ([string]$rule.Profile -notmatch 'Any|Private') { continue }
        $port = $rule | Get-NetFirewallPortFilter -ErrorAction Stop
        if ($port.Protocol -notin @('UDP','17','Any')) { continue }
        $ports = @($port.LocalPort) + @($port.RemotePort)
        if (-not ($ports -contains 'Any' -or $ports -contains '319' -or $ports -contains '320')) { continue }
        if (-not (Test-FirewallRuleAppliesToW32Time $rule)) { continue }
        $address = $rule | Get-NetFirewallAddressFilter -ErrorAction Stop
        foreach ($range in @($address.RemoteAddress)) {
            if ((Test-IpInFirewallRange $Server ([string]$range)) -or
                ([string]$rule.Direction -eq 'Outbound' -and (Test-IpInFirewallRange $script:PtpMulticastAddress ([string]$range)))) {
                throw "effective firewall block rule '$($rule.Name)' may block PTP UDP 319/320; no firewall policy was changed"
            }
        }
    }
}

function Confirm-PtpPrerequisites([string]$Server) {
    # Write-Host keeps prompts out of this function's boolean return value.
    Write-Host "PTP prerequisites: ESP32TimeServer at $Server must serve unsigned PTPv2 over UDP/IPv4 (PTP_L3_SERVER_ENABLED 1): default profile, domain 0, two-step E2E with multicast Delay_Req, ports 319/320, multicast $($script:PtpMulticastAddress), and valid UTC metadata after GNSS/PPS qualification."
    Write-Host 'Confirm these prerequisites and continue with PTP configuration? (y/N)'
    $answer = $null
    try {
        if ([Console]::IsInputRedirected) { $answer = $null }
        else { $answer = [Console]::ReadLine() }
    } catch { $answer = $null }
    while ($null -ne $answer -and $answer -notmatch '^(?i:y|yes|n|no)?$') {
        Write-Host 'Please enter Y/Yes or N/No.'
        try { $answer = [Console]::ReadLine() } catch { $answer = $null }
    }
    return ($null -ne $answer -and $answer -match '^(?i:y|yes)$')
}

function Get-PtpChosenInterfaceIndex([string]$Server, [DateTime]$SinceUtc) {
    # PTP-Operational event 513 names the local IfIndex on which Windows chose the master.
    try {
        $events = @(Get-WinEvent -FilterHashtable @{
            LogName = $script:PtpOperationalLog; Id = 513; StartTime = $SinceUtc.ToLocalTime()
        } -ErrorAction Stop)
    } catch { return $null }
    $serverPattern = '(?<![\d.])' + [regex]::Escape($Server) + '(?![\d.])'
    foreach ($event in $events) {
        $message = [string]$event.Message
        if ($message -notmatch $serverPattern) { continue }
        $match = [regex]::Match($message, 'IfIndex:\s*(\d+)')
        if ($match.Success) { return [int]$match.Groups[1].Value }
    }
    return $null
}

function Test-PtpNicSelection($Candidate, [string]$Server, [DateTime]$SinceUtc) {
    $expectedIndex = [int]$Candidate.Adapter.ifIndex
    $chosen = Get-PtpChosenInterfaceIndex $Server $SinceUtc
    if ($null -eq $chosen) {
        return [pscustomobject]@{ State = 'Unknown'; Detail = "Windows did not log which adapter it chose (PTP-Operational event 513); expected '$($Candidate.Adapter.Name)' (IfIndex $expectedIndex)" }
    }
    if ($chosen -eq $expectedIndex) {
        return [pscustomobject]@{ State = 'Verified'; Detail = "Windows PTP chose '$($Candidate.Adapter.Name)' (IfIndex $chosen), as expected" }
    }
    $name = $null
    try { $name = [string](Get-NetAdapter -InterfaceIndex $chosen -IncludeHidden -ErrorAction Stop).Name } catch { }
    $label = if ($name) { "'$name' (IfIndex $chosen)" } else { "IfIndex $chosen" }
    return [pscustomobject]@{ State = 'Other'; Detail = "Windows PTP chose $label, not the expected '$($Candidate.Adapter.Name)' (IfIndex $expectedIndex)" }
}

function Set-PtpConfiguration([System.Collections.IDictionary]$Arguments) {
    if (-not (Test-Administrator)) { Stop-Config 2 '-set PTP requires an elevated administrator token' }
    Enter-MachineLock
    try {
        Test-CurrentDirectory
        $null = Test-Environment
        $server = Test-Address ([string]$Arguments['serveraddress']) $true
        if ($null -eq $server) { Stop-Config 1 "PTP server '$($Arguments['serveraddress'])' must be an IPv4 unicast literal" }
        try {
            if (Test-DirectedBroadcast $server.Text) { Stop-Config 1 "server '$($server.Text)' is a local IPv4 network or broadcast address" }
        } catch { Stop-Config 2 "local IPv4 network validation failed: $($_.Exception.Message)" }
        try { $null = Get-Service -Name W32Time -ErrorAction Stop }
        catch { Stop-Config 2 "W32Time service could not be queried: $($_.Exception.Message)" }
        Test-CompetingClockControllers
        if (-not (Test-PtpProviderInstalled)) {
            Stop-Config 2 "the Windows PTP provider '$(Get-PtpProviderPath)' is not installed; no changes were made"
        }
        try { $readiness = Get-PtpReadiness $server.Text $true }
        catch { Stop-Config 2 "PTP readiness could not be assessed: $($_.Exception.Message)" }
        Write-PtpReadiness $readiness
        if (-not $readiness.Ready) {
            Stop-Config 3 'PTP readiness check failed; no backup or configuration changes were made. Resolve the items listed above and rerun.'
        }
        $candidate = $readiness.Candidate
        $adapter = $candidate.Adapter
        try { $profiles = @(Get-NetConnectionProfile -InterfaceIndex $adapter.ifIndex -ErrorAction Stop) }
        catch { Stop-Config 2 "network profile for '$($adapter.Name)' could not be queried: $($_.Exception.Message)" }
        if ($profiles.Count -ne 1 -or [string]$profiles[0].NetworkCategory -ne 'Private') {
            Stop-Config 2 "the network profile for '$($adapter.Name)' must be Private; this utility does not change network category. No changes were made."
        }
        $ptpAdapterNames = @($readiness.Wired | ForEach-Object { [string]$_.Adapter.Name })
        foreach ($entry in @($readiness.Wired | Where-Object { [int]$_.Adapter.ifIndex -ne [int]$adapter.ifIndex })) {
            $category = $null
            try { $category = [string](@(Get-NetConnectionProfile -InterfaceIndex $entry.Adapter.ifIndex -ErrorAction Stop)[0].NetworkCategory) } catch { }
            if ($category -ne 'Private') {
                Write-Warning "The network profile for alternate PTP adapter '$($entry.Adapter.Name)' is not Private; the PTP firewall rules do not apply to it if Windows chooses it."
            }
        }
        try { Test-PtpFirewallPolicy $server.Text }
        catch { Stop-Config 2 "PTP firewall policy check failed: $($_.Exception.Message)" }
        foreach ($product in @(Get-ThirdPartyFirewallProducts)) {
            Write-Warning ("$product is registered as a firewall. This script only creates Windows Defender Firewall rules; " +
                "make sure $product allows Windows Time (svchost.exe) UDP 319/320 to and from $($server.Text) and multicast $($script:PtpMulticastAddress), or PTP may never synchronize.")
        }
        $firewallService = Get-W32TimeFirewallService
        $rules = Get-PtpFirewallRuleDefinitions $server.Text $ptpAdapterNames $firewallService
        $desired = Get-PtpRegistryStates $server.Text
        Write-Output 'Mode: PTP (native W32Time PTP client)'
        Write-Output "PTP master: $($server.Text); expected NIC: $($adapter.Name) ($($adapter.InterfaceGuid)); Windows chooses the adapter itself and the script reports its choice after the restart"
        Write-Output "Intended changes: configure and enable PtpClient (multicast receive/transmit), disable NtpClient while keeping its peer list, create two utility-owned UDP 319/320 firewall rules for the server on PTP adapter(s) $(($ptpAdapterNames | ForEach-Object { "'$_'" }) -join ', '), set W32Time startup to Automatic (non-delayed), and apply the PTP clock-discipline profile: $(Get-ClockDisciplineDescription 'PTP')."
        Write-Warning 'Native Windows PTP is unauthenticated: PtpMasters and firewall address scoping are not authentication. Use a trusted network.'
        Write-Warning 'There is no automatic NTP fallback. If the ESP32 stops serving qualified PTP time (it has no PTP holdover), Windows stops receiving time updates until PTP returns or you restore/set NTP.'
        Write-Warning 'If you add or rename a network adapter on this network later, rerun -set PTP so the firewall rules include it. W32Time may step or slew the system clock.'
        if (-not (Confirm-PtpPrerequisites $server.Text)) {
            Stop-Config 2 'PTP prerequisites were not acknowledged; no backup or configuration changes were made'
        }
        $script:SelectedNic = $adapter
        $script:ExpectedNtpPeers = @()
        $script:ExpectedPtpMaster = $server.Text
        $script:PtpCandidate = $candidate
        $backup = Write-Backup $true
        $before = Read-BackupFile $backup
        try {
            foreach ($state in $desired) { Set-RegistryValueState $state }
            Restore-FirewallRules $rules
            Set-ServiceAutomatic
            foreach ($state in $desired) {
                $actual = Get-RegistryValueState $state.path $state.name
                if (-not $actual.present -or [string]$actual.kind -ne $state.kind -or [string]$actual.value -ne [string]$state.value) {
                    throw "PTP registry readback failed for '$($state.path)\$($state.name)'"
                }
            }
            if (-not (Test-PtpFirewallRules $rules)) { throw 'PTP firewall rule readback did not match the requested rules' }
            Test-PtpFirewallPolicy $server.Text
            $null = Test-Environment
        } catch {
            $original = $_.Exception.Message
            try { Restore-Payload $before; Stop-Config 5 "PTP apply failed: $original; configuration rolled back. Safety backup: '$backup'" }
            catch { Stop-Config 7 "PTP apply failed: $original; rollback failed: $($_.Exception.Message); safety backup: '$backup'" }
        }
        Write-Output "Configuration saved for PTP master $($server.Text) on '$($adapter.Name)'."
        Write-Output "Safety backup: $backup"
        Confirm-Restart $backup
    } finally { Exit-MachineLock }
}

function Test-Preflight([bool]$IsPtp, [string[]]$Servers) {
    if (-not (Test-Administrator)) { Stop-Config 2 'this operation requires an elevated administrator token' }
}

function ConvertTo-WindowsArgument([string]$Value) {
    if ($Value.Length -gt 0 -and $Value -notmatch '[\s"]') { return $Value }
    $builder = New-Object Text.StringBuilder
    [void]$builder.Append('"')
    $slashes = 0
    foreach ($character in $Value.ToCharArray()) {
        if ($character -eq '\') { $slashes++; continue }
        if ($character -eq '"') {
            [void]$builder.Append(('\' * (2 * $slashes + 1)))
            [void]$builder.Append('"')
            $slashes = 0
            continue
        }
        if ($slashes -gt 0) { [void]$builder.Append(('\' * $slashes)); $slashes = 0 }
        [void]$builder.Append($character)
    }
    if ($slashes -gt 0) { [void]$builder.Append(('\' * (2 * $slashes))) }
    [void]$builder.Append('"')
    return $builder.ToString()
}

function Invoke-External([string]$FileName, [string[]]$Arguments, [int]$TimeoutSeconds = 30) {
    $start = New-Object Diagnostics.ProcessStartInfo
    $start.FileName = $FileName
    $start.Arguments = (@($Arguments | ForEach-Object { ConvertTo-WindowsArgument ([string]$_) }) -join ' ')
    $start.UseShellExecute = $false
    $start.CreateNoWindow = $true
    $start.RedirectStandardOutput = $true
    $start.RedirectStandardError = $true
    $process = New-Object Diagnostics.Process
    $process.StartInfo = $start
    if (-not $process.Start()) { throw "could not start '$FileName'" }
    $stdout = $process.StandardOutput.ReadToEndAsync()
    $stderr = $process.StandardError.ReadToEndAsync()
    if (-not $process.WaitForExit($TimeoutSeconds * 1000)) {
        try { $process.Kill(); $null = $process.WaitForExit(2000) } catch { }
        throw "external command '$FileName' exceeded the $TimeoutSeconds second timeout"
    }
    return [pscustomobject]@{ ExitCode = $process.ExitCode; Output = $stdout.Result; Error = $stderr.Result }
}

function Set-W32TimeRunningState([ValidateSet('Running','Stopped')][string]$State) {
    $controller = New-Object ServiceProcess.ServiceController('W32Time')
    try {
        $controller.Refresh()
        if ($State -eq 'Running' -and $controller.Status -ne [ServiceProcess.ServiceControllerStatus]::Running) {
            $controller.Start()
            $controller.WaitForStatus([ServiceProcess.ServiceControllerStatus]::Running, [TimeSpan]::FromSeconds(30))
        } elseif ($State -eq 'Stopped' -and $controller.Status -ne [ServiceProcess.ServiceControllerStatus]::Stopped) {
            $controller.Stop()
            $controller.WaitForStatus([ServiceProcess.ServiceControllerStatus]::Stopped, [TimeSpan]::FromSeconds(30))
        }
    } finally { $controller.Dispose() }
}

function Set-ServiceAutomatic {
    $result = Invoke-External "$env:SystemRoot\System32\sc.exe" @('config','W32Time','start=','auto')
    if ($result.ExitCode -ne 0) { throw "sc.exe config failed ($($result.ExitCode)): $($result.Output) $($result.Error)" }
    $key = [Microsoft.Win32.Registry]::LocalMachine.OpenSubKey('SYSTEM\CurrentControlSet\Services\W32Time', $true)
    try { $key.DeleteValue('DelayedAutoStart', $false) } finally { $key.Dispose() }
    $service = Get-CimInstance -ClassName Win32_Service -Filter "Name='W32Time'" -ErrorAction Stop
    $verifyKey = [Microsoft.Win32.Registry]::LocalMachine.OpenSubKey('SYSTEM\CurrentControlSet\Services\W32Time', $false)
    try {
        if ($service.StartMode -ne 'Auto' -or $verifyKey.GetValueNames() -contains 'DelayedAutoStart') {
            throw 'W32Time Automatic non-delayed startup readback failed'
        }
    } finally { $verifyKey.Dispose() }
}

function Get-NtpPeerTokens([object[]]$Peers) {
    for ($i = 0; $i -lt $Peers.Count; $i++) {
        $flag = if ($Peers.Count -eq 2 -and $i -eq 1) { '0xA' } else { '0x8' }
        $Peers[$i].Text + ',' + $flag
    }
}

function Set-NtpConfiguration([System.Collections.IDictionary]$Arguments) {
    Test-Preflight $false @()
    Enter-MachineLock
    try {
        Test-CurrentDirectory
        $null = Test-Environment
        try { $timeService = Get-Service -Name W32Time -ErrorAction Stop }
        catch { Stop-Config 2 "W32Time service could not be queried: $($_.Exception.Message)" }
        if ($null -eq $timeService) { Stop-Config 2 'W32Time service does not exist' }
        Test-CompetingClockControllers
        $peers = @()
        for ($n = 1; $n -le 4; $n++) {
            if (-not $Arguments.Contains("serveraddress$n")) { continue }
            $address = Test-Address ([string]$Arguments["serveraddress$n"])
            if ($null -eq $address) {
                Stop-Config 1 "-serveraddress$n '$($Arguments["serveraddress$n"])' is not an allowed literal unicast address"
            }
            if ($peers.Normalized -contains $address.Normalized) {
                Stop-Config 1 "duplicate NTP peer '$($Arguments["serveraddress$n"])' after address normalization"
            }
            $peers += $address
        }
        $peerText = @(Get-NtpPeerTokens $peers)
        $script:ExpectedNtpPeers = @($peers | ForEach-Object { $_.Text })
        $script:ExpectedPtpMaster = $null
        $script:PtpCandidate = $null
        $script:SelectedNic = $null
        try { Test-NtpFirewallPolicy @($peers | ForEach-Object { $_.Text }) }
        catch { Stop-Config 2 "effective firewall policy could not be checked: $($_.Exception.Message)" }
        foreach ($peer in $peers) {
            if ($peer.Family -eq 'IPv4') {
                try {
                    if (Test-DirectedBroadcast $peer.Text) {
                        Stop-Config 1 "server '$($peer.Text)' is a local IPv4 network or broadcast address"
                    }
                } catch { Stop-Config 2 "local IPv4 network validation failed: $($_.Exception.Message)" }
            }
            if (-not (Test-Ping $peer.Text)) {
                Stop-Config 3 "server '$($peer.Text)' did not answer any of three 1000 ms ICMP Echo requests; ICMP may be filtered even when time traffic works"
            }
        }
        Write-Output 'Mode: NTP (manual W32Time input)'
        Write-Output "Peers and W32Time flags: $($peerText -join ', ')"
        Write-Output "Intended changes: configure unsigned NTP peers, enable NtpClient, disable PtpClient if present, set W32Time startup to Automatic (non-delayed), remove only utility-owned PTP firewall rules, and apply the NTP clock-discipline profile (Windows standalone defaults): $(Get-ClockDisciplineDescription 'NTP')."
        Write-Output 'ICMP success confirms IP reachability only, not NTP availability, source quality, or synchronization.'
        Write-Warning 'Native Windows NTP is unsigned; the server must permit unsigned NTP. W32Time may step or slew the system clock.'
        $backup = Write-Backup $true
        $before = Read-BackupFile $backup
        try {
            $argsForW32tm = @('/config', ('/manualpeerlist:' + ($peerText -join ' ')), '/syncfromflags:manual')
            $output = Invoke-External "$env:SystemRoot\System32\w32tm.exe" $argsForW32tm
            if ($output.ExitCode -ne 0) { throw "w32tm /config failed ($($output.ExitCode)): $($output.Output) $($output.Error)" }
            Set-RegistryValueState ([pscustomobject]@{ path='Parameters'; name='Type'; present=$true; kind='String'; value='NTP' })
            Set-RegistryValueState ([pscustomobject]@{ path='TimeProviders\NtpClient'; name='Enabled'; present=$true; kind='DWord'; value=1 })
            Set-RegistryValueState ([pscustomobject]@{ path='TimeProviders\NtpClient'; name='InputProvider'; present=$true; kind='DWord'; value=1 })
            $discipline = @(Get-ClockDisciplineStates 'NTP')
            foreach ($state in $discipline) { Set-RegistryValueState $state }
            if (Test-Path 'HKLM:\SYSTEM\CurrentControlSet\Services\W32Time\TimeProviders\PtpClient') {
                Set-RegistryValueState ([pscustomobject]@{ path='TimeProviders\PtpClient'; name='Enabled'; present=$true; kind='DWord'; value=0 })
            }
            foreach ($name in $script:OwnedRuleNames) {
                Remove-OwnedFirewallRule $name
            }
            if (@(Get-OwnedFirewallState).Count -ne 0) {
                throw 'utility-owned PTP firewall rules remain after NTP transition'
            }
            $legacyRules = @(Get-NetFirewallRule -ErrorAction Stop |
                Where-Object { [string]$_.Name -like 'Homelab-PTP-*' })
            if ($legacyRules.Count -gt 0) {
                Write-Warning 'Legacy Homelab-PTP-* firewall rules remain; inspect them separately. They were not changed.'
            }
            Set-ServiceAutomatic
            $type = Get-RegistryValueState 'Parameters' 'Type'
            $ntpServer = Get-RegistryValueState 'Parameters' 'NtpServer'
            $ntpEnabled = Get-RegistryValueState 'TimeProviders\NtpClient' 'Enabled'
            $inputEnabled = Get-RegistryValueState 'TimeProviders\NtpClient' 'InputProvider'
            $expectedNtpServer = ($peerText -join ' ')
            if ($type.value -ne 'NTP' -or
                ($ntpServer.value -replace '\s+',' ').Trim() -ne $expectedNtpServer -or
                $ntpEnabled.value -ne 1 -or $inputEnabled.value -ne 1) {
                throw 'NTP registry readback did not match the requested configuration'
            }
            foreach ($state in $discipline) {
                $actual = Get-RegistryValueState $state.path $state.name
                if (-not $actual.present -or [string]$actual.kind -ne $state.kind -or [string]$actual.value -ne [string]$state.value) {
                    throw "NTP registry readback failed for '$($state.path)\$($state.name)'"
                }
            }
            Test-NtpFirewallPolicy @($peers | ForEach-Object { $_.Text })
            $null = Test-Environment
        } catch {
            $original = $_.Exception.Message
            try { Restore-Payload $before; Stop-Config 5 "NTP apply failed: $original; configuration rolled back. Safety backup: '$backup'" }
            catch { Stop-Config 7 "NTP apply failed: $original; rollback failed: $($_.Exception.Message); safety backup: '$backup'" }
        }
        Write-Output "Configuration saved for NTP peers: $($peerText -join ', ')."
        Write-Output "Safety backup: $backup"
        Confirm-Restart $backup
    } finally { Exit-MachineLock }
}

function Set-ServiceStateFromPayload($ServiceState) {
    $mode = [string]$ServiceState.startMode
    $scMode = switch ($mode) { 'Auto' { 'auto' } 'Manual' { 'demand' } 'Disabled' { 'disabled' } default { throw "unsupported saved W32Time startup mode '$mode'" } }
    $result = Invoke-External "$env:SystemRoot\System32\sc.exe" @('config','W32Time','start=',$scMode)
    if ($result.ExitCode -ne 0) { throw "could not restore W32Time startup mode: $($result.Output) $($result.Error)" }
    $key = [Microsoft.Win32.Registry]::LocalMachine.OpenSubKey('SYSTEM\CurrentControlSet\Services\W32Time', $true)
    try {
        if ($ServiceState.delayedAutoStart.present) {
            $delayedKind = [Microsoft.Win32.RegistryValueKind][Enum]::Parse(
                [Microsoft.Win32.RegistryValueKind], [string]$ServiceState.delayedAutoStart.kind, $false)
            $key.SetValue('DelayedAutoStart', $ServiceState.delayedAutoStart.value, $delayedKind)
        } else { $key.DeleteValue('DelayedAutoStart', $false) }
    } finally { $key.Dispose() }
    if ($ServiceState.state -notin @('Running','Stopped')) { throw "unsupported saved W32Time runtime state '$($ServiceState.state)'" }
    $current = Get-Service W32Time
    if ($ServiceState.state -eq 'Running' -and $current.Status -ne 'Running' -and $mode -eq 'Disabled') {
        $temporary = Invoke-External "$env:SystemRoot\System32\sc.exe" @('config','W32Time','start=','demand')
        if ($temporary.ExitCode -ne 0) { throw "could not temporarily enable W32Time for runtime-state restoration: $($temporary.Error)" }
        Set-W32TimeRunningState 'Running'
        $disabled = Invoke-External "$env:SystemRoot\System32\sc.exe" @('config','W32Time','start=','disabled')
        if ($disabled.ExitCode -ne 0) { throw "could not restore disabled W32Time startup after starting it: $($disabled.Error)" }
    } else { Set-W32TimeRunningState $ServiceState.state }
    $verifyService = Get-CimInstance -ClassName Win32_Service -Filter "Name='W32Time'" -ErrorAction Stop
    $verifyDelayed = [Microsoft.Win32.Registry]::LocalMachine.OpenSubKey('SYSTEM\CurrentControlSet\Services\W32Time', $false)
    try {
        $delayedPresent = $verifyDelayed.GetValueNames() -contains 'DelayedAutoStart'
        if ($verifyService.StartMode -ne $mode -or $delayedPresent -ne [bool]$ServiceState.delayedAutoStart.present) {
            throw 'W32Time startup configuration readback failed during restore'
        }
        if ($ServiceState.delayedAutoStart.present -and
            ($verifyDelayed.GetValueKind('DelayedAutoStart').ToString() -ne $ServiceState.delayedAutoStart.kind -or
                $verifyDelayed.GetValue('DelayedAutoStart') -ne $ServiceState.delayedAutoStart.value)) {
            throw 'W32Time delayed-start type/value readback failed during restore'
        }
    } finally { $verifyDelayed.Dispose() }
    $restoredStatus = (Get-Service -Name W32Time -ErrorAction Stop).Status.ToString()
    if ($restoredStatus -ne $ServiceState.state) {
        throw "W32Time runtime-state readback failed; expected '$($ServiceState.state)', found '$restoredStatus'"
    }
}

function Remove-OwnedFirewallRule([string]$Name) {
    $rules = @(Get-NetFirewallRule -ErrorAction Stop | Where-Object { [string]$_.Name -ceq $Name })
    foreach ($rule in $rules) {
        if ($rule.Description -notmatch 'configTimeSettings ownership marker' -or
            $rule.Group -ne 'ESP32TimeServer-configTimeSettings') {
            throw "firewall rule name collision for '$Name'; ownership could not be verified"
        }
        Remove-NetFirewallRule -InputObject $rule -ErrorAction Stop | Out-Null
    }
}

function Restore-FirewallRules($Rules) {
    foreach ($name in $script:OwnedRuleNames) {
        Remove-OwnedFirewallRule $name
    }
    foreach ($rule in $Rules) {
        $parameters = @{
            Name = [string]$rule.name; DisplayName = [string]$rule.displayName
            Description = [string]$rule.description; Group = [string]$rule.group
            Direction = [string]$rule.direction; Action = [string]$rule.action
            Enabled = [string]$rule.enabled; Profile = [string]$rule.profile
            Protocol = [string]$rule.protocol; LocalPort = @($rule.localPort)
            RemotePort = @($rule.remotePort); LocalAddress = @($rule.localAddress)
            RemoteAddress = @($rule.remoteAddress); InterfaceAlias = @($rule.interfaceAlias)
        }
        if ($rule.application) { $parameters.Program = [string]$rule.application }
        if ($rule.service) { $parameters.Service = [string]$rule.service }
        New-NetFirewallRule @parameters -ErrorAction Stop | Out-Null
    }
}

function Restore-Payload($Payload) {
    foreach ($state in $Payload.registry) { Set-RegistryValueState $state }
    Restore-FirewallRules $Payload.firewallRules
    $ptpPath = "HKLM:\$script:RegistryRoot\TimeProviders\PtpClient"
    if (-not $Payload.ptpClientKeyExisted -and (Test-Path $ptpPath)) {
        $key = Get-Item -LiteralPath $ptpPath
        if ($key.SubKeyCount -eq 0 -and $key.GetValueNames().Count -eq 0) {
            Remove-Item -LiteralPath $ptpPath -Force -ErrorAction Stop
        } else {
            Write-Warning 'PtpClient key retained because unrelated values or subkeys are present'
        }
    }
    Set-ServiceStateFromPayload $Payload.service
}

function Get-RejectedTimeSamples([DateTime]$SinceUtc) {
    # Event 34: W32Time refused a correction beyond MaxPos/MaxNegPhaseCorrection.
    try {
        $events = @(Get-WinEvent -FilterHashtable @{
            LogName = 'System'; ProviderName = 'Microsoft-Windows-Time-Service'; Id = 34
            StartTime = $SinceUtc.ToLocalTime()
        } -ErrorAction Stop)
    } catch { return $null }
    if ($events.Count -lt 1) { return $null }
    $match = [regex]::Match([string]$events[0].Message, '(-?\d+)\s+seconds')
    $seconds = if ($match.Success) { $match.Groups[1].Value } else { 'unknown' }
    return [pscustomobject]@{ Count = $events.Count; Seconds = $seconds }
}

function Confirm-Restart([string]$BackupPath) {
    Write-Output 'Restart the Windows Time service now? (Y/n)'
    $answer = $null
    try {
        if ([Console]::IsInputRedirected) { $answer = 'N' }
        else { $answer = [Console]::ReadLine() }
    } catch { $answer = 'N' }
    if ($null -eq $answer) { $answer = 'N' }
    while ($answer -notmatch '^(?i:y|yes|n|no)?$') {
        Write-Output 'Please enter Y/Yes or N/No.'
        try { $answer = [Console]::ReadLine() } catch { $answer = 'N' }
        if ($null -eq $answer) { $answer = 'N' }
    }
    if ([string]::IsNullOrEmpty($answer) -or $answer -match '^(?i:y|yes)$') {
        try {
            Write-Output 'Restarting Windows Time; this can take up to a minute...'
            $service = Get-Service W32Time
            if ($service.Status -eq 'Running') {
                Set-W32TimeRunningState 'Stopped'
                Set-W32TimeRunningState 'Running'
            } else { Set-W32TimeRunningState 'Running' }
            $activationTime = [DateTime]::UtcNow
            $watch = [Diagnostics.Stopwatch]::StartNew()
            $output = Invoke-External "$env:SystemRoot\System32\w32tm.exe" @('/config','/update') 30
            if ($output.ExitCode -ne 0) { throw "w32tm /config /update failed: $($output.Output) $($output.Error)" }
            $output = Invoke-External "$env:SystemRoot\System32\w32tm.exe" @('/resync','/rediscover') 30
            if ($output.ExitCode -ne 0) {
                # PTP input arrives on the master's schedule; an immediate resync can fail before the first Sync.
                if (-not $script:ExpectedPtpMaster) { throw "w32tm /resync failed: $($output.Output) $($output.Error)" }
                Write-Warning "w32tm /resync did not complete immediately ($($output.ExitCode)); waiting for PTP synchronization."
            }
            Write-Output 'Waiting up to 3 minutes for a fresh synchronization (no input needed)...'
            $verified = $false
            $lastStatus = ''
            while ($watch.Elapsed.TotalSeconds -lt 180) {
                $remaining = [int][Math]::Floor(180 - $watch.Elapsed.TotalSeconds)
                if ($remaining -lt 1) { break }
                $status = Invoke-External "$env:SystemRoot\System32\w32tm.exe" @('/query','/status','/verbose') ([Math]::Min(30, $remaining))
                $lastStatus = $status.Output
                if ($status.ExitCode -eq 0) {
                    $syncLine = [regex]::Match($status.Output, '(?im)^Last Successful Sync Time:\s*(.+)$')
                    $syncTime = [DateTime]::MinValue
                    $errorLine = [regex]::Match($status.Output, '(?im)^Last Sync Error:\s*(\d+)')
                    $resultLine = [regex]::Match($status.Output, '(?im)^Last Sync Result:\s*(0x[0-9a-f]+)')
                    $goodResult = ($errorLine.Success -and $errorLine.Groups[1].Value -eq '0') -or
                        ($resultLine.Success -and $resultLine.Groups[1].Value -match '^0x0+$')
                    if ($syncLine.Success -and [DateTime]::TryParse(
                        $syncLine.Groups[1].Value.Trim(), [Globalization.CultureInfo]::CurrentCulture,
                        [Globalization.DateTimeStyles]::AssumeLocal, [ref]$syncTime) -and
                        $syncTime.ToUniversalTime() -ge $activationTime -and $goodResult) {
                        $remaining = [int][Math]::Floor(180 - $watch.Elapsed.TotalSeconds)
                        if ($remaining -lt 1) { break }
                        $source = Invoke-External "$env:SystemRoot\System32\w32tm.exe" @('/query','/source') ([Math]::Min(30, $remaining))
                        if ($script:ExpectedPtpMaster) {
                            $masterPattern = '(?<![\d.])' + [regex]::Escape($script:ExpectedPtpMaster) + '(?![\d.])'
                            if (($source.ExitCode -eq 0 -and $source.Output -match $masterPattern) -or
                                $status.Output -match '(?im)^ReferenceId:\s*0x4D505450\b') {
                                $verified = $true
                            }
                        } elseif ($source.ExitCode -eq 0) {
                            # w32tm reports manual peers with their flags, e.g. "192.168.1.24,0x8".
                            $actualSource = $source.Output.Trim() -replace ',0x[0-9a-fA-F]+$', ''
                            foreach ($expectedPeer in $script:ExpectedNtpPeers) {
                                if ($actualSource -ceq $expectedPeer) { $verified = $true; break }
                                $actualAddress = Test-Address $actualSource
                                $expectedAddress = Test-Address $expectedPeer
                                if ($null -ne $actualAddress -and $null -ne $expectedAddress -and
                                    $actualAddress.Normalized -eq $expectedAddress.Normalized) {
                                    $verified = $true
                                    break
                                }
                            }
                        }
                        if ($verified) { break }
                    }
                }
                $remaining = [int][Math]::Floor(180 - $watch.Elapsed.TotalSeconds)
                if ($remaining -gt 0) { Start-Sleep -Seconds ([Math]::Min(5, $remaining)) }
            }
            if (-not $verified) {
                if (-not [string]::IsNullOrWhiteSpace($lastStatus)) {
                    Write-Output 'Last W32Time status diagnostics:'
                    Write-Output $lastStatus.Trim()
                }
                Write-Output "Configuration applied; synchronization unverified. Restore with: .\tools\configTimeSettings.ps1 -restore `"$BackupPath`""
                exit 6
            }
            if ($script:ExpectedPtpMaster -and $null -ne $script:PtpCandidate) {
                $settle = 120 - ([DateTime]::UtcNow - $activationTime).TotalSeconds
                if ($settle -gt 0) {
                    Write-Output "Watching for rejected PTP samples for $([int][Math]::Ceiling($settle)) more seconds (no input needed)..."
                    Start-Sleep -Seconds ([int][Math]::Ceiling($settle))
                }
                $selection = Test-PtpNicSelection $script:PtpCandidate $script:ExpectedPtpMaster $activationTime
                if ($selection.State -eq 'Verified') {
                    Write-Output "PTP adapter check: $($selection.Detail)"
                } else {
                    Write-Warning "PTP adapter check: $($selection.Detail). Synchronization still works; this only affects which card timestamps PTP packets. Run -readiness for details."
                }
                $rejected = Get-RejectedTimeSamples $activationTime
                if ($null -ne $rejected) {
                    Write-Output "PTP samples are being rejected: Windows Time logged event 34 $($rejected.Count) time(s) since the restart (latest requested correction: $($rejected.Seconds) seconds)."
                    Write-Output "A correction this large means Windows used a zero PTP timestamp; ESP32 firmware that sends Sync messages without an originTimestamp causes this. Update the ESP32 firmware. See misc\setting_up_PTP_on_Windows.md, 'PTP samples rejected'."
                    Write-Output "Restore with: .\tools\configTimeSettings.ps1 -restore `"$BackupPath`""
                    exit 6
                }
                Write-Output 'PTP sample check: no rejected PTP samples (event 34) since the restart.'
            }
            $phase = [regex]::Match($lastStatus, '(?im)^Phase Offset:\s*(.+)$')
            $poll = [regex]::Match($lastStatus, '(?im)^Poll Interval:\s*(.+)$')
            if ($phase.Success -or $poll.Success) {
                $phaseText = if ($phase.Success) { $phase.Groups[1].Value.Trim() } else { 'unknown' }
                $pollText = if ($poll.Success) { $poll.Groups[1].Value.Trim() } else { 'unknown' }
                Write-Output "Current phase offset: $phaseText; poll interval: $pollText."
                Write-Output 'A fresh sync proves the source is active, not that the clock is aligned. Offsets below MaxAllowedPhaseOffset are slewed; recheck w32tm /query /status /verbose and confirm Phase Offset keeps shrinking.'
            }
            Write-Output 'Configuration and fresh synchronization verified.'
        } catch {
            try {
                $recovery = Read-BackupFile $BackupPath
                Restore-Payload $recovery
                Stop-Config 5 "service activation failed; configuration and original service state rolled back. Safety backup: '$BackupPath'. $($_.Exception.Message)"
            } catch {
                Stop-Config 7 "service activation failed and rollback could not be established; safety backup: '$BackupPath'. $($_.Exception.Message)"
            }
        }
    } else {
        Write-Output 'Configuration saved; service restart and synchronization verification deferred.'
        Write-Output 'A manual Windows Time service restart is needed to ensure activation.'
    }
}

function Get-EnabledTimeProviders($Payload) {
    $enabled = @()
    foreach ($provider in @('NtpClient','PtpClient')) {
        $state = @($Payload.registry | Where-Object { $_.path -eq "TimeProviders\$provider" -and $_.name -eq 'Enabled' })
        $on = if ($state.Count -eq 0 -or -not $state[0].present) { $provider -eq 'NtpClient' } else { [int]$state[0].value -ne 0 }
        if ($on) { $enabled += $provider }
    }
    return ($enabled -join ',')
}

function Invoke-Restore([string]$Path) {
    Test-BackupPath $Path
    if (-not (Test-Administrator)) { Stop-Config 2 '-restore requires an elevated administrator token' }
    Enter-MachineLock
    try {
        Test-CurrentDirectory
        $null = Test-Environment
        try { $payload = Read-BackupFile (Resolve-Path -LiteralPath $Path).Path }
        catch { Stop-Config 1 "backup file '$Path' is invalid: $($_.Exception.Message)" }
        if ($payload.scriptVersion -ceq $script:LegacyBackupVersion) {
            Write-Warning "This backup was made by configTimeSettings $($script:LegacyBackupVersion), before clock-discipline values were managed. MinPollInterval, MaxPollInterval, and UpdateInterval are left unchanged. Run -set NTP or -set PTP afterwards to apply that mode's profile."
        }
        $machineGuid = (Get-ItemProperty -LiteralPath 'HKLM:\SOFTWARE\Microsoft\Cryptography' -Name MachineGuid -ErrorAction Stop).MachineGuid
        if ($payload.machine.name -ne [Environment]::MachineName -or
            $payload.machine.machineGuid -ne $machineGuid) {
            Stop-Config 1 'backup belongs to a different machine'
        }
        $currentOs = Get-CimInstance Win32_OperatingSystem -ErrorAction Stop
        if ([string]$payload.machine.build -ne [string]$currentOs.BuildNumber -or
            [string]$payload.machine.architecture -ne [string]$currentOs.OSArchitecture) {
            Stop-Config 1 'backup was captured on an incompatible Windows build or architecture'
        }
        if ($null -ne $payload.selectedNic) {
            $savedGuid = [string]$payload.selectedNic.guid
            $savedAdapter = @(Get-NetAdapter -IncludeHidden -ErrorAction Stop |
                Where-Object { [string]$_.InterfaceGuid -ieq $savedGuid })
            if ($savedAdapter.Count -ne 1) {
                Stop-Config 1 "backup requires NIC '$savedGuid', which is not present"
            }
        }
        $safetyPath = Write-Backup $true
        $safety = Read-BackupFile $safetyPath
        try {
            Restore-Payload $payload
            foreach ($state in $payload.registry) {
                $actual = Get-RegistryValueState $state.path $state.name
                if ([bool]$actual.present -ne [bool]$state.present -or
                    ($state.present -and ($actual.kind -ne $state.kind -or (ConvertTo-Json $actual.value -Compress) -ne (ConvertTo-Json $state.value -Compress)))) {
                    throw "registry readback failed for '$($state.path)\$($state.name)'"
                }
            }
            if (-not (Test-FirewallRulesMatch $payload.firewallRules)) {
                throw 'owned firewall rule readback failed during restore'
            }
        } catch {
            $original = $_.Exception.Message
            try { Restore-Payload $safety; Stop-Config 5 "restore verification failed: $original; safety rollback succeeded. Safety backup: '$safetyPath'" }
            catch { Stop-Config 7 "restore failed: $original; safety rollback failed: $($_.Exception.Message); safety backup: '$safetyPath'" }
        }
        # Restoring the saved Running state does not reload a service that was already running.
        $activation = 'W32Time is stopped as in the backup; the restored configuration applies when it next starts.'
        if ([string]$payload.service.state -eq 'Running') {
            try {
                # w32tm /config /update does not load or unload time providers; switching NTP/PTP needs a restart.
                $restarted = $false
                if ((Get-EnabledTimeProviders $safety) -ne (Get-EnabledTimeProviders $payload)) {
                    Write-Output 'Restarting Windows Time because the enabled time providers changed; this can take up to a minute...'
                    Set-W32TimeRunningState 'Stopped'
                    Set-W32TimeRunningState 'Running'
                    $restarted = $true
                }
                $update = Invoke-External "$env:SystemRoot\System32\w32tm.exe" @('/config','/update') 30
                if ($update.ExitCode -ne 0) { throw "w32tm /config /update failed ($($update.ExitCode)): $($update.Output) $($update.Error)" }
                $activation = if ($restarted) { 'Windows Time was restarted and is using the restored configuration.' }
                    else { 'The running Windows Time service reloaded the restored configuration.' }
            } catch {
                $activation = $null
                Write-Warning "Restored configuration is saved, but the running Windows Time service did not reload it: $($_.Exception.Message). Restart the Windows Time service to activate it."
            }
        }
    } finally { Exit-MachineLock }
    if ($activation) { Write-Output $activation }
    Write-Output "Configuration restored and read back. Current synchronization was not restored or verified. Safety backup: $safetyPath"
}

if ($MyInvocation.InvocationName -eq '.') { return }

$null = Get-PhysicalComputerSystem
$parsed = Parse-Arguments @($args)
if ($parsed.Contains('help')) { Show-Help; exit 0 }

$operation = @('backup','restore','inventory','readiness','set') | Where-Object { $parsed.Contains($_) } | Select-Object -First 1
if ($operation -eq 'inventory') {
    Get-Inventory ([string]$parsed['inventory'])
    $inventoryServer = Test-Address ([string]$parsed['inventory']) $true
    if ($null -ne $inventoryServer) {
        try { Write-PtpReadiness (Get-PtpReadiness $inventoryServer.Text $false) }
        catch { Write-Warning "PTP readiness could not be assessed: $($_.Exception.Message)" }
    }
    if ($null -ne $script:InventoryRecommendation) { exit 0 }
    exit 3
}
if ($operation -eq 'readiness') {
    $server = Test-Address ([string]$parsed['readiness']) $true
    if ($null -eq $server) { Stop-Config 1 "readiness server '$($parsed['readiness'])' must be an IPv4 unicast literal" }
    try {
        if (Test-DirectedBroadcast $server.Text) { Stop-Config 1 "server '$($server.Text)' is a local IPv4 network or broadcast address" }
    } catch { Stop-Config 2 "local IPv4 network validation failed: $($_.Exception.Message)" }
    $null = Test-Environment $true
    try { $readiness = Get-PtpReadiness $server.Text $true }
    catch { Stop-Config 2 "PTP readiness could not be assessed: $($_.Exception.Message)" }
    Write-PtpReadiness $readiness
    if ($readiness.Ready) { exit 0 }
    exit 3
}
if ($operation -eq 'backup') {
    if (-not (Test-Administrator)) { Stop-Config 2 '-backup requires an elevated administrator token' }
    Enter-MachineLock
    try { Test-CurrentDirectory; $null = Test-Environment $true; $null = Write-Backup }
    finally { Exit-MachineLock }
    exit 0
}
if ($operation -eq 'restore') {
    Invoke-Restore ([string]$parsed['restore'])
    exit 0
}
if ($operation -eq 'set') {
    if ($parsed['set'] -eq 'PTP') {
        Set-PtpConfiguration $parsed
        exit 0
    }
    Set-NtpConfiguration $parsed
    exit 0
}
