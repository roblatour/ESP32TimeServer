$ErrorActionPreference = 'Stop'

if ($PSVersionTable.PSVersion.Major -ne 5 -or $PSVersionTable.PSEdition -eq 'Core') {
    throw 'Run this test suite with Windows PowerShell 5.1.'
}

$scriptPath = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..\tools\configTimeSettings.ps1'))
$testRoot = Join-Path ([IO.Path]::GetTempPath()) ('configTimeSettings-tests-' + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $testRoot | Out-Null
$passed = 0
$failed = 0

function Invoke-Utility(
    [string[]]$Arguments,
    [ValidateSet('Physical','Virtual','VirtualWithoutManufacturer','Unavailable','Missing','MissingModel','MissingManufacturer','NoResult')]
    [string]$Machine = 'Physical'
) {
    $mockComputer = switch ($Machine) {
        'Physical' { '[pscustomobject]@{ Manufacturer = "Dell"; Model = "Precision"; PartOfDomain = $false }' }
        'Virtual' { '[pscustomobject]@{ Manufacturer = "Microsoft Corporation"; Model = "Virtual Machine"; PartOfDomain = $false }' }
        'VirtualWithoutManufacturer' { '[pscustomobject]@{ Manufacturer = ""; Model = "Virtual Machine"; PartOfDomain = $false }' }
        'Unavailable' { 'throw "simulated CIM failure"' }
        'Missing' { '[pscustomobject]@{ Manufacturer = ""; Model = ""; PartOfDomain = $false }' }
        'MissingModel' { '[pscustomobject]@{ Manufacturer = "Dell"; Model = ""; PartOfDomain = $false }' }
        'MissingManufacturer' { '[pscustomobject]@{ Manufacturer = ""; Model = "Precision"; PartOfDomain = $false }' }
        'NoResult' { 'return' }
    }
    $scriptLiteral = "'" + $scriptPath.Replace("'", "''") + "'"
    $argumentLiterals = @($Arguments | ForEach-Object { "'" + ([string]$_).Replace("'", "''") + "'" }) -join ' '
    $command = @"
function Get-CimInstance {
    [CmdletBinding()]
    param([string]`$ClassName)
    if (`$ClassName -ne 'Win32_ComputerSystem') { throw 'Unexpected CIM query during a command-line test' }
    $mockComputer
}
& $scriptLiteral $argumentLiterals
exit `$LASTEXITCODE
"@
    $encoded = [Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($command))
    $start = New-Object Diagnostics.ProcessStartInfo
    $start.FileName = Join-Path $PSHOME 'powershell.exe'
    $start.Arguments = "-NoLogo -NoProfile -NonInteractive -ExecutionPolicy RemoteSigned -EncodedCommand $encoded"
    $start.WorkingDirectory = $testRoot
    $start.UseShellExecute = $false
    $start.CreateNoWindow = $true
    $start.RedirectStandardOutput = $true
    $start.RedirectStandardError = $true
    $process = New-Object Diagnostics.Process
    $process.StartInfo = $start
    if (-not $process.Start()) { throw 'could not start Windows PowerShell test process' }
    $stdoutTask = $process.StandardOutput.ReadToEndAsync()
    $stderrTask = $process.StandardError.ReadToEndAsync()
    if (-not $process.WaitForExit(15000)) {
        $process.Kill()
        throw "utility timed out for arguments: $($Arguments -join ' ')"
    }
    return [pscustomobject]@{
        ExitCode = $process.ExitCode
        Output = $stdoutTask.Result
        Error = $stderrTask.Result
    }
}

function Assert-Case([string]$Name, [string[]]$Arguments, [int]$ExpectedCode, [string]$ExpectedText = '') {
    $result = Invoke-Utility $Arguments
    $allText = $result.Output + $result.Error
    if ($result.ExitCode -ne $ExpectedCode -or
        ($ExpectedText -and $allText -notmatch [regex]::Escape($ExpectedText))) {
        $script:failed++
        Write-Error ("FAIL: {0}; expected exit {1}, got {2}`n{3}" -f $Name, $ExpectedCode, $result.ExitCode, $allText)
    } else {
        $script:passed++
        Write-Output "PASS: $Name"
    }
}

try {
    Assert-Case 'no arguments show help' @() 0 'configTimeSettings v1'
    Assert-Case 'help alias' @('-?') 0 'Website: https://github.com/roblatour/ESP32TimeServer'
    Assert-Case 'case-insensitive help' @('-HELP') 0 'Copyright Rob Latour, 2026'
    Assert-Case 'help states the fixed NTP port' @('-help') 0 'Native W32Time NTP always uses UDP port 123'
    Assert-Case 'help states the virtual machine limitation' @('-help') 0 'This script is unable to work on a Virtual Machine'
    Assert-Case 'duplicate equal operation' @('-backup','-BACKUP') 1 'command line argument was duplicated'
    Assert-Case 'duplicate operation with different values' @('-set','NTP','-serveraddress1','192.0.2.1','-set','PTP') 1 'command line argument was duplicated'
    Assert-Case 'duplicate equal peer' @('-set','NTP','-serveraddress1','192.0.2.1','-SERVERADDRESS1','192.0.2.1') 1 'command line argument was duplicated'
    Assert-Case 'duplicate peer with different values' @('-set','NTP','-serveraddress1','192.0.2.1','-serveraddress1','192.0.2.2') 1 'command line argument was duplicated'
    Assert-Case 'help aliases are one logical parameter' @('-help','-?') 1 'command line argument was duplicated'
    Assert-Case 'help cannot combine with an operation' @('-help','-backup') 1 'exactly one operation'
    Assert-Case 'abbreviations are rejected' @('-inv','192.0.2.1') 1 'unknown parameter'
    Assert-Case 'double-dash aliases are rejected' @('--backup') 1 'unknown parameter'
    Assert-Case 'unexpected positional token is rejected' @('backup') 1 'unexpected positional token'
    Assert-Case 'NTP server is required' @('-set','NTP') 1 '-serveraddress1 is required'
    Assert-Case 'peer-number gap is rejected' @('-set','NTP','-serveraddress1','192.0.2.1','-serveraddress3','192.0.2.2') 1 '-serveraddress3 requires -serveraddress2'
    foreach ($n in 1..4) {
        Assert-Case "removed port$n option is rejected" @('-set','NTP','-serveraddress1','192.0.2.1',"-port$n",'123') 1 "unknown parameter '-port$n'"
    }
    Assert-Case 'nonstandard port option is rejected' @('-set','NTP','-serveraddress1','192.0.2.1','-port1','80') 1 "unknown parameter '-port1'"
    Assert-Case 'NTP peer cannot be a DNS name' @('-inventory','time.example.org') 1 'must be an IPv4 unicast literal'
    Assert-Case 'ambiguous IPv4 with leading zero is rejected' @('-inventory','010.0.0.1') 1 'must be an IPv4 unicast literal'
    Assert-Case 'IPv4 shorthand is rejected' @('-inventory','192.0.2') 1 'must be an IPv4 unicast literal'
    Assert-Case 'IPv4 loopback is rejected' @('-inventory','127.0.0.1') 1 'must be an IPv4 unicast literal'
    Assert-Case 'IPv4 multicast is rejected' @('-inventory','239.1.2.3') 1 'must be an IPv4 unicast literal'
    Assert-Case 'IPv4 unspecified range is rejected' @('-inventory','0.1.2.3') 1 'must be an IPv4 unicast literal'
    Assert-Case 'PTP cannot be combined with NTP peer arguments' @('-set','PTP','192.0.2.1','-serveraddress1','192.0.2.2') 1 'NTP peer parameters cannot be combined with PTP'
    Assert-Case 'removed -nic option is rejected' @('-set','PTP','192.0.2.1','-nic','Ethernet') 1 "parameter '-nic' is no longer supported"
    Assert-Case 'readiness requires a server address' @('-readiness') 1 "parameter '-readiness' requires a non-empty value"
    Assert-Case 'readiness cannot combine with another operation' @('-readiness','192.0.2.1','-inventory','192.0.2.1') 1 'exactly one operation'
    Assert-Case 'readiness rejects NTP peer parameters' @('-readiness','192.0.2.1','-serveraddress1','192.0.2.1') 1 "parameter '-serveraddress1' is not valid for -readiness"
    Assert-Case 'readiness rejects a DNS name' @('-readiness','time.example.org') 1 'must be an IPv4 unicast literal'
    Assert-Case 'PTP requires a positional server' @('-set','PTP') 1 '-set PTP requires an IPv4 server address immediately after PTP'
    Assert-Case 'old PTP server option is rejected' @('-set','PTP','-serveraddress','192.0.2.1') 1 '-set PTP requires an IPv4 server address immediately after PTP'
    Assert-Case 'old server option is rejected after a positional server' @('-set','PTP','192.0.2.1','-serveraddress','192.0.2.2') 1 "unknown parameter '-serveraddress'"
    Assert-Case 'old readiness server option is rejected' @('-readiness','-serveraddress','192.0.2.1') 1 "parameter '-readiness' requires a non-empty value"
    Assert-Case 'extra PTP positional address is rejected' @('-set','PTP','192.0.2.1','192.0.2.2') 1 'unexpected positional token'
    Assert-Case 'extra readiness positional address is rejected' @('-readiness','192.0.2.1','192.0.2.2') 1 'unexpected positional token'
    Assert-Case 'duplicate readiness is rejected' @('-readiness','192.0.2.1','-READINESS','192.0.2.2') 1 'command line argument was duplicated'
    foreach ($operation in @('inventory','readiness')) {
        foreach ($invalid in @('2001:db8::1','192.0.2.256','255.255.255.255','192.0.2.1:123','192.0.2.1/24')) {
            Assert-Case "$operation rejects $invalid" @("-$operation",$invalid) 1 'must be an IPv4 unicast literal'
        }
    }
    foreach ($name in @('backup.json','backup','backup.bak.json')) {
        Assert-Case "restore rejects a non-bak extension: $name" @('-restore',$name) 1 'must have a .bak extension'
    }
    Assert-Case 'restore accepts an arbitrary name before checking existence' @('-restore','my-settings.bak') 1 "backup file 'my-settings.bak' does not exist"
    Assert-Case 'restore does not interpret dates in filenames' @('-restore','configtimebackup_2026_02_30_12_00_00.bak') 1 "backup file 'configtimebackup_2026_02_30_12_00_00.bak' does not exist"
    $spacePath = Join-Path (Join-Path $testRoot 'path with spaces') 'backup.json'
    New-Item -ItemType Directory -Path (Split-Path -Parent $spacePath) | Out-Null
    Assert-Case 'quoted path with spaces is one argument' @('-restore',$spacePath) 1 'must have a .bak extension'
    Remove-Item -LiteralPath (Split-Path -Parent $spacePath) -Recurse -Force
    Assert-Case 'URL is rejected as a server' @('-inventory','https://192.0.2.1') 1 'must be an IPv4 unicast literal'

    $help = Invoke-Utility @('-help')
    if ($help.ExitCode -ne 0 -or $help.Output -match '(?i)-port\d*\b') {
        throw 'help still advertises a removed port option'
    }
    $helpLines = $help.Output.TrimEnd() -split '\r?\n'
    $footer = @($helpLines[($helpLines.Length - 4)..($helpLines.Length - 1)]) -join "`n"
    $expectedFooter = @(
        'configTimeSettings v1.1',
        'Copyright Rob Latour, 2026',
        'License: MIT',
        'Website: https://github.com/roblatour/ESP32TimeServer'
    ) -join "`n"
    if ($footer -cne $expectedFooter) { throw 'help footer did not match the required final lines' }
    $passed++
    Write-Output 'PASS: help does not advertise configurable ports and preserves the footer'

    $vmCases = @(
        [pscustomobject]@{ Name = 'no arguments'; Arguments = [string[]]@() }
        [pscustomobject]@{ Name = 'help'; Arguments = [string[]]@('-help') }
        [pscustomobject]@{ Name = 'help alias'; Arguments = [string[]]@('-?') }
        [pscustomobject]@{ Name = 'backup'; Arguments = [string[]]@('-backup') }
        [pscustomobject]@{ Name = 'restore'; Arguments = [string[]]@('-restore','configtimebackup_2026_11_05_11_20_30.bak') }
        [pscustomobject]@{ Name = 'inventory'; Arguments = [string[]]@('-inventory','192.0.2.1') }
        [pscustomobject]@{ Name = 'readiness'; Arguments = [string[]]@('-readiness','192.0.2.1') }
        [pscustomobject]@{ Name = 'NTP'; Arguments = [string[]]@('-set','NTP','-serveraddress1','192.0.2.1') }
        [pscustomobject]@{ Name = 'PTP'; Arguments = [string[]]@('-set','PTP','192.0.2.1') }
        [pscustomobject]@{ Name = 'invalid arguments'; Arguments = [string[]]@('-unknown') }
    )
    foreach ($case in $vmCases) {
        $result = Invoke-Utility -Arguments $case.Arguments -Machine 'Virtual'
        if ($result.ExitCode -ne 2 -or $result.Error -notmatch 'ERROR: A virtual machine was detected' -or
            $result.Error -match '(?i)\bwarning\b' -or $result.Output -match '\S') {
            throw "VM startup refusal failed for $($case.Name): exit $($result.ExitCode); $($result.Output) $($result.Error)"
        }
        $passed++
        Write-Output "PASS: VM startup refusal precedes $($case.Name)"
    }
    $result = Invoke-Utility -Arguments @('-help') -Machine 'VirtualWithoutManufacturer'
    if ($result.ExitCode -ne 2 -or $result.Error -notmatch 'ERROR: A virtual machine was detected') {
        throw 'a virtual model with a missing manufacturer was not identified as a VM'
    }
    $passed++
    Write-Output 'PASS: a recognizable VM is reported even when its manufacturer is missing'
    foreach ($machine in @('Unavailable','Missing','MissingModel','MissingManufacturer','NoResult')) {
        $result = Invoke-Utility -Arguments @('-help') -Machine $machine
        if ($result.ExitCode -ne 2 -or $result.Error -notmatch 'ERROR: virtual-machine detection failed' -or
            $result.Output -match '\S') {
            throw "indeterminate VM status did not fail safely for $machine"
        }
        $passed++
        Write-Output "PASS: $machine computer identity fails safely at startup"
    }

    $residual = @(Get-ChildItem -LiteralPath $testRoot -Force)
    if ($residual.Count -ne 0) { throw 'invalid invocations created files in the caller working directory' }

    $previousLocation = Get-Location
    try {
        Set-Location -LiteralPath $testRoot
        . $scriptPath
        $ptpArguments = Parse-Arguments @('-SET','ptp','192.0.2.1')
        $readinessArguments = Parse-Arguments @('-READINESS','192.0.2.1')
        $inventoryArguments = Parse-Arguments @('-INVENTORY','192.0.2.1')
        if ($ptpArguments['set'] -cne 'PTP' -or $ptpArguments['serveraddress'] -ne '192.0.2.1' -or
            $ptpArguments.Count -ne 2 -or $readinessArguments['readiness'] -ne '192.0.2.1' -or
            $readinessArguments.Count -ne 1 -or $inventoryArguments['inventory'] -ne '192.0.2.1') {
            throw 'positional IPv4 server parsing failed'
        }
        $passed++
        Write-Output 'PASS: positional PTP, readiness, and inventory servers are parsed case-insensitively'
        $restoreDirectory = Join-Path $testRoot 'restore files with spaces'
        New-Item -ItemType Directory -Path $restoreDirectory | Out-Null
        try {
            foreach ($name in @('my settings.bak','renamed.backup.BAK','configtimebackup_2026_02_30_12_00_00.bak')) {
                $restorePath = Join-Path $restoreDirectory $name
                New-Item -ItemType File -Path $restorePath | Out-Null
                Test-BackupPath $restorePath
                Test-BackupPath (Join-Path 'restore files with spaces' $name)
            }
        } finally {
            Remove-Item -LiteralPath $restoreDirectory -Recurse
        }
        $passed++
        Write-Output 'PASS: restore accepts arbitrary .bak names, uppercase extensions, and relative/absolute paths with spaces'
        $addresses = @('192.0.2.1','2001:db8::2','192.0.2.3','2001:db8::4')
        $expectedPeerLists = @(
            '192.0.2.1,0x8',
            '192.0.2.1,0x8 2001:db8::2,0xA',
            '192.0.2.1,0x8 2001:db8::2,0x8 192.0.2.3,0x8',
            '192.0.2.1,0x8 2001:db8::2,0x8 192.0.2.3,0x8 2001:db8::4,0x8'
        )
        for ($count = 1; $count -le 4; $count++) {
            $tokens = @('-set','NTP')
            $peers = @()
            for ($n = 1; $n -le $count; $n++) {
                $tokens += @("-serveraddress$n", $addresses[$n - 1])
                $peer = Test-Address $addresses[$n - 1]
                if ($null -eq $peer) { throw "valid NTP peer $n was rejected" }
                $peers += $peer
            }
            $parsed = Parse-Arguments $tokens
            if ($parsed['set'] -ne 'NTP' -or $parsed.Count -ne ($count + 1)) {
                throw "portless parsing failed for $count NTP peers"
            }
            $peerList = @(Get-NtpPeerTokens $peers) -join ' '
            if ($peerList -cne $expectedPeerLists[$count - 1]) {
                throw "unexpected W32Time peer list for $count peers: $peerList"
            }
        }
        if ($null -ne (Test-Address '192.0.2.1:123')) {
            throw 'an IPv4 address with an appended port was accepted'
        }
        $passed++
        Write-Output 'PASS: one to four portless NTP peers preserve W32Time flags and default-port syntax'
        foreach ($invalid in @('time.example.org','::','::1','ff02::1','fe80::1','[2001:db8::1]',
            '[2001:db8::1]:123','2001:db8::1,0x8','2001:db8::gg','::ffff:127.0.0.1',
            '::ffff:7f00:1','::ffff:239.1.2.3','::ffff:0.0.0.0','::ffff:192.000.2.1',
            ' 2001:db8::1','2001:db8::1 ','192.0.2.1:123','010.0.0.1','0xc0000201')) {
            if ($null -ne (Test-Address $invalid)) { throw "invalid NTP address accepted: $invalid" }
        }
        if ((Test-Address '2001:0DB8:0:0:0:0:0:24').Normalized -cne '2001:db8::24' -or
            $null -eq (Test-Address '::ffff:192.0.2.1')) {
            throw 'IPv6 normalization or valid mapped IPv4 address handling failed'
        }
        & {
            function Get-NetAdapter {
                [pscustomobject]@{ ifIndex = 23; Name = 'Ethernet Test'; InterfaceGuid = '{11111111-2222-3333-4444-555555555555}' }
                [pscustomobject]@{ ifIndex = 24; Name = 'Other'; InterfaceGuid = '{22222222-2222-3333-4444-555555555555}' }
            }
            $numeric = Test-Address 'fe80::24%23'
            foreach ($scope in @('Ethernet Test','{11111111-2222-3333-4444-555555555555}')) {
                $named = Test-Address "fe80::24%$scope"
                if ($null -eq $named -or $named.Text -cne 'fe80::24%23' -or
                    $named.Normalized -cne $numeric.Normalized -or $named.IP.ScopeId -ne 23 -or
                    (@(Get-NtpPeerTokens @($named)) -join ' ') -cne 'fe80::24%23,0x8') {
                    throw "IPv6 scope '$scope' did not reach W32Time as a numeric interface index"
                }
            }
            if ($null -ne (Test-Address 'fe80::24%999999') -or
                $null -ne (Test-Address '2001:db8::24%23') -or
                (Test-Address 'fe80::24%24').Normalized -ceq $numeric.Normalized) {
                throw 'IPv6 scope validation or scope-aware duplicate identity failed'
            }
        }
        $passed++
        Write-Output 'PASS: IPv6 validation, normalization, mapped-address exclusions, and numeric link-local scope propagation'

        $virtualComputers = @(
            [pscustomobject]@{ Manufacturer = 'Microsoft Corporation'; Model = 'Virtual Machine' }
            [pscustomobject]@{ Manufacturer = 'VMware, Inc.'; Model = 'VMware7,1' }
            [pscustomobject]@{ Manufacturer = 'innotek GmbH'; Model = 'VirtualBox' }
            [pscustomobject]@{ Manufacturer = 'QEMU'; Model = 'Standard PC' }
            [pscustomobject]@{ Manufacturer = 'Google'; Model = 'Google Compute Engine' }
            [pscustomobject]@{ Manufacturer = 'Parallels'; Model = 'Generic PC' }
        )
        foreach ($computer in $virtualComputers) {
            if (-not (Test-VirtualMachine $computer)) {
                throw "virtual machine was not recognized: $($computer.Manufacturer) $($computer.Model)"
            }
        }
        $physicalComputer = [pscustomobject]@{
            Manufacturer = 'Microsoft Corporation'
            Model = 'Surface Laptop'
            HypervisorPresent = $true
        }
        if (Test-VirtualMachine $physicalComputer) {
            throw 'a physical computer with a hypervisor was treated as a virtual machine'
        }
        $passed++
        Write-Output 'PASS: VM identification and physical hypervisor-host distinction'

        $ptpCapabilities = [pscustomobject]@{
            Hardware = [pscustomobject]@{
                PtpV2OverUdpIPv4EventMessageReceive = $true
                PtpV2OverUdpIPv4AllMessageReceive = $false
                PtpV2OverUdpIPv4EventMessageTransmit = $false
                PtpV2OverUdpIPv4AllMessageTransmit = $false
                PtpV2OverUdpIPv6EventMessageReceive = $false
                PtpV2OverUdpIPv6AllMessageReceive = $false
                PtpV2OverUdpIPv6EventMessageTransmit = $false
                PtpV2OverUdpIPv6AllMessageTransmit = $false
                AllReceive = $false
                AllTransmit = $false
                TaggedTransmit = $true
            }
        }
        if (-not (Test-PtpV2Ipv4Hardware $ptpCapabilities)) {
            throw 'IPv4 PTP event receive plus tagged transmit was not recognized'
        }
        $ptpCapabilities.Hardware.PtpV2OverUdpIPv4EventMessageReceive = $false
        $ptpCapabilities.Hardware.PtpV2OverUdpIPv6EventMessageReceive = $true
        if (Test-PtpV2Ipv4Hardware $ptpCapabilities) {
            throw 'IPv6 receive capability was incorrectly treated as IPv4 support'
        }
        $ptpCapabilities.Hardware.PtpV2OverUdpIPv4AllMessageReceive = $true
        if (-not (Test-PtpV2Ipv4Hardware $ptpCapabilities)) {
            throw 'IPv4 all-message receive capability was not recognized'
        }
        Initialize-TimestampApi
        $hardwareStruct = New-Object ConfigTimeSettingsTimestampApi+HardwareCapabilities
        $softwareStruct = New-Object ConfigTimeSettingsTimestampApi+SoftwareCapabilities
        $timestampStruct = New-Object ConfigTimeSettingsTimestampApi+TimestampCapabilities
        $hardwareSize = [Runtime.InteropServices.Marshal]::SizeOf($hardwareStruct)
        $softwareSize = [Runtime.InteropServices.Marshal]::SizeOf($softwareStruct)
        $timestampSize = [Runtime.InteropServices.Marshal]::SizeOf($timestampStruct)
        if ($hardwareSize -ne 11 -or $softwareSize -ne 3 -or $timestampSize -ne 24) {
            throw "timestamp API native structure layout is unexpected ($hardwareSize/$softwareSize/$timestampSize)"
        }
        $passed++
        Write-Output 'PASS: PTPv2 IPv4 capability assessment and native IP Helper structure layout'
        $unknownTimestamp = Get-AdapterTimestampState ([pscustomobject]@{ ifIndex = 0 })
        if ($unknownTimestamp.Support -ne 'Unknown' -or
            $unknownTimestamp.Active -ne 'Unknown' -or -not $unknownTimestamp.Error) {
            throw 'timestamp API query failure did not produce explicit Unknown status'
        }
        $passed++
        Write-Output 'PASS: timestamp query failure is explicit and fail-safe'
        $kind = [Microsoft.Win32.RegistryValueKind][Enum]::Parse(
            [Microsoft.Win32.RegistryValueKind], 'DWord', $false)
        if ($kind -ne [Microsoft.Win32.RegistryValueKind]::DWord) {
            throw 'typed registry value conversion failed'
        }
        Test-CurrentDirectory
        $backupPath = Write-Backup
        if ([IO.Path]::GetFileName($backupPath) -notmatch '^configtimebackup_\d{4}_\d{2}_\d{2}_\d{2}_\d{2}_\d{2}\.bak$') {
            throw 'backup filename did not match the required local-time format'
        }
        $payload = Read-BackupFile $backupPath
        if ($payload.registry.Count -ne 16) { throw "backup contained $($payload.registry.Count) registry states instead of 16" }
        if ($payload.scriptVersion -ne '1.1' -or
            $payload.capturedLocalOffset -notmatch '^[+-]\d{2}:\d{2}(:\d{2})?$') {
            throw 'backup omitted its version or local UTC offset'
        }
        $backupAcl = Get-Acl -LiteralPath $backupPath
        if (-not $backupAcl.AreAccessRulesProtected) { throw 'backup ACL inheritance was not disabled' }
        $allowedSids = @(
            [Security.Principal.WindowsIdentity]::GetCurrent().User.Value,
            'S-1-5-32-544',
            'S-1-5-18'
        ) | Sort-Object
        $actualSids = @($backupAcl.GetAccessRules($true, $false,
            [Security.Principal.SecurityIdentifier]) | ForEach-Object { $_.IdentityReference.Value } |
            Sort-Object -Unique)
        if (($actualSids -join ',') -cne ($allowedSids -join ',')) {
            throw 'backup ACL does not contain exactly the current user, Administrators, and SYSTEM'
        }

        $invalidDigestPath = Join-Path $testRoot 'invalid-digest.json'
        $envelope = ConvertFrom-Json -InputObject ([IO.File]::ReadAllText($backupPath))
        $envelope.payloadSha256 = '0000000000000000000000000000000000000000000000000000000000000000'
        [IO.File]::WriteAllText($invalidDigestPath, (ConvertTo-Json $envelope -Compress),
            (New-Object Text.UTF8Encoding($false)))
        $digestRejected = $false
        try { $null = Read-BackupFile $invalidDigestPath }
        catch { $digestRejected = $_.Exception.Message -match 'digest' }
        if (-not $digestRejected) { throw 'backup digest corruption was not rejected' }
        Remove-Item -LiteralPath $invalidDigestPath -Force

        $duplicateRejected = $false
        try { $null = Test-NoDuplicateJsonPropertyNames '{"outer":{"key":1,"key":2}}' }
        catch { $duplicateRejected = $_.Exception.Message -match 'duplicate JSON property' }
        if (-not $duplicateRejected) { throw 'duplicate nested JSON property was not rejected' }
        $depthRejected = $false
        try { $null = Test-NoDuplicateJsonPropertyNames (('[' * 33) + '0' + (']' * 33)) }
        catch { $depthRejected = $_.Exception.Message -match 'nesting' }
        if (-not $depthRejected) { throw 'JSON nesting beyond 32 levels was not rejected' }
        if ($null -eq (Test-Address '2001:db8::1234') -or
            $null -ne (Test-Address '::') -or $null -ne (Test-Address '::1') -or
            $null -ne (Test-Address 'ff02::1') -or $null -ne (Test-Address 'fe80::1')) {
            throw 'IPv6 unicast/special-address validation failed'
        }
        $localAdapter = Get-NetAdapter -IncludeHidden | Select-Object -First 1
        $scopedAddress = Test-Address ("fe80::1%{0}" -f $localAdapter.ifIndex)
        if ($null -eq $scopedAddress -or $scopedAddress.Family -ne 'IPv6' -or
            $scopedAddress.Text -ne ("fe80::1%{0}" -f $localAdapter.ifIndex)) {
            throw 'valid local IPv6 scope was not retained'
        }
        if (-not (Test-IpInFirewallRange '192.0.2.25' '192.0.2.0/24') -or
            (Test-IpInFirewallRange '192.0.3.25' '192.0.2.0/24')) {
            throw 'firewall CIDR address matching failed'
        }

        $external = Invoke-External "$env:SystemRoot\System32\cmd.exe" @('/c','echo argument with spaces')
        if ($external.ExitCode -ne 0 -or $external.Output.Trim() -ne 'argument with spaces') {
            throw 'bounded structured external invocation did not preserve argument boundaries'
        }
        $passed++
        Write-Output 'PASS: directory access probe, JSON integrity/schema, protected ACL, registry types, and bounded argument passing'
        Remove-Item -LiteralPath $backupPath -Force
    } finally {
        Set-Location -LiteralPath $previousLocation.Path
    }

    $script:inventoryMock = [pscustomobject]@{
        Adapters = @(
            [pscustomobject]@{ Name='Fast Unknown'; InterfaceGuid='{00000000-0000-0000-0000-000000000003}'; ifIndex=31; InterfaceDescription='Test NIC'; MediaType='802.3'; NdisPhysicalMedium=14; HardwareInterface=$true; Status='Up' }
            [pscustomobject]@{ Name='Hardware Candidate B'; InterfaceGuid='{00000000-0000-0000-0000-000000000001}'; ifIndex=32; InterfaceDescription='Test NIC'; MediaType='802.3'; NdisPhysicalMedium=14; HardwareInterface=$true; Status='Up' }
            [pscustomobject]@{ Name='Hardware Candidate A'; InterfaceGuid='{00000000-0000-0000-0000-000000000002}'; ifIndex=33; InterfaceDescription='Test NIC'; MediaType='802.3'; NdisPhysicalMedium=14; HardwareInterface=$true; Status='Up' }
            [pscustomobject]@{ Name='Partial Replies'; InterfaceGuid='{00000000-0000-0000-0000-000000000004}'; ifIndex=34; InterfaceDescription='Test NIC'; MediaType='802.3'; NdisPhysicalMedium=14; HardwareInterface=$true; Status='Up' }
            [pscustomobject]@{ Name='Virtual Adapter'; InterfaceGuid='{00000000-0000-0000-0000-000000000005}'; ifIndex=35; InterfaceDescription='Test NIC'; MediaType='802.3'; NdisPhysicalMedium=0; HardwareInterface=$false; Status='Up' }
            [pscustomobject]@{ Name='Wrong Route'; InterfaceGuid='{00000000-0000-0000-0000-000000000006}'; ifIndex=36; InterfaceDescription='Test NIC'; MediaType='802.3'; NdisPhysicalMedium=14; HardwareInterface=$true; Status='Up' }
            [pscustomobject]@{ Name='Down Adapter'; InterfaceGuid='{00000000-0000-0000-0000-000000000007}'; ifIndex=37; InterfaceDescription='Test NIC'; MediaType='802.3'; NdisPhysicalMedium=14; HardwareInterface=$true; Status='Disconnected' }
            [pscustomobject]@{ Name='Connected Wi-Fi'; InterfaceGuid='{00000000-0000-0000-0000-000000000008}'; ifIndex=38; InterfaceDescription='Test Wi-Fi NIC'; MediaType='Native 802.11'; NdisPhysicalMedium=9; HardwareInterface=$true; Status='Up' }
        )
        Addresses = @(
            [pscustomobject]@{ InterfaceIndex=31; IPAddress='192.0.2.11' }
            [pscustomobject]@{ InterfaceIndex=32; IPAddress='192.0.2.12' }
            [pscustomobject]@{ InterfaceIndex=33; IPAddress='192.0.2.13' }
            [pscustomobject]@{ InterfaceIndex=34; IPAddress='192.0.2.14' }
            [pscustomobject]@{ InterfaceIndex=35; IPAddress='192.0.2.15' }
            [pscustomobject]@{ InterfaceIndex=36; IPAddress='192.0.2.16' }
            [pscustomobject]@{ InterfaceIndex=38; IPAddress='192.0.2.18' }
        )
        ProbeCalls = @()
    }
    foreach ($physicalMedium in @(1, 9, 10)) {
        $emulatedWireless = [pscustomobject]@{
            HardwareInterface=$true; MediaType='802.3'; NdisPhysicalMedium=$physicalMedium
        }
        if (Test-PhysicalEthernetAdapter $emulatedWireless) {
            throw "wireless physical medium $physicalMedium was accepted as Ethernet"
        }
    }
    foreach ($physicalMedium in @(0, 14)) {
        $wiredAdapter = [pscustomobject]@{
            HardwareInterface=$true; MediaType='802.3'; NdisPhysicalMedium=$physicalMedium
        }
        if (-not (Test-PhysicalEthernetAdapter $wiredAdapter)) {
            throw "wired Ethernet physical medium $physicalMedium was rejected"
        }
    }
    function Test-Environment { return $true }
    function Test-DirectedBroadcast { return $false }
    function Get-NetAdapter { return $script:inventoryMock.Adapters }
    function Get-NetIPAddress { return $script:inventoryMock.Addresses }
    function Get-AdapterTimestampState($Adapter) {
        $support = switch ($Adapter.ifIndex) {
            32 { 'Yes' }
            33 { 'Yes' }
            34 { 'No' }
            35 { 'Yes' }
            36 { 'Unknown' }
            38 { 'Yes' }
            default { 'Unknown' }
        }
        $active = if ($Adapter.ifIndex -eq 32) { 'Yes' } else { 'Unknown' }
        return [pscustomobject]@{ Support=$support; Active=$active; Error=$null }
    }
    function Get-NetIPInterface($InterfaceIndex, $AddressFamily) {
        return [pscustomobject]@{ InterfaceIndex=$InterfaceIndex; WeakHostSend='Disabled' }
    }
    function Find-NetRoute($RemoteIPAddress, $LocalIPAddress) {
        $address = $script:inventoryMock.Addresses |
            Where-Object { $_.IPAddress -eq $LocalIPAddress } |
            Select-Object -First 1
        $routeIndex = if ($address.InterfaceIndex -eq 36) { 99 } else { $address.InterfaceIndex }
        return @(
            [pscustomobject]@{ DestinationPrefix='192.0.2.0/24'; InterfaceIndex=$routeIndex }
            [pscustomobject]@{ IPAddress=$LocalIPAddress; InterfaceIndex=$address.InterfaceIndex }
        )
    }
    function Invoke-InterfaceIcmpProbe($SourceAddress, $Server) {
        $script:inventoryMock.ProbeCalls += $SourceAddress
        $rtts = if ($SourceAddress -eq '192.0.2.14') { [double[]]@(1, 2) }
            elseif ($SourceAddress -eq '192.0.2.15') { [double[]]@(3, 3, 3) }
            elseif ($SourceAddress -eq '192.0.2.18') { [double[]]@(0, 0, 0) }
            else { [double[]]@(8, 8, 8) }
        $statuses = if ($rtts.Count -eq 2) { [string[]]@('Reply','Reply','Timeout') }
            else { [string[]]@('Reply','Reply','Reply') }
        return [pscustomobject]@{ Measurable=$true; RoundTripTimes=$rtts; Statuses=$statuses; Error=$null }
    }
    $inventoryRecords = @(Get-Inventory '192.0.2.1' 6>&1)
    $inventoryOutput = $inventoryRecords -join "`n"
    $recommendationLines = @($inventoryRecords | Where-Object {
        $_ -is [Management.Automation.InformationRecord] -and
        $_.MessageData.Message -like 'Recommendation:*'
    })
    if ($script:InventoryRecommendation.Guid -cne '{00000000-0000-0000-0000-000000000001}' -or
        $script:InventoryRecommendation.Active -ne 'Yes' -or
        $inventoryOutput -notmatch 'Recommendation: Hardware Candidate B' -or
        $inventoryOutput -notmatch 'partial-sample mean RTT: 1\.500 ms' -or
        $inventoryOutput -notmatch 'Wrong Route.*Not measurable' -or
        $inventoryOutput -notmatch 'Down Adapter.*Not tested \(not operationally up\)' -or
        $inventoryOutput -notmatch 'Connected Wi-Fi.*eligibility: No \(not physical wired Ethernet\)' -or
        $script:inventoryMock.ProbeCalls.Count -ne 6 -or
        $recommendationLines.Count -ne 1 -or
        $recommendationLines[0].MessageData.ForegroundColor -ne [ConsoleColor]::Green) {
        throw "inventory selection, reporting, or interface route validation failed`n$inventoryOutput"
    }
    $passed++
    Write-Output 'PASS: inventory probes, Ethernet-only eligibility, hardware-first ranking, tie-break, partial samples, route verification, and green recommendation'
    $wifiAdapter = @($script:inventoryMock.Adapters | Where-Object { $_.ifIndex -eq 38 })
    $wifiAddress = @($script:inventoryMock.Addresses | Where-Object { $_.InterfaceIndex -eq 38 })
    $script:inventoryMock.Adapters = @($script:inventoryMock.Adapters |
        Where-Object { $_.ifIndex -eq 34 })
    $script:inventoryMock.Addresses = @($script:inventoryMock.Addresses |
        Where-Object { $_.InterfaceIndex -eq 34 })
    $script:inventoryMock.ProbeCalls = @()
    $noCandidateRecords = @(Get-Inventory '192.0.2.1' 6>&1)
    $noCandidateOutput = $noCandidateRecords -join "`n"
    $noCandidateLines = @($noCandidateRecords | Where-Object {
        $_ -is [Management.Automation.InformationRecord] -and
        $_.MessageData.Message -like 'Recommendation:*'
    })
    if ($null -ne $script:InventoryRecommendation -or
        $noCandidateOutput -notmatch 'Recommendation: No eligible NIC' -or
        $noCandidateOutput -notmatch 'eligibility: No \(probe succeeded 2/3 \(3 required\)\)' -or
        $noCandidateLines.Count -ne 1 -or
        $noCandidateLines[0].MessageData.ForegroundColor -ne [ConsoleColor]::Yellow) {
        throw "inventory did not report a no-eligible result for partial ICMP replies`n$noCandidateOutput"
    }
    $passed++
    Write-Output 'PASS: inventory reports no eligible NIC in yellow when fewer than three replies succeed'
    $script:inventoryMock.Adapters = $wifiAdapter
    $script:inventoryMock.Addresses = $wifiAddress
    $wifiOnlyRecords = @(Get-Inventory '192.0.2.1' 6>&1)
    $wifiOnlyOutput = $wifiOnlyRecords -join "`n"
    if ($null -ne $script:InventoryRecommendation -or
        $wifiOnlyOutput -notmatch 'successes: 3/3' -or
        $wifiOnlyOutput -notmatch 'Recommendation: No eligible NIC') {
        throw "connected Wi-Fi was not excluded from recommendation`n$wifiOnlyOutput"
    }
    $passed++
    Write-Output 'PASS: reachable hardware-timestamp-capable Wi-Fi cannot be recommended'

    function New-PtpMockAdapter($Name, $Guid, $Index, $Media, $Physical, $Hardware, $Status) {
        return [pscustomobject]@{ Name=$Name; InterfaceGuid=$Guid; ifIndex=$Index; InterfaceDescription='Test NIC'
            MediaType=$Media; NdisPhysicalMedium=$Physical; HardwareInterface=$Hardware; Status=$Status; MacAddress='00-00-00-00-00-00' }
    }
    function Reset-PtpMock {
        $script:ptpMock = [pscustomobject]@{
            Adapters = @(
                New-PtpMockAdapter 'Ethernet 1 GB' '{CE3F6E03-6574-43F1-82DA-6DB677CB115A}' 23 '802.3' 14 $true 'Up'
                New-PtpMockAdapter 'Ethernet 10 GB' '{91F2659B-3D7E-4769-8135-62296F4095B0}' 16 '802.3' 14 $true 'Up'
                New-PtpMockAdapter 'vEthernet (Default Switch)' '{9514502E-D7F4-47B9-8FB4-90BB01005F0B}' 64 '802.3' 0 $false 'Up'
                New-PtpMockAdapter 'Wi-Fi' '{C19748A9-1D5F-4B50-BEB3-DD46551E70FA}' 20 'Native 802.11' 9 $true 'Disconnected'
            )
            Addresses = @(
                [pscustomobject]@{ InterfaceIndex=23; IPAddress='192.168.1.11'; PrefixLength=24 }
                [pscustomobject]@{ InterfaceIndex=16; IPAddress='192.168.1.10'; PrefixLength=24 }
                [pscustomobject]@{ InterfaceIndex=64; IPAddress='172.22.192.1'; PrefixLength=20 }
                [pscustomobject]@{ InterfaceIndex=20; IPAddress='169.254.10.20'; PrefixLength=16 }
            )
            Support = @{ 23='Yes'; 16='No'; 64='No'; 20='Yes' }
            ServerRoute = 16; MulticastRoute = 16; DefaultRoutes = @(23, 16)
            Replies = [double[]]@(8, 9, 10)
        }
    }
    function Set-PtpMockAdapterStatus($Index, $Status) {
        foreach ($adapter in $script:ptpMock.Adapters) { if ($adapter.ifIndex -eq $Index) { $adapter.Status = $Status } }
    }
    function Get-NetAdapter { return $script:ptpMock.Adapters }
    function Get-NetIPAddress { return $script:ptpMock.Addresses }
    function Get-NetRoute { return @($script:ptpMock.DefaultRoutes | ForEach-Object { [pscustomobject]@{ DestinationPrefix='0.0.0.0/0'; InterfaceIndex=$_ } }) }
    function Find-NetRoute($RemoteIPAddress, $LocalIPAddress) {
        if ($LocalIPAddress) {
            $index = @($script:ptpMock.Addresses | Where-Object { $_.IPAddress -eq $LocalIPAddress })[0].InterfaceIndex
            return @([pscustomobject]@{ DestinationPrefix='192.168.1.0/24'; InterfaceIndex=$index }
                [pscustomobject]@{ IPAddress=$LocalIPAddress; InterfaceIndex=$index })
        }
        $index = if ($RemoteIPAddress -eq '224.0.1.129') { $script:ptpMock.MulticastRoute } else { $script:ptpMock.ServerRoute }
        if ($null -eq $index) { throw 'simulated route lookup failure' }
        return @([pscustomobject]@{ IPAddress='192.168.1.11'; InterfaceIndex=$index }
            [pscustomobject]@{ DestinationPrefix='0.0.0.0/0'; InterfaceIndex=$index })
    }
    function Get-AdapterTimestampState($Adapter) {
        $support = $script:ptpMock.Support[[int]$Adapter.ifIndex]
        if (-not $support) { $support = 'Unknown' }
        return [pscustomobject]@{ Support=$support; Active='Unknown'; Error='active timestamp query failed (ERROR_BAD_DRIVER, 2001)' }
    }
    function Invoke-InterfaceIcmpProbe($SourceAddress, $Server) {
        return [pscustomobject]@{ Measurable=$true; RoundTripTimes=$script:ptpMock.Replies; Statuses=[string[]]@(); Error=$null }
    }
    function Invoke-PtpReadinessCase([bool]$Probe = $true) {
        $readiness = Get-PtpReadiness '192.168.1.24' $Probe
        $records = @(Write-PtpReadiness $readiness 6>&1 3>&1)
        $colored = @($records | Where-Object { $_ -is [Management.Automation.InformationRecord] })
        return [pscustomobject]@{
            Readiness = $readiness
            Text = ($records | ForEach-Object { if ($_ -is [Management.Automation.InformationRecord]) { $_.MessageData.Message } else { [string]$_ } }) -join "`n"
            Green = @($colored | Where-Object { $_.MessageData.ForegroundColor -eq [ConsoleColor]::Green } | ForEach-Object { $_.MessageData.Message })
            Yellow = @($colored | Where-Object { $_.MessageData.ForegroundColor -eq [ConsoleColor]::Yellow } | ForEach-Object { $_.MessageData.Message })
        }
    }

    Reset-PtpMock
    $case = Invoke-PtpReadinessCase
    if (-not $case.Readiness.Ready -or $case.Readiness.Candidate.Adapter.ifIndex -ne 23 -or
        $case.Green.Count -ne 2 -or $case.Green[0] -notmatch '^\s+\[PTP adapter - expected\] Ethernet 1 GB; GUID: \{CE3F6E03' -or
        $case.Green[1] -notmatch "^PTP readiness: Ready - expected PTP adapter 'Ethernet 1 GB' for 192\.168\.1\.24\.$" -or
        $case.Yellow.Count -ne 1 -or $case.Yellow[0] -notmatch '\[PTP adapter - alternate\] Ethernet 10 GB; .*hardware timestamp support: No; .*Windows routes PTP multicast 224\.0\.1\.129 through it' -or
        $case.Text -match 'vEthernet' -or $case.Text -notmatch 'This is allowed: Windows PTP listens on all adapters' -or
        $case.Text -match 'Disable-NetAdapter' -or $case.Text -notmatch 'ping 3/3 replies' -or
        @($case.Readiness.Wired).Count -ne 2) {
        throw "mixed-NIC readiness was not ready with the timestamp-capable NIC expected and the other NIC as an alternate`n$($case.Text)"
    }
    $passed++
    Write-Output 'PASS: PTP readiness accepts mixed NICs, expects the timestamp-capable NIC, and warns about the alternate'

    Set-PtpMockAdapterStatus 16 'Disabled'
    $script:ptpMock.ServerRoute = 23
    $script:ptpMock.MulticastRoute = 23
    $case = Invoke-PtpReadinessCase
    if (-not $case.Readiness.Ready -or $case.Readiness.Candidate.Adapter.ifIndex -ne 23 -or $case.Yellow.Count -ne 0 -or
        $case.Green.Count -ne 2 -or $case.Green[1] -notmatch "^PTP readiness: Ready - expected PTP adapter 'Ethernet 1 GB'" -or
        $case.Text -match 'This is allowed' -or
        $case.Text -notmatch 'ping 3/3 replies; mean RTT: 9\.000 ms' -or
        $case.Text -notmatch 'Windows does not confirm that hardware timestamping is active on .Ethernet 1 GB.*ERROR_BAD_DRIVER.*Hardware timestamping is not required') {
        throw "single-path readiness was not reported as ready`n$($case.Text)"
    }
    $passed++
    Write-Output 'PASS: PTP readiness is green with a single NIC and reports unconfirmed active timestamping as optional'

    $script:ptpMock.Replies = [double[]]@()
    $case = Invoke-PtpReadinessCase
    if (-not $case.Readiness.Ready -or $case.Text -notmatch 'ping 0/3 replies' -or $case.Text -notmatch 'PTP setup can still proceed') {
        throw "ICMP loss blocked PTP readiness`n$($case.Text)"
    }
    $passed++
    Write-Output 'PASS: PTP readiness treats ping as report-only'

    $script:ptpMock.Replies = [double[]]@(8, 9, 10)
    $script:ptpMock.MulticastRoute = 1
    $case = Invoke-PtpReadinessCase
    if (-not $case.Readiness.Ready -or $case.Readiness.Candidate.Adapter.ifIndex -ne 23) {
        throw "a multicast route outside the candidate blocked readiness`n$($case.Text)"
    }
    $script:ptpMock.ServerRoute = $null
    $script:ptpMock.MulticastRoute = 23
    $case = Invoke-PtpReadinessCase
    if (-not $case.Readiness.Ready -or $case.Text -notmatch 'Windows route lookup for 192\.168\.1\.24 failed; PTP setup can still proceed') {
        throw "a failed server route lookup was not reported as a warning`n$($case.Text)"
    }
    $script:ptpMock.ServerRoute = 64
    $case = Invoke-PtpReadinessCase
    if (-not $case.Readiness.Ready -or $case.Text -notmatch '\[Other path\] vEthernet \(Default Switch\); .*reason: virtual/software adapter' -or
        $case.Text -notmatch 'Windows routes 192\.168\.1\.24 through interface index 64, which is not a wired Ethernet adapter') {
        throw "a server route through a virtual adapter was not reported as a warning`n$($case.Text)"
    }
    $script:ptpMock.ServerRoute = 23
    $passed++
    Write-Output 'PASS: PTP readiness reports unexpected server or multicast routes as warnings only'

    $script:ptpMock.ServerRoute = 23
    $script:ptpMock.MulticastRoute = $null
    $script:ptpMock | Add-Member -Force -NotePropertyName RouteTable -NotePropertyValue @(
        [pscustomobject]@{ DestinationPrefix='224.0.0.0/4'; InterfaceIndex=64; RouteMetric=256 }
        [pscustomobject]@{ DestinationPrefix='224.0.0.0/4'; InterfaceIndex=23; RouteMetric=256 }
        [pscustomobject]@{ DestinationPrefix='224.0.0.0/4'; InterfaceIndex=1; RouteMetric=256 }
        [pscustomobject]@{ DestinationPrefix='224.0.0.0/4'; InterfaceIndex=20; RouteMetric=0 }
        [pscustomobject]@{ DestinationPrefix='0.0.0.0/0'; InterfaceIndex=23; RouteMetric=0 }
    )
    function Get-NetRoute($DestinationPrefix) {
        if ($DestinationPrefix) { return @($script:ptpMock.DefaultRoutes | ForEach-Object { [pscustomobject]@{ DestinationPrefix='0.0.0.0/0'; InterfaceIndex=$_ } }) }
        return $script:ptpMock.RouteTable
    }
    function Get-NetIPInterface {
        return @(
            [pscustomobject]@{ InterfaceIndex=64; InterfaceMetric=5000; ConnectionState='Connected' }
            [pscustomobject]@{ InterfaceIndex=23; InterfaceMetric=50; ConnectionState='Connected' }
            [pscustomobject]@{ InterfaceIndex=1; InterfaceMetric=75; ConnectionState='Connected' }
            [pscustomobject]@{ InterfaceIndex=20; InterfaceMetric=1; ConnectionState='Disconnected' }
        )
    }
    $case = Invoke-PtpReadinessCase
    if (-not $case.Readiness.Ready -or $case.Text -notmatch 'Windows routes PTP multicast 224\.0\.1\.129 through it') {
        throw "multicast route-table fallback did not select the lowest-metric connected interface`n$($case.Text)"
    }
    $script:ptpMock.RouteTable[2].RouteMetric = 231
    if ($null -ne (Get-RouteInterfaceIndex '224.0.1.129')) { throw 'tied multicast route metrics were not treated as ambiguous' }
    $script:ptpMock.ServerRoute = $null
    if ($null -ne (Get-RouteInterfaceIndex '192.168.1.99')) { throw 'route-table fallback was used for a unicast destination' }
    $script:ptpMock.ServerRoute = 23
    function Get-NetIPInterface($InterfaceIndex, $AddressFamily) {
        return [pscustomobject]@{ InterfaceIndex=$InterfaceIndex; WeakHostSend='Disabled' }
    }
    function Get-NetRoute { return @($script:ptpMock.DefaultRoutes | ForEach-Object { [pscustomobject]@{ DestinationPrefix='0.0.0.0/0'; InterfaceIndex=$_ } }) }
    $passed++
    Write-Output 'PASS: PTP multicast route falls back to the Windows route table when Find-NetRoute fails'

    Reset-PtpMock
    $script:ptpMock.Support[16] = 'Yes'
    $case = Invoke-PtpReadinessCase
    if (-not $case.Readiness.Ready -or $case.Readiness.Candidate.Adapter.ifIndex -ne 16 -or
        $case.Green[0] -notmatch '\[PTP adapter - expected\] Ethernet 10 GB;' -or
        $case.Yellow.Count -ne 1 -or $case.Yellow[0] -notmatch '\[PTP adapter - alternate\] Ethernet 1 GB;') {
        throw "two timestamp-capable NICs did not prefer the one Windows routes the server through`n$($case.Text)"
    }
    $passed++
    Write-Output 'PASS: PTP readiness accepts two timestamp-capable NICs and expects the server-route NIC'

    Reset-PtpMock
    $script:ptpMock.Support[23] = 'Unknown'
    $script:ptpMock.Support[16] = 'No'
    $script:ptpMock.Adapters[1].Name = "Rob's NIC"
    $case = Invoke-PtpReadinessCase
    if (-not $case.Readiness.Ready -or $case.Readiness.Candidate.Adapter.ifIndex -ne 16 -or
        $case.Text -notmatch "Ready - expected PTP adapter 'Rob's NIC'") {
        throw "NICs without hardware timestamp support were not accepted`n$($case.Text)"
    }
    $passed++
    Write-Output 'PASS: PTP readiness does not require hardware timestamp support'

    Reset-PtpMock
    Set-PtpMockAdapterStatus 16 'Disabled'
    Set-PtpMockAdapterStatus 20 'Up'
    $script:ptpMock.Addresses[3].IPAddress = '10.0.0.5'
    $script:ptpMock.Addresses[3].PrefixLength = 24
    $script:ptpMock.ServerRoute = 23
    $script:ptpMock.MulticastRoute = 20
    $case = Invoke-PtpReadinessCase
    if (-not $case.Readiness.Ready -or $case.Yellow[0] -notmatch '\[Other path\] Wi-Fi; .*reason: not physical wired Ethernet; path: Windows routes PTP multicast 224\.0\.1\.129 through it') {
        throw "an adapter carrying PTP multicast off-subnet was not reported`n$($case.Text)"
    }
    $script:ptpMock.MulticastRoute = 64
    $case = Invoke-PtpReadinessCase
    if (-not $case.Readiness.Ready -or $case.Text -notmatch '\[Other path\] vEthernet \(Default Switch\); .*reason: virtual/software adapter') {
        throw "a virtual adapter carrying PTP multicast was not reported`n$($case.Text)"
    }
    $passed++
    Write-Output 'PASS: PTP readiness reports Wi-Fi and virtual adapters that carry PTP multicast as other paths'

    Reset-PtpMock
    Set-PtpMockAdapterStatus 16 'Disabled'
    Set-PtpMockAdapterStatus 23 'Disabled'
    $case = Invoke-PtpReadinessCase
    if ($case.Readiness.Ready -or $case.Text -notmatch 'No up, physical wired Ethernet adapter is on the IPv4 subnet of 192\.168\.1\.24' -or
        $case.Yellow[-1] -ne 'PTP readiness: Not ready - resolve the items above, then rerun.' -or $null -ne $case.Readiness.Reachability) {
        throw "readiness without a wired adapter on the server subnet was accepted`n$($case.Text)"
    }
    $passed++
    Write-Output 'PASS: PTP readiness requires a wired Ethernet adapter on the server subnet'

    $rules = @(Get-PtpFirewallRuleDefinitions '192.168.1.24' @('Ethernet 1 GB') 'W32Time')
    $inbound = $rules | Where-Object { $_.direction -eq 'Inbound' }
    $outbound = $rules | Where-Object { $_.direction -eq 'Outbound' }
    if ($rules.Count -ne 2 -or $rules[0].name -cne $script:OwnedRuleNames[0] -or $rules[1].name -cne $script:OwnedRuleNames[1] -or
        $rules[0].description -notmatch 'configTimeSettings ownership marker' -or $rules[0].group -ne 'ESP32TimeServer-configTimeSettings' -or
        (@($inbound.remoteAddress) -join ',') -ne '192.168.1.24' -or (@($outbound.remoteAddress) -join ',') -ne '192.168.1.24,224.0.1.129' -or
        (@($inbound.localPort) -join ',') -ne '319,320' -or (@($outbound.remotePort) -join ',') -ne '319,320' -or
        $inbound.profile -ne 'Private' -or $inbound.service -ne 'W32Time' -or (@($inbound.interfaceAlias) -join ',') -ne 'Ethernet 1 GB') {
        throw 'PTP firewall rule definitions are incorrect'
    }
    function Get-OwnedFirewallState {
        return @($script:ruleReadback | ForEach-Object {
            $copy = [ordered]@{}
            foreach ($property in $_.PSObject.Properties) { $copy[$property.Name] = $property.Value }
            $copy.application = 'Any'
            $copy.service = 'w32time'
            $copy.remoteAddress = @($copy.remoteAddress | Sort-Object -Descending)
            $copy
        })
    }
    $script:ruleReadback = $rules
    if (-not (Test-PtpFirewallRules $rules)) { throw 'equivalent PTP firewall readback was rejected' }
    $script:ruleReadback = @(Get-PtpFirewallRuleDefinitions '192.168.1.25' @('Ethernet 1 GB') 'W32Time')
    if (Test-PtpFirewallRules $rules) { throw 'PTP firewall readback with a different server was accepted' }
    $multiRules = @(Get-PtpFirewallRuleDefinitions '192.168.1.24' @('Ethernet 10 GB', 'Ethernet 1 GB', 'Ethernet 10 GB') 'W32Time')
    if ((@($multiRules[0].interfaceAlias) -join ',') -ne 'Ethernet 1 GB,Ethernet 10 GB' -or
        (@($multiRules[1].interfaceAlias) -join ',') -ne 'Ethernet 1 GB,Ethernet 10 GB') {
        throw 'PTP firewall rules did not cover every PTP adapter exactly once'
    }
    $script:ruleReadback = $rules
    if (Test-PtpFirewallRules $multiRules) { throw 'PTP firewall readback missing an adapter was accepted' }
    $passed++
    Write-Output 'PASS: PTP firewall rules are owned, scoped to the server, multicast, PTP NICs, ports, profile, and service, and read back exactly'

    $states = @(Get-PtpRegistryStates '192.168.1.24')
    $stateText = @($states | ForEach-Object { "$($_.path)\$($_.name)=$($_.kind):$($_.value)" }) -join ';'
    if ($states.Count -ne 12 -or $stateText -notmatch 'TimeProviders\\PtpClient\\PtpMasters=String:192\.168\.1\.24' -or
        $stateText -notmatch 'TimeProviders\\PtpClient\\DllName=String:[^;]*\\System32\\ptpprov\.dll' -or
        $stateText -notmatch 'TimeProviders\\NtpClient\\Enabled=DWord:0' -or
        $stateText -notmatch 'EnableMulticastRx=DWord:1' -or $stateText -notmatch 'EnableMulticastTx=DWord:1' -or
        $stateText -notmatch 'DelayPollInterval=DWord:16000' -or $stateText -notmatch 'AnnounceInterval=DWord:4000') {
        throw "PTP registry states are incorrect: $stateText"
    }
    foreach ($state in $states) {
        if (-not $script:KnownValues.Contains("$($state.path)\$($state.name)")) { throw "PTP registry value is not allowlisted: $($state.name)" }
    }
    $passed++
    Write-Output 'PASS: PTP registry values are allowlisted and keep the NTP peer list untouched'

    $ntpPayload = [pscustomobject]@{ registry = @([pscustomobject]@{ path='TimeProviders\NtpClient'; name='Enabled'; present=$true; value=1 }
        [pscustomobject]@{ path='TimeProviders\PtpClient'; name='Enabled'; present=$false }) }
    $ptpPayload = [pscustomobject]@{ registry = @(Get-PtpRegistryStates '192.168.1.24') }
    if ((Get-EnabledTimeProviders $ntpPayload) -ne 'NtpClient' -or (Get-EnabledTimeProviders $ptpPayload) -ne 'PtpClient' -or
        (Get-EnabledTimeProviders ([pscustomobject]@{ registry = @() })) -ne 'NtpClient') {
        throw 'enabled time providers were not derived correctly from backup payloads'
    }
    $passed++
    Write-Output 'PASS: restore detects an NTP/PTP provider change that requires a Windows Time restart'

    Reset-PtpMock
    $candidate = (Get-PtpReadiness '192.168.1.24' $false).Candidate
    function Get-NetAdapter($InterfaceIndex, [switch]$IncludeHidden, $ErrorAction) {
        if ($null -ne $InterfaceIndex) { return @($script:ptpMock.Adapters | Where-Object { $_.ifIndex -eq $InterfaceIndex })[0] }
        return $script:ptpMock.Adapters
    }
    function New-Ptp513Event([string]$Master, [int]$IfIndex) {
        return [pscustomobject]@{ Message = "PTP provider has chosen the PTP Master $Master with IP Address ${Master}:320 on local IfIndex:$IfIndex `r`nas the best master and the source of time." }
    }
    function Get-WinEvent($FilterHashtable) {
        if ($FilterHashtable.Id -ne 513 -or $FilterHashtable.LogName -ne $script:PtpOperationalLog) { throw 'unexpected event query' }
        if ($script:ptpEvents.Count -lt 1) { throw 'No events were found that match the specified selection criteria.' }
        return $script:ptpEvents
    }
    $since = [DateTime]::UtcNow
    $script:ptpEvents = @((New-Ptp513Event '192.168.1.24' 23), (New-Ptp513Event '192.168.1.24' 16))
    $selection = Test-PtpNicSelection $candidate '192.168.1.24' $since
    if ($selection.State -ne 'Verified' -or $selection.Detail -ne "Windows PTP chose 'Ethernet 1 GB' (IfIndex 23), as expected") {
        throw "event 513 on the expected adapter was not verified: $($selection.Detail)"
    }
    $script:ptpEvents = @((New-Ptp513Event '192.168.1.240' 23), (New-Ptp513Event '192.168.1.24' 16))
    $selection = Test-PtpNicSelection $candidate '192.168.1.24' $since
    if ($selection.State -ne 'Other' -or $selection.Detail -notmatch "chose 'Ethernet 10 GB' \(IfIndex 16\), not the expected 'Ethernet 1 GB' \(IfIndex 23\)") {
        throw "event 513 on another adapter was not reported: $($selection.Detail)"
    }
    $script:ptpEvents = @()
    $selection = Test-PtpNicSelection $candidate '192.168.1.24' $since
    if ($selection.State -ne 'Unknown' -or $selection.Detail -notmatch 'did not log which adapter') {
        throw "missing event 513 was not reported as unknown: $($selection.Detail)"
    }
    Remove-Item Function:\Get-WinEvent
    $passed++
    Write-Output 'PASS: post-activation PTP NIC check reports the adapter Windows chose from event 513'

    $script:rejectedEvents = @(
        [pscustomobject]@{ Message = 'The time service has detected that the system time needs to be changed by -896959682 seconds. The time service will not change the system time by more than 54000 seconds.' }
        [pscustomobject]@{ Message = 'The time service has detected that the system time needs to be changed by -896959666 seconds.' }
    )
    function Get-WinEvent($FilterHashtable) {
        if ($FilterHashtable.Id -ne 34 -or $FilterHashtable.LogName -ne 'System') { throw 'unexpected event query' }
        if ($script:rejectedEvents.Count -lt 1) { throw 'No events were found that match the specified selection criteria.' }
        return $script:rejectedEvents
    }
    $rejected = Get-RejectedTimeSamples ([DateTime]::UtcNow)
    if ($null -eq $rejected -or $rejected.Count -ne 2 -or $rejected.Seconds -ne '-896959682') { throw 'rejected W32Time samples (event 34) were not reported' }
    $script:rejectedEvents = @()
    if ($null -ne (Get-RejectedTimeSamples ([DateTime]::UtcNow))) { throw 'absent event 34 was reported as rejected samples' }
    Remove-Item Function:\Get-WinEvent
    $passed++
    Write-Output 'PASS: rejected W32Time samples (event 34) are detected after PTP activation'

    function Stop-Config([int]$Code, [string]$Message) { throw "STOP ${Code}: $Message" }
    function Test-Administrator { return $true }
    function Enter-MachineLock { }
    function Exit-MachineLock { }
    function Test-CurrentDirectory { }
    function Get-Service { return [pscustomobject]@{ Name='W32Time'; Status='Running' } }
    function Get-RegistryValueState($RelativePath, $ValueName) {
        if ($RelativePath -eq 'TimeProviders\VMICTimeProvider') { return [ordered]@{ present = $true; kind = 'DWord'; value = 1 } }
        return [ordered]@{ present = $false }
    }
    function Get-CimInstance($ClassName, $Filter) {
        if ($Filter -match 'vmictimesync') { return [pscustomobject]@{ Name = 'vmictimesync'; State = $script:vmicState } }
        return @()
    }
    $script:vmicState = 'Stopped'
    try { Test-CompetingClockControllers } catch { throw "default-enabled VMICTimeProvider on a physical host was refused: $($_.Exception.Message)" }
    $script:vmicState = 'Running'
    $vmicResult = try { Test-CompetingClockControllers; 'no stop' } catch { $_.Exception.Message }
    if ($vmicResult -notmatch '^STOP 2: VMICTimeProvider is enabled and the Hyper-V Time Synchronization service') {
        throw "a running Hyper-V time synchronization service was not refused: $vmicResult"
    }
    $passed++
    Write-Output 'PASS: default-enabled VMICTimeProvider is refused only while Hyper-V time synchronization runs'
    function Get-NetFirewallProfile { return [pscustomobject]@{ Name='Private'; AllowLocalFirewallRules='True'; DefaultOutboundAction='Allow' } }
    function Get-NetFirewallRule { return $script:fwBlocks }
    function Get-NetFirewallPortFilter { param([Parameter(ValueFromPipeline)]$Rule) process { [pscustomobject]@{ Protocol='UDP'; LocalPort=@('Any'); RemotePort=@('Any') } } }
    function Get-NetFirewallAddressFilter { param([Parameter(ValueFromPipeline)]$Rule) process { [pscustomobject]@{ RemoteAddress=@('Any') } } }
    function Get-NetFirewallApplicationFilter { param([Parameter(ValueFromPipeline)]$Rule) process { [pscustomobject]@{ Program=$Rule.TestProgram } } }
    function Get-NetFirewallServiceFilter { param([Parameter(ValueFromPipeline)]$Rule) process { [pscustomobject]@{ Service=$Rule.TestService } } }
    function New-TestBlock($Name, $Program, $Service) {
        return [pscustomobject]@{ Name=$Name; Profile='Any'; Direction='Inbound'; TestProgram=$Program; TestService=$Service }
    }
    $script:fwBlocks = @(
        New-TestBlock 'UDP Query User{47B2C4D8}C:\arduino\arduino ide.exe' 'C:\arduino\arduino ide.exe' 'Any'
        New-TestBlock 'Other service block' '%SystemRoot%\System32\svchost.exe' 'Dnscache'
    )
    try { Test-PtpFirewallPolicy '192.168.1.24' } catch { throw "a program-specific block rule was treated as blocking W32Time: $($_.Exception.Message)" }
    foreach ($blocking in @(
        (New-TestBlock 'Global block' 'Any' 'Any'),
        (New-TestBlock 'Svchost block' '%SystemRoot%\System32\svchost.exe' 'Any'),
        (New-TestBlock 'W32Time block' 'Any' 'W32Time'))) {
        $script:fwBlocks = @($blocking)
        $fwResult = try { Test-PtpFirewallPolicy '192.168.1.24'; 'no block' } catch { $_.Exception.Message }
        if ($fwResult -notmatch "effective firewall block rule '$([regex]::Escape($blocking.Name))'") {
            throw "a block rule applying to W32Time was not detected: $($blocking.Name): $fwResult"
        }
    }
    $passed++
    Write-Output 'PASS: PTP firewall policy ignores block rules for other programs or services and detects rules affecting W32Time'
    $confirmCheck = 'y' | & powershell.exe -NoProfile -Command ". '$scriptPath'; `$r = @(Confirm-PtpPrerequisites '192.168.1.24' 6>`$null); if (`$r.Count -eq 1 -and `$r[0] -is [bool] -and -not `$r[0]) { 'BOOL-FALSE' } else { 'BAD:' + (`$r -join '|') }"
    if (@($confirmCheck) -notcontains 'BOOL-FALSE') { throw "Confirm-PtpPrerequisites must return a single boolean and decline redirected input: $confirmCheck" }
    $passed++
    Write-Output 'PASS: PTP prerequisite prompt is displayed, not returned, and redirected input declines'
    function Test-CompetingClockControllers { }
    function Test-PtpProviderInstalled { return $true }
    function Get-NetConnectionProfile($InterfaceIndex) {
        $category = if ($InterfaceIndex -eq 23) { $script:ptpSet.Category } else { $script:ptpSet.AltCategory }
        return [pscustomobject]@{ NetworkCategory = $category }
    }
    function Test-PtpFirewallPolicy { }
    function Get-W32TimeFirewallService { return 'W32Time' }
    function Confirm-PtpPrerequisites { return $script:ptpSet.Acknowledge }
    function Write-Backup([bool]$SafetyOnly) { $script:ptpSet.Backups += $SafetyOnly; return 'configtimebackup_2026_01_01_00_00_00.bak' }
    function Read-BackupFile { return [pscustomobject]@{ registry = @() } }
    function Set-RegistryValueState($State) { $script:ptpSet.Registry["$($State.path)\$($State.name)"] = $State }
    function Get-RegistryValueState($RelativePath, $ValueName) {
        $state = $script:ptpSet.Registry["$RelativePath\$ValueName"]
        if ($null -eq $state) { return [ordered]@{ present = $false } }
        return [ordered]@{ path = $RelativePath; name = $ValueName; present = $true; kind = $state.kind; value = $state.value }
    }
    function Restore-FirewallRules($Rules) { $script:ptpSet.Rules = @($Rules) }
    function Set-ServiceAutomatic { if ($script:ptpSet.FailService) { throw 'simulated service failure' } }
    function Test-PtpFirewallRules { return $true }
    function Restore-Payload { $script:ptpSet.Restored++ }
    function Confirm-Restart([string]$BackupPath) { $script:ptpSet.Restarted = $BackupPath }
    function Invoke-PtpSetCase([string]$Server = '192.168.1.24') {
        try { $output = @(Set-PtpConfiguration (Parse-Arguments @('-set','PTP',$Server)) 6>&1 3>&1) -join "`n"; return "OK`n$output" }
        catch { return $_.Exception.Message }
    }
    function Reset-PtpSet {
        $script:ptpSet = [pscustomobject]@{ Category='Private'; AltCategory='Private'; Acknowledge=$true; Backups=@(); Registry=@{}; Rules=@(); Restored=0; Restarted=$null; FailService=$false }
        $script:SelectedNic = $null; $script:ExpectedPtpMaster = $null; $script:PtpCandidate = $null
    }

    Reset-PtpMock
    Set-PtpMockAdapterStatus 16 'Disabled'
    Set-PtpMockAdapterStatus 23 'Disabled'
    Reset-PtpSet
    $result = Invoke-PtpSetCase
    if ($result -notmatch '^STOP 3: PTP readiness check failed; no backup or configuration changes were made' -or
        $script:ptpSet.Backups.Count -ne 0 -or $script:ptpSet.Registry.Count -ne 0) {
        throw "-set PTP did not stop before changes when readiness failed: $result"
    }
    Reset-PtpMock
    Reset-PtpSet
    $script:ptpSet.Category = 'Public'
    $result = Invoke-PtpSetCase
    if ($result -notmatch "^STOP 2: the network profile for 'Ethernet 1 GB' must be Private" -or $script:ptpSet.Backups.Count -ne 0) {
        throw "-set PTP accepted a Public network profile: $result"
    }
    Reset-PtpSet
    $script:ptpSet.Acknowledge = $false
    $result = Invoke-PtpSetCase
    if ($result -notmatch '^STOP 2: PTP prerequisites were not acknowledged' -or $script:ptpSet.Backups.Count -ne 0) {
        throw "-set PTP continued without acknowledgement: $result"
    }
    $passed++
    Write-Output 'PASS: -set PTP makes no backup or changes when readiness, network profile, or acknowledgement fails'

    Reset-PtpSet
    $result = Invoke-PtpSetCase
    $ptpMasters = $script:ptpSet.Registry['TimeProviders\PtpClient\PtpMasters']
    $ntpEnabled = $script:ptpSet.Registry['TimeProviders\NtpClient\Enabled']
    if ($result -notmatch '^OK' -or $result -notmatch "Configuration saved for PTP master 192\.168\.1\.24 on 'Ethernet 1 GB'" -or
        $result -notmatch 'There is no automatic NTP fallback' -or
        $result -notmatch "expected NIC: Ethernet 1 GB" -or $result -notmatch "PTP adapter\(s\) 'Ethernet 1 GB', 'Ethernet 10 GB'" -or
        $result -match 'alternate PTP adapter .* is not Private' -or
        (@($script:ptpSet.Backups) -join ',') -ne 'True' -or $script:ptpSet.Registry.Count -ne 12 -or
        $ptpMasters.value -ne '192.168.1.24' -or $ntpEnabled.value -ne 0 -or
        $script:ptpSet.Rules.Count -ne 2 -or (@($script:ptpSet.Rules[0].interfaceAlias) -join ',') -ne 'Ethernet 1 GB,Ethernet 10 GB' -or
        $script:ptpSet.Restarted -ne 'configtimebackup_2026_01_01_00_00_00.bak' -or
        $script:SelectedNic.InterfaceGuid -ne '{CE3F6E03-6574-43F1-82DA-6DB677CB115A}' -or
        $script:ExpectedPtpMaster -ne '192.168.1.24' -or @($script:ExpectedNtpPeers).Count -ne 0 -or
        $script:PtpCandidate.Adapter.ifIndex -ne 23 -or $script:ptpSet.Restored -ne 0) {
        throw "-set PTP success path did not apply the expected configuration: $result"
    }
    Reset-PtpSet
    $script:ptpSet.AltCategory = 'Public'
    $result = Invoke-PtpSetCase
    if ($result -notmatch '^OK' -or $result -notmatch "alternate PTP adapter 'Ethernet 10 GB' is not Private") {
        throw "-set PTP did not warn about a non-Private alternate adapter: $result"
    }
    Reset-PtpSet
    $script:ptpSet.FailService = $true
    $result = Invoke-PtpSetCase
    if ($result -notmatch 'PTP apply failed: simulated service failure; configuration rolled back' -or $script:ptpSet.Restored -ne 1 -or
        $null -ne $script:ptpSet.Restarted) {
        throw "-set PTP apply failure was not rolled back: $result"
    }
    $passed++
    Write-Output 'PASS: -set PTP backs up, applies PtpClient/NtpClient/firewall/service changes, verifies, and rolls back on failure'

    foreach ($invalid in @('2001:db8::24','time.example.org','010.0.0.1','192.0.2.256','127.0.0.1','239.1.2.3','255.255.255.255')) {
        Reset-PtpSet
        $result = Invoke-PtpSetCase $invalid
        if ($result -notmatch '^STOP 1: PTP server .* must be an IPv4 unicast literal' -or
            $script:ptpSet.Backups.Count -ne 0 -or $script:ptpSet.Registry.Count -ne 0) {
            throw "PTP accepted an invalid IPv4 server or changed configuration: $invalid; $result"
        }
    }
    $passed++
    Write-Output 'PASS: positional PTP rejects IPv6, DNS names, and invalid IPv4 before backup or configuration writes'

    & {
        function Get-NetAdapter {
            [pscustomobject]@{ ifIndex = 23; Name = 'Ethernet Test'; InterfaceGuid = '{11111111-2222-3333-4444-555555555555}' }
        }
        function Test-NtpFirewallPolicy($Peers) { $script:ntpFirewallPeers = @($Peers) }
        function Test-Ping($Address) { $script:ntpPingPeers += $Address; return $script:ntpPingSuccess }
        function Get-OwnedFirewallState { return @() }
        function Remove-OwnedFirewallRule { }
        function Get-NetFirewallRule { return @() }
        function Test-Path { return $false }
        function Invoke-External($FileName, $Arguments) {
            if ($FileName -ne "$env:SystemRoot\System32\w32tm.exe" -or $Arguments.Count -ne 3 -or
                $Arguments[0] -ne '/config' -or $Arguments[2] -ne '/syncfromflags:manual' -or
                $Arguments[1] -cne "/manualpeerlist:$script:ntpExpectedList") {
                throw "unexpected NTP command: $FileName $Arguments"
            }
            Set-RegistryValueState ([pscustomobject]@{ path='Parameters'; name='NtpServer'; kind='String'; value=$script:ntpExpectedList })
            return [pscustomobject]@{ ExitCode=0; Output='OK'; Error='' }
        }
        function Invoke-NtpSetCase($Tokens) {
            try { $null = Set-NtpConfiguration (Parse-Arguments $Tokens) 3>&1; return 'OK' }
            catch { return $_.Exception.Message }
        }
        $script:ntpPingSuccess = $true
        $cases = @(
            @{ Addresses=@('2001:0DB8:0:0:0:0:0:24'); Expected='2001:db8::24,0x8'; Peers=@('2001:db8::24') },
            @{ Addresses=@('192.0.2.1','2001:db8::24'); Expected='192.0.2.1,0x8 2001:db8::24,0xA'; Peers=@('192.0.2.1','2001:db8::24') },
            @{ Addresses=@('fe80::24%Ethernet Test'); Expected='fe80::24%23,0x8'; Peers=@('fe80::24%23') },
            @{ Addresses=@('192.0.2.1','2001:db8::24','192.0.2.3','fe80::24%23'); Expected='192.0.2.1,0x8 2001:db8::24,0x8 192.0.2.3,0x8 fe80::24%23,0x8'; Peers=@('192.0.2.1','2001:db8::24','192.0.2.3','fe80::24%23') }
        )
        foreach ($case in $cases) {
            Reset-PtpSet
            $script:ntpPingPeers = @()
            $script:ntpExpectedList = $case.Expected
            $tokens = @('-set','NTP')
            for ($n = 0; $n -lt $case.Addresses.Count; $n++) {
                $tokens += @("-serveraddress$($n + 1)", $case.Addresses[$n])
            }
            $result = Invoke-NtpSetCase $tokens
            if ($result -cne 'OK' -or $script:ptpSet.Backups.Count -ne 1 -or
                -not $script:ptpSet.Restarted -or $script:ptpSet.Restored -ne 0 -or
                ($script:ntpPingPeers -join ',') -cne ($case.Peers -join ',') -or
                ($script:ntpFirewallPeers -join ',') -cne ($case.Peers -join ',') -or
                ($script:ExpectedNtpPeers -join ',') -cne ($case.Peers -join ',')) {
                throw "NTP IPv4/IPv6 configuration pipeline failed: $result"
            }
        }
        foreach ($invalid in @('time.example.org','[2001:db8::24]','::ffff:127.0.0.1','fe80::24%999999')) {
            Reset-PtpSet
            $result = Invoke-NtpSetCase @('-set','NTP','-serveraddress1',$invalid)
            if ($result -notmatch '^STOP 1: .* is not an allowed literal unicast address' -or
                $script:ptpSet.Backups.Count -ne 0 -or $script:ptpSet.Registry.Count -ne 0) {
                throw "invalid NTP peer was not rejected before writes: $invalid; $result"
            }
        }
        foreach ($addresses in @(
            @('2001:db8::24','2001:0DB8:0:0:0:0:0:24'),
            @('fe80::24%23','fe80::24%Ethernet Test'))) {
            Reset-PtpSet
            $result = Invoke-NtpSetCase @('-set','NTP','-serveraddress1',$addresses[0],'-serveraddress2',$addresses[1])
            if ($result -notmatch '^STOP 1: duplicate NTP peer' -or $script:ptpSet.Backups.Count -ne 0) {
                throw "equivalent IPv6 NTP peers were not rejected before backup: $result"
            }
        }
        Reset-PtpSet
        $script:ntpPingSuccess = $false
        $result = Invoke-NtpSetCase @('-set','NTP','-serveraddress1','2001:db8::24')
        if ($result -notmatch '^STOP 3: .* did not answer any of three 1000 ms ICMP Echo requests' -or
            $script:ptpSet.Backups.Count -ne 0 -or $script:ptpSet.Registry.Count -ne 0) {
            throw "unreachable IPv6 NTP peer was not rejected before writes: $result"
        }
    }
    $passed++
    Write-Output 'PASS: mocked NTP apply preserves IPv4/IPv6 peer syntax, scopes, flags, ping/firewall inputs, and pre-write refusals'

    Write-Output ("configTimeSettings regression tests: {0} passed, {1} failed" -f $passed, $failed)
    if ($failed -gt 0) { exit 1 }
} finally {
    if (Test-Path -LiteralPath $testRoot) {
        Remove-Item -LiteralPath $testRoot -Recurse -Force
    }
}
