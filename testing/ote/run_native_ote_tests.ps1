$ErrorActionPreference = 'Stop'
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
if ($env:VSROOT) {
    $vsroot = $env:VSROOT
} elseif (Test-Path -LiteralPath $vswhere) {
    $vsroot = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if ($LASTEXITCODE -ne 0 -or -not $vsroot) {
        throw 'Visual Studio C++ build tools not found; set VSROOT to an installation with the x64 tools.'
    }
} else {
    throw 'Visual Studio Installer not found; set VSROOT to an installation with the x64 C++ tools.'
}
$vcvars = Join-Path $vsroot 'VC\Auxiliary\Build\vcvars64.bat'
if (-not (Test-Path -LiteralPath $vcvars)) {
    throw "Visual Studio x64 compiler environment not found: $vcvars"
}
$build = Join-Path $PSScriptRoot 'host_build'
if (Test-Path -LiteralPath $build) {
    throw "Refusing to overwrite existing generated directory: $build"
}
New-Item -ItemType Directory -Path $build | Out-Null
try {
    foreach ($family in @(0, 1, 2)) {
        $source = Join-Path $PSScriptRoot 'native_ote_host.cpp'
        $stubs = Join-Path $PSScriptRoot 'stubs'
        $command = 'call "{0}" >nul && cd /d "{1}" && cl /nologo /EHsc /std:c++17 /W4 /WX /DTEST_PREFER_IPVX={2} /I"{3}" "{4}" /Fenative_ote_host.exe /Fonative_ote_host.obj && native_ote_host.exe' -f $vcvars,$build,$family,$stubs,$source
        & $env:ComSpec /d /c $command
        if ($LASTEXITCODE -ne 0) {
            throw "Native OTE host tests failed for PreferIPvX=$family"
        }
    }
}
finally {
    Remove-Item -LiteralPath $build -Recurse -Force
    if (Test-Path -LiteralPath $build) {
        throw "Generated host test directory remains: $build"
    }
}
