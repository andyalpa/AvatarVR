param(
    [Parameter(Mandatory)][string]$BuildDir,
    [Parameter(Mandatory)][string]$OutputDir
)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$stage = Join-Path ([IO.Path]::GetTempPath()) ('AvatarVR-package-' + [guid]::NewGuid().ToString('N'))
$null = New-Item -ItemType Directory -Path (Join-Path $stage 'scripts'), (Join-Path $stage 'licenses'), $OutputDir -Force
Copy-Item -LiteralPath (Join-Path $BuildDir 'bin\AvatarVR.dll') -Destination (Join-Path $stage 'scripts')
$readme = Join-Path $root 'tools\share\overlay\README.md'
if (!(Test-Path -LiteralPath $readme)) { $readme = Join-Path $root 'README.md' }
Copy-Item -LiteralPath $readme -Destination $stage
$docs = Join-Path $stage 'docs'
New-Item -ItemType Directory -Path $docs | Out-Null
foreach ($name in @('ROADMAP.md', 'STATUS.md')) {
    $doc = Join-Path $root "docs\$name"
    if (!(Test-Path -LiteralPath $doc)) { $doc = Join-Path $root "tools\share\overlay\docs\$name" }
    if (Test-Path -LiteralPath $doc) { Copy-Item -LiteralPath $doc -Destination $docs }
}
$licenses = @{
    'MinHook.txt' = 'minhook\LICENSE.txt'
    'Dear-ImGui.txt' = 'imgui\LICENSE.txt'
    'Lua.txt' = 'lua\LICENSE'
    'OpenXR.txt' = 'openxr-sdk\LICENSE'
    'OpenXR-COPYING.adoc' = 'openxr-sdk\COPYING.adoc'
    'JsonCpp.txt' = 'openxr-sdk\src\external\jsoncpp\LICENSE'
}
foreach ($name in $licenses.Keys) {
    Copy-Item -LiteralPath (Join-Path (Join-Path $root 'third_party') $licenses[$name]) -Destination (Join-Path $stage "licenses\$name")
}
$zip = Join-Path $OutputDir 'AvatarVR-win-x64.zip'
Compress-Archive -Path (Join-Path $stage '*') -DestinationPath $zip -Force
$hash = (Get-FileHash -LiteralPath $zip -Algorithm SHA256).Hash.ToLowerInvariant()
Set-Content -LiteralPath "$zip.sha256" -Value "$hash  AvatarVR-win-x64.zip" -Encoding ASCII
