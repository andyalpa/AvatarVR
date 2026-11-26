param(
    [Parameter(Mandatory)][string]$BuildDir,
    [Parameter(Mandatory)][string]$OutputDir
)
# Release ZIP: scripts\AvatarVR.dll, README.md, and one THIRD_PARTY_NOTICES.txt.
# The notices stay because MinHook, Dear ImGui, Lua and the OpenXR loader are linked into the DLL
# and their licenses ask for the copyright text to ship with binaries.
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$stage = Join-Path ([IO.Path]::GetTempPath()) ('AvatarVR-package-' + [guid]::NewGuid().ToString('N'))
$null = New-Item -ItemType Directory -Path (Join-Path $stage 'scripts'), $OutputDir -Force
Copy-Item -LiteralPath (Join-Path $BuildDir 'bin\AvatarVR.dll') -Destination (Join-Path $stage 'scripts')
$readme = Join-Path $root 'tools\share\overlay\README.md'
if (!(Test-Path -LiteralPath $readme)) { $readme = Join-Path $root 'README.md' }
Copy-Item -LiteralPath $readme -Destination $stage

$licenses = [ordered]@{
    'MinHook'                 = 'minhook\LICENSE.txt'
    'Dear ImGui'              = 'imgui\LICENSE.txt'
    'Lua'                     = 'lua\LICENSE'
    'OpenXR SDK'              = 'openxr-sdk\LICENSE'
    'OpenXR SDK (COPYING)'    = 'openxr-sdk\COPYING.adoc'
    'JsonCpp (OpenXR loader)' = 'openxr-sdk\src\external\jsoncpp\LICENSE'
}
$notices = [Text.StringBuilder]::new()
[void]$notices.AppendLine('AvatarVR.dll includes the following open source software.')
foreach ($name in $licenses.Keys) {
    $text = Get-Content -Raw -LiteralPath (Join-Path (Join-Path $root 'third_party') $licenses[$name])
    [void]$notices.AppendLine().AppendLine(('=' * 78)).AppendLine($name).AppendLine(('=' * 78)).AppendLine().AppendLine($text.TrimEnd())
}
Set-Content -LiteralPath (Join-Path $stage 'THIRD_PARTY_NOTICES.txt') -Value $notices.ToString() -Encoding UTF8

$zip = Join-Path $OutputDir 'AvatarVR-win-x64.zip'
Compress-Archive -Path (Join-Path $stage '*') -DestinationPath $zip -Force
Remove-Item -LiteralPath $stage -Recurse -Force
$hash = (Get-FileHash -LiteralPath $zip -Algorithm SHA256).Hash.ToLowerInvariant()
Set-Content -LiteralPath "$zip.sha256" -Value "$hash  AvatarVR-win-x64.zip" -Encoding ASCII
