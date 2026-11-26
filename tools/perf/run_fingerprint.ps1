# Run fingerprint (2026-11-11, ISSUE-05): what was true on this PC when the game started.
# Read-only. Writes scripts\RUN_FINGERPRINT.log, which run_cycle archives with the other logs, so a fast run and a slow
# run can be compared on everything the mod's own logs do not record: the game's settings file, the mod settings and
# switch files, the DLL, the power plan, GPU scheduling, monitor refresh and the other programs that use the GPU.
# Started by tools\run_cycle.ps1 once afop.exe is running. By hand:  powershell -File tools\perf\run_fingerprint.ps1
# Compare runs with:  python tools\perf\runs_report.py
param(
  [Parameter(Mandatory)][string]$Game,
  [string]$LogPath = ''
)
$ErrorActionPreference = 'Continue'
$scripts = Join-Path $Game 'scripts'
if (-not $LogPath) { $LogPath = Join-Path $scripts 'RUN_FINGERPRINT.log' }
$out = New-Object System.Collections.Generic.List[string]
function Add($text) { $out.Add([string]$text) }
function Section($name) { Add ''; Add "== $name" }

Add ('{0:yyyy-MM-dd HH:mm:ss} run fingerprint' -f (Get-Date))

Section 'mod dll'
try {
  $dll = Get-Item (Join-Path $scripts 'AvatarVR.dll')
  Add ('sha256={0} bytes={1} written={2:yyyy-MM-dd HH:mm:ss}' -f (Get-FileHash $dll.FullName -Algorithm SHA256).Hash.Substring(0, 16), $dll.Length, $dll.LastWriteTime)
} catch { Add "unreadable: $($_.Exception.Message)" }

Section 'game settings (graphic settings.cfg)'
try {
  $cfg = @("$env:USERPROFILE\OneDrive\Documents\My Games\AFOP\graphic settings.cfg",
           "$env:USERPROFILE\Documents\My Games\AFOP\graphic settings.cfg") | Where-Object { Test-Path $_ } | Select-Object -First 1
  if ($cfg) {
    foreach ($line in (Get-Content $cfg)) {
      $m = [regex]::Match($line, '^\s*(?:\["([^"]+)"\]|(\w+))\s*=\s*([^,{]+),?\s*$')
      if ($m.Success) { Add ('cfg {0}={1}' -f ($m.Groups[1].Value + $m.Groups[2].Value), $m.Groups[3].Value.Trim()) }
    }
  } else { Add 'not found' }
} catch { Add "unreadable: $($_.Exception.Message)" }

Section 'mod settings (AvatarVR_settings.ini)'
try { Get-Content (Join-Path $scripts 'AvatarVR_settings.ini') | Where-Object { $_ -notmatch '^\s*;' } | ForEach-Object { Add "ini $_" } }
catch { Add "unreadable: $($_.Exception.Message)" }

Section 'switch files'
try {
  Get-ChildItem $scripts -Filter 'AvatarVR_*.txt' -File | Sort-Object Name | ForEach-Object {
    $first = if ($_.Length -gt 0 -and $_.Length -lt 400) { (Get-Content $_.FullName -TotalCount 1) } else { '' }
    Add ('switch {0} bytes={1} first="{2}"' -f $_.Name, $_.Length, $first)
  }
} catch {}

Section 'windows'
try { Add ('power plan=' + ((powercfg /getactivescheme) -replace '^.*\(([^)]*)\).*$', '$1')) } catch {}
try {
  $hags = (Get-ItemProperty 'HKLM:\SYSTEM\CurrentControlSet\Control\GraphicsDrivers' -ErrorAction Stop).HwSchMode
  Add ('gpu scheduling (HwSchMode)={0} (2 = on, 1 = off)' -f $hags)
} catch { Add 'gpu scheduling (HwSchMode)=not set' }
try { Add ('game mode=' + (Get-ItemProperty 'HKCU:\Software\Microsoft\GameBar' -ErrorAction Stop).AutoGameModeEnabled) } catch {}
try {
  Get-CimInstance Win32_VideoController | ForEach-Object {
    Add ('display {0} {1}x{2} @ {3} Hz driver {4}' -f $_.Name, $_.CurrentHorizontalResolution, $_.CurrentVerticalResolution, $_.CurrentRefreshRate, $_.DriverVersion)
  }
} catch {}
try {
  $os = Get-CimInstance Win32_OperatingSystem
  Add ('memory free={0:N0} MiB of {1:N0} MiB' -f ($os.FreePhysicalMemory / 1024), ($os.TotalVisibleMemorySize / 1024))
} catch {}

Section 'game process'
try {
  Get-Process -Name afop -ErrorAction SilentlyContinue | ForEach-Object {
    Add ('afop pid={0} priority={1} started={2:HH:mm:ss}' -f $_.Id, $_.PriorityClass, $_.StartTime)
  }
  # 2026-11-11: launch arguments (engine options such as -noasynchcompute are read from them; INFERENCE).
  Get-CimInstance Win32_Process -Filter "Name='afop.exe'" -ErrorAction SilentlyContinue | ForEach-Object {
    Add ('afop command line: ' + $_.CommandLine)
  }
} catch {}

Section 'other programs that can use the GPU or hook the game'
try {
  $pattern = 'OFXR|Razer|Cortex|GameBar|ShadowPlay|nvcontainer|NVIDIA|sunshine|obs64|Medal|Overwolf|Discord|steam|upc|Uplay|VirtualDesktop|chrome|msedge|firefox|SearchHost|Widgets|Teams|RTSS|MSIAfterburner|PresentMon|HWiNFO|wallpaper'
  Get-Process | Where-Object { $_.ProcessName -match $pattern } | Group-Object ProcessName | Sort-Object Name |
    ForEach-Object { Add ('program {0} x{1}' -f $_.Name, $_.Count) }
} catch {}
try {
  # A WebView2 GPU process shows up in nvidia-smi as "msedgewebview2"; the host program is in its command line.
  Get-CimInstance Win32_Process -Filter "Name='msedgewebview2.exe'" | ForEach-Object {
    $h = [regex]::Match([string]$_.CommandLine, '--webview-exe-name=(\S+)')
    $t = [regex]::Match([string]$_.CommandLine, '--type=([\w-]+)')
    if ($h.Success -and $t.Success -and $t.Groups[1].Value -eq 'gpu-process') {
      Add ('webview2 gpu process pid={0} host={1}' -f $_.ProcessId, $h.Groups[1].Value)
    }
  }
} catch {}
try {
  $smi = (Get-Command nvidia-smi -ErrorAction SilentlyContinue).Source
  if (-not $smi) { $smi = 'C:\Windows\System32\nvidia-smi.exe' }
  Add ('encoder sessions/fps=' + (& $smi --query-gpu=encoder.stats.sessionCount,encoder.stats.averageFps --format=csv,noheader,nounits 2>&1))
} catch {}

Set-Content -Path $LogPath -Value $out -Encoding ascii
