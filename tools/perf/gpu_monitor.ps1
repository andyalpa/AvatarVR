# GPU monitor for the fast-run investigation (2026-11-09, ISSUE-10 / dev notes "Fast-run investigation").
# Read-only: logs what nvidia-smi reports while the game runs, so runs at the same spot can be compared for GPU clock,
# power, throttling and other programs using the GPU (encoders, recorders, streamers, browsers).
#   every IntervalSec : gpu utilisation, graphics/memory clock, power, temperature, P-state, throttle reasons,
#                       memory used, video encoder sessions and fps
#   every ProcessSec  : the processes using the GPU (shader %, encoder %, decoder %), busy ones only
# Started by tools\run_cycle.ps1 once afop.exe is running; it stops by itself when the game is gone for 20 s or after
# MaxMinutes. Can also be run by hand:  powershell -File tools\perf\gpu_monitor.ps1
param(
  [string]$LogPath = 'H:\Gaming\UbisoftLibrary\AFOP\scripts\GPU_MONITOR.log',
  [int]$IntervalSec = 2,
  [int]$ProcessSec = 10,
  [string]$GameProcess = 'afop',
  [int]$MaxMinutes = 30
)
$ErrorActionPreference = 'Continue'
$smi = (Get-Command nvidia-smi -ErrorAction SilentlyContinue).Source
if (-not $smi) { $smi = 'C:\Windows\System32\nvidia-smi.exe' }

function Log($text) {
  Add-Content -Path $LogPath -Value ('{0:HH:mm:ss.fff} {1}' -f (Get-Date), $text) -Encoding ascii
}

# Throttle ("clock event") reason bits, from nvidia-smi's clocks_event_reasons documentation.
$reasons = [ordered]@{ 0x1 = 'idle'; 0x2 = 'app-clock'; 0x4 = 'sw-power-cap'; 0x8 = 'hw-slowdown'; 0x10 = 'sync-boost';
                       0x20 = 'sw-thermal'; 0x40 = 'hw-thermal'; 0x80 = 'hw-power-brake'; 0x100 = 'display-clock' }
function Reason-Names([string]$hex) {
  $value = 0L
  if (-not [Int64]::TryParse(($hex -replace '^0x', ''), [Globalization.NumberStyles]::HexNumber, $null, [ref]$value)) { return $hex }
  if ($value -eq 0) { return 'none' }
  $names = @()
  foreach ($bit in $reasons.Keys) { if ($value -band $bit) { $names += $reasons[$bit] } }
  if ($names.Count -eq 0) { return $hex }
  return ($names -join '+')
}

Set-Content -Path $LogPath -Value ('{0:yyyy-MM-dd HH:mm:ss} gpu monitor start (interval {1} s, processes every {2} s, stops when {3} is gone)' -f
  (Get-Date), $IntervalSec, $ProcessSec, $GameProcess) -Encoding ascii
$static = & $smi --query-gpu=name,driver_version,power.limit,clocks.max.graphics,clocks.max.memory --format=csv,noheader 2>&1
Log "gpu: $static"
Log 'columns: util% | graphics MHz | memory MHz | power W | temp C | pstate | throttle | memory used | encoder sessions/fps'
try {
  $plan = (powercfg /getactivescheme) -replace '^.*\(([^)]*)\).*$', '$1'
  Log "power plan: $plan"
} catch {}

# 2026-11-11: every video encoder session, once a second (owner pid, codec, size, fps), next to this log as
# ENCODER_SESSIONS.log. The headset streamer, a desktop streamer and a recorder each hold their own session.
$encLog = Join-Path (Split-Path -Parent $LogPath) 'ENCODER_SESSIONS.log'
$encProc = $null
$encNamed = @{}
try {
  $encProc = Start-Process -FilePath $smi -ArgumentList 'encodersessions' -WindowStyle Hidden -PassThru `
    -RedirectStandardOutput $encLog -RedirectStandardError ($encLog + '.err')
} catch { Log "encoder session log not started: $($_.Exception.Message)" }
function Stop-EncoderLog { if ($encProc -and -not $encProc.HasExited) { try { $encProc.Kill() } catch {} } }
function Name-Encoders {
  # Names each encoder session's owner once (the session list itself only has pids).
  try {
    foreach ($line in (Get-Content $encLog -Tail 8 -ErrorAction Stop)) {
      $c = ($line.Trim() -split '\s+')
      if ($c.Count -lt 8 -or $c[0] -eq '#' -or $encNamed.ContainsKey($c[2])) { continue }
      $encNamed[$c[2]] = $true
      $owner = (Get-CimInstance Win32_Process -Filter "ProcessId=$($c[2])" -ErrorAction SilentlyContinue).Name
      Log ('encoder session owner: pid {0} = {1} ({2} {3}x{4})' -f $c[2], $(if ($owner) { $owner } else { '?' }), $c[3], $c[4], $c[5])
    }
  } catch {}
}

$modulesLogged = $false
$monitorStart = Get-Date
$deadline = (Get-Date).AddMinutes($MaxMinutes)
$lastProcess = [DateTime]::MinValue
$webviewHost = @{}
$goneSince = $null
while ((Get-Date) -lt $deadline) {
  $row = & $smi --query-gpu=utilization.gpu,clocks.gr,clocks.mem,power.draw,temperature.gpu,pstate,clocks_event_reasons.active,memory.used,encoder.stats.sessionCount,encoder.stats.averageFps --format=csv,noheader,nounits 2>&1
  $f = ($row -split ',') | ForEach-Object { $_.Trim() }
  if ($f.Count -ge 10) {
    Log ('gpu {0,3}% | {1,4} MHz | {2,5} MHz | {3,6} W | {4,3} C | {5} | {6} | {7} MiB | enc {8}/{9} fps' -f
         $f[0], $f[1], $f[2], $f[3], $f[4], $f[5], (Reason-Names $f[6]), $f[7], $f[8], $f[9])
  } else {
    Log "gpu query failed: $row"
  }
  if (((Get-Date) - $lastProcess).TotalSeconds -ge $ProcessSec) {
    $lastProcess = Get-Date
    $busy = @()
    foreach ($line in (& $smi pmon -c 1 -s u 2>&1)) {
      if ($line -match '^\s*#' -or $line -notmatch '\S') { continue }
      $c = ($line.Trim() -split '\s+')
      if ($c.Count -lt 10) { continue }
      $sm = $c[3]; $enc = $c[5]; $dec = $c[6]; $name = ($c[9..($c.Count - 1)] -join ' ')
      if ($sm -ne '-' -or $enc -ne '-' -or $dec -ne '-') {
        # 2026-11-11: a WebView2 GPU process is named after its host program (looked up once per pid).
        if ($name -like 'msedgewebview2*') {
          if (-not $webviewHost.ContainsKey($c[1])) {
            $webviewHost[$c[1]] = '?'
            try {
              $cl = [string](Get-CimInstance Win32_Process -Filter "ProcessId=$($c[1])" -ErrorAction Stop).CommandLine
              $h = [regex]::Match($cl, '--webview-exe-name=(\S+)')
              if ($h.Success) { $webviewHost[$c[1]] = $h.Groups[1].Value }
            } catch {}
          }
          $name = 'msedgewebview2[' + $webviewHost[$c[1]] + ']'
        }
        $busy +=('{0}(pid {1}) sm {2} enc {3} dec {4}' -f $name, $c[1], $sm, $enc, $dec)
      }
    }
    Log ('processes on the GPU: ' + $(if ($busy.Count) { $busy -join '; ' } else { 'none busy' }))
    Name-Encoders
  }
  # 2026-11-11: once, 60 s in, every non-Windows module in the game (overlays, OpenXR API layers, ASI plugins).
  if (-not $modulesLogged -and ((Get-Date) - $monitorStart).TotalSeconds -ge 60) {
    $modulesLogged = $true
    try {
      $gp = Get-Process -Name $GameProcess -ErrorAction Stop | Select-Object -First 1
      $gp.Modules | Where-Object { $_.FileName -notmatch '\\Windows\\' } |
        ForEach-Object { Log ('game module: ' + $_.FileName) }
    } catch { Log "game modules not read: $($_.Exception.Message)" }
  }
  if (@(Get-Process -Name $GameProcess -ErrorAction SilentlyContinue).Count -eq 0) {
    if ($null -eq $goneSince) { $goneSince = Get-Date }
    elseif (((Get-Date) - $goneSince).TotalSeconds -ge 20) { Log "$GameProcess gone for 20 s; monitor stops"; Stop-EncoderLog; exit 0 }
  } else {
    $goneSince = $null
  }
  Start-Sleep -Seconds $IntervalSec
}
Stop-EncoderLog
Log "monitor stopped after $MaxMinutes minutes"
