# AvatarVR build -> load-proof -> deploy -> launch cycle.
# Writes progress to build\cycle_status.txt and the full build output to build\cycle_build.log so a
# remote session can follow along. Never touches afop.exe, References\, version.dll or version.ini.
# The game is launched once and left for the user to close.
param([int]$MaxMinutes = 8, [switch]$Profile)
$ErrorActionPreference = 'Stop'
$root    = Split-Path -Parent $PSScriptRoot            # ...\rogue\AvatarVR
$build   = Join-Path $root 'build\playtest'
if (-not $env:AFOP_GAME_ROOT) { throw 'Set AFOP_GAME_ROOT to the installed game folder.' }
$game = (Resolve-Path -LiteralPath $env:AFOP_GAME_ROOT).Path
if (-not (Test-Path -LiteralPath (Join-Path $game 'afop.exe') -PathType Leaf)) {
  throw 'AFOP_GAME_ROOT does not contain afop.exe.'
}
New-Item -ItemType Directory -Path $build -Force | Out-Null
$status  = Join-Path $build 'cycle_status.txt'
$buildLog = Join-Path $build 'cycle_build.log'

function Say($msg) {
  $line = '{0:yyyy-MM-dd HH:mm:ss} {1}' -f (Get-Date), $msg
  Add-Content -Path $status -Value $line
  Write-Host $line
}

function Archive-Logs($tag) {
  # Copy observer logs into the build workspace so a later launch's rotation can never
  # overwrite a gameplay log again (run 7 was lost that way).
  $src = Join-Path $game 'scripts'
  $runs = Join-Path $build 'runs'
  $logs = @(Get-ChildItem $src -Filter '*.log' -File -ErrorAction SilentlyContinue)
  if ($logs.Count -eq 0) { return }
  $last = Get-ChildItem $runs -Directory -ErrorAction SilentlyContinue | Sort-Object Name | Select-Object -Last 1
  $newest = ($logs | Measure-Object LastWriteTime -Maximum).Maximum
  # "before": the last "after" already holds these logs unless the game was started outside this script since.
  if ($tag -eq 'before' -and $last -and $newest -le $last.LastWriteTime) {
    $dst = $last.FullName
  } else {
    $dst = Join-Path $runs ('{0:yyyyMMdd_HHmmss}_{1}' -f (Get-Date), $tag)
    New-Item -ItemType Directory -Path $dst -Force | Out-Null
    $logs | ForEach-Object { Copy-Item $_.FullName $dst -Force }
    Say "ARCHIVED $($logs.Count) logs -> $dst"
  }
  if ($tag -eq 'before') {
    # These logs have no .prev rotation in the mod and only ever grew, so each archive repeated every earlier launch.
    # Before a launch they move into the run folder and the new session starts them empty.
    $logs | Where-Object { $_.Name -match '^(AvatarVR|BONE_COPY_PROBE|CODEDUMP|HAND_GESTURES|MOUNT_CAMERA|MOVE_NUDGE|MOVE_PROBE|MOVE_ROOMSCALE|POSE_PHASE_PROBE|SKIN_PALETTE_PROBE|VR_CONTROLS|VR_MENU|VR_WEAPON)\.log$' } |
      ForEach-Object { Move-Item $_.FullName $dst -Force }
  }
  # Keep the newest 30 run folders here; older ones go to the archive next to the game folder (same drive), never deleted.
  $old = @(Get-ChildItem $runs -Directory -ErrorAction SilentlyContinue | Sort-Object Name | Select-Object -SkipLast 30)
  if ($old.Count -gt 0) {
    $keep = Join-Path (Split-Path -Parent $game) 'AFOP_archive\scripts\runs'
    New-Item -ItemType Directory -Path $keep -Force | Out-Null
    $old | ForEach-Object { Move-Item $_.FullName $keep -Force }
    Say "MOVED $($old.Count) old run folders -> $keep"
  }
}

function Native-Text($scripts) {
  # scripts\AvatarVR_settings.ini [native]; the old AvatarVR_native.txt until the first launch moves it there.
  $old = Join-Path $scripts 'AvatarVR_native.txt'
  if (Test-Path $old) { return Get-Content -Path $old -Raw }
  $store = Join-Path $scripts 'AvatarVR_settings.ini'
  if (-not (Test-Path $store)) { return '' }
  $m = [regex]::Match((Get-Content -Path $store -Raw), '(?ms)^\[native\]\r?\n(.*?)(?=^\[[a-z]+\]\r?$|\z)')
  if ($m.Success) { return $m.Groups[1].Value } else { return '' }
}

function Set-Native-Text($scripts, $value) {
  $old = Join-Path $scripts 'AvatarVR_native.txt'
  $store = Join-Path $scripts 'AvatarVR_settings.ini'
  if ((Test-Path $old) -or -not (Test-Path $store)) { Set-Content -Path $old -Value $value -NoNewline; return }
  $text = Get-Content -Path $store -Raw
  $block = "[native]`r`n$value`r`n"
  $rx = [regex]'(?ms)^\[native\]\r?\n.*?(?=^\[[a-z]+\]\r?$|\z)'
  if ($rx.IsMatch($text)) { $text = $rx.Replace($text, { param($m) $block }, 1) } else { $text += $block }
  Set-Content -Path $store -Value $text -NoNewline
}

function Report-Crashes($since) {
  # Read-only: Windows' own crash records for afop.exe (exception code, faulting module, offset).
  try {
    $events = @(Get-WinEvent -FilterHashtable @{ LogName = 'Application'; StartTime = $since } -ErrorAction Stop |
      Where-Object { ($_.ProviderName -eq 'Application Error' -or $_.ProviderName -eq 'Windows Error Reporting') -and $_.Message -match 'afop' } |
      Select-Object -First 6)
  } catch { $events = @() }
  if ($events.Count -eq 0) { Say "CRASH LOG: no afop.exe crash events since $($since.ToString('HH:mm'))"; return }
  foreach ($e in $events) {
    $msg = ($e.Message -replace '\s+', ' ')
    if ($msg.Length -gt 700) { $msg = $msg.Substring(0, 700) }
    Say ("CRASH LOG {0:HH:mm:ss} [{1}] {2}" -f $e.TimeCreated, $e.ProviderName, $msg)
  }
}

Set-Content -Path $status -Value ''
try {
  Say 'START'
  Report-Crashes ((Get-Date).AddHours(-3))
  $running = @(Get-CimInstance Win32_Process -Filter "Name='afop.exe'")
  if ($running.Count -gt 0) { Say "ABORT afop.exe already running (pid $($running[0].ProcessId)); close the game first"; exit 1 }

  $cmake = (Get-Command cmake -ErrorAction SilentlyContinue).Source
  if (-not $cmake) {
    $cands = @('C:\Program Files\CMake\bin\cmake.exe') +
      @(Get-ChildItem 'C:\Program Files\Microsoft Visual Studio\*\*\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' -ErrorAction SilentlyContinue | ForEach-Object FullName)
    $cmake = $cands | Where-Object { $_ -and (Test-Path $_) } | Select-Object -First 1
  }
  if (-not $cmake) { Say 'ABORT cmake not found'; exit 1 }
  Say "cmake=$cmake"

  # One-time unpack of the vendored OpenXR SDK (shipped as a single zip to keep transfers small).
  $xrDir = Join-Path $root 'third_party\openxr-sdk'
  $xrZip = Join-Path $root 'third_party\openxr-sdk_1.1.63.zip'
  if (-not (Test-Path (Join-Path $xrDir 'CMakeLists.txt'))) {
    if (-not (Test-Path $xrZip)) { Say 'ABORT third_party\openxr-sdk missing and no zip to unpack'; exit 1 }
    Expand-Archive -Path $xrZip -DestinationPath $xrDir -Force
    Say 'UNPACKED third_party\openxr-sdk from zip (first OpenXR build reconfigures and takes longer)'
  }

  if (-not (Test-Path -LiteralPath (Join-Path $build 'CMakeCache.txt'))) {
    & $cmake -S $root -B $build -G 'Visual Studio 18 2026' -A x64 *> $buildLog
    if ($LASTEXITCODE) { Say 'CONFIGURE FAILED (see cycle_build.log)'; exit 1 }
  }
  Push-Location $build
  & $cmake --build . --config Release --parallel *>> $buildLog
  $rc = $LASTEXITCODE
  Pop-Location
  if ($rc -ne 0) { Say "BUILD FAILED rc=$rc (see cycle_build.log)"; exit 1 }
  $dll = Join-Path $build 'bin\AvatarVR.dll'
  Say ("BUILD OK AvatarVR.dll bytes={0}" -f (Get-Item $dll).Length)

  $host_exe = Join-Path $build 'bin\load_proof_host.exe'
  & $host_exe *>> $buildLog
  $rc = $LASTEXITCODE
  if ($rc -ne 0) { Say "LOAD PROOF FAILED rc=$rc"; exit 1 }
  Say 'LOAD PROOF OK rc=0'

  Archive-Logs 'before'
  $installDir = Join-Path $game 'scripts'
  New-Item -ItemType Directory -Path $installDir -Force | Out-Null
  $rollback = Join-Path $build ('rollback\' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff'))
  New-Item -ItemType Directory -Path $rollback -Force | Out-Null
  foreach ($name in @('AvatarVR.dll', 'AvatarVR_settings.ini')) {
    $installed = Join-Path $installDir $name
    if (Test-Path -LiteralPath $installed) { Copy-Item -LiteralPath $installed -Destination $rollback }
  }
  $target = Join-Path $installDir 'AvatarVR.dll'
  Copy-Item -LiteralPath $dll -Destination $target -Force
  if ((Get-FileHash -LiteralPath $dll).Hash -ne (Get-FileHash -LiteralPath $target).Hash) {
    throw "Deployment hash mismatch. Backup: $rollback"
  }
  Say "ROLLBACK $rollback"
  Say ("DEPLOYED bytes={0}" -f (Get-Item (Join-Path $game 'scripts\AvatarVR.dll')).Length)

  $launchedAt = Get-Date
  # Without "autostart" the second engine view is created in gameplay by hand: launch with it off. With "autostart"
  # (mod menu: Stereo -> Auto start) the mod rebuilds it fresh when gameplay starts, so the file is kept.
  $scriptsDir = Join-Path $game 'scripts'
  $nativeText = Native-Text $scriptsDir
  if ($nativeText -match 'autostart') {
    Say ("native view control kept (autostart): {0}" -f $nativeText.Trim())
  } else {
    Set-Native-Text $scriptsDir 'observe'
    Say 'native view control reset to observe'
  }
  Start-Process -FilePath (Join-Path $game 'afop.exe') -WorkingDirectory $game
  Say 'LAUNCHED afop.exe (stub -> Ubisoft launcher -> game); close the game yourself when done'
  $deadline = (Get-Date).AddMinutes(3)
  $seen = $false
  while ((Get-Date) -lt $deadline) {
    Start-Sleep -Seconds 5
    $p = @(Get-CimInstance Win32_Process -Filter "Name='afop.exe'")
    if ($p.Count -gt 0) { Say "GAME RUNNING pids=$(($p | ForEach-Object ProcessId) -join ',')"; $seen = $true; break }
  }
  if (-not $seen) { Say 'GAME NOT SEEN within 3 min'; exit 1 }
  if ($Profile) {
    $profileDir = Join-Path $build ('profiles\' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
    New-Item -ItemType Directory -Path $profileDir -Force | Out-Null
    # GPU monitor (2026-11-09, read-only nvidia-smi log -> GPU_MONITOR.log, archived with the other logs); it stops
    # by itself when the game is gone. A failure to start it never stops the cycle.
    try {
      $gpuMon = Join-Path $root 'tools\perf\gpu_monitor.ps1'
      if (Test-Path $gpuMon) {
        Start-Process -FilePath 'powershell.exe' -WindowStyle Hidden -ArgumentList @('-NoProfile', '-ExecutionPolicy', 'Bypass',
          '-File', $gpuMon, '-LogPath', (Join-Path $profileDir 'GPU_MONITOR.log'), '-MaxMinutes', ($MaxMinutes + 5)) | Out-Null
        Say 'GPU MONITOR started (GPU_MONITOR.log)'
      }
    } catch { Say "GPU MONITOR not started: $($_.Exception.Message)" }
    # Run fingerprint (2026-11-11, read-only -> RUN_FINGERPRINT.log, archived with the other logs): game settings,
    # mod settings, DLL hash, power plan and other GPU programs at launch, for tools\perf\runs_report.py. Never stops the cycle.
    try {
      $fingerprint = Join-Path $root 'tools\perf\run_fingerprint.ps1'
      if (Test-Path $fingerprint) {
        Start-Process -FilePath 'powershell.exe' -WindowStyle Hidden -ArgumentList @('-NoProfile', '-ExecutionPolicy', 'Bypass',
          '-File', $fingerprint, '-Game', $game, '-LogPath', (Join-Path $profileDir 'RUN_FINGERPRINT.log')) | Out-Null
        Say 'RUN FINGERPRINT written (RUN_FINGERPRINT.log)'
      }
    } catch { Say "RUN FINGERPRINT not written: $($_.Exception.Message)" }
  }
  # Watchdog: GPU-safety cap. The game must never be left running.
  $stop = (Get-Date).AddMinutes($MaxMinutes)
  Say "WATCHDOG armed: game will be closed at $($stop.ToString('HH:mm:ss')) if still running"
  while ($true) {
    Start-Sleep -Seconds 3
    $p = @(Get-Process -Name afop -ErrorAction SilentlyContinue)
    if ($p.Count -eq 0) {
      # The stub hands off to the Ubisoft launcher before the real game starts; only a sustained
      # absence (15 s) counts as an exit.
      $gone = $true
      for ($i = 0; $i -lt 5; $i++) {
        Start-Sleep -Seconds 3
        if (@(Get-Process -Name afop -ErrorAction SilentlyContinue).Count -gt 0) { $gone = $false; break }
      }
      if ($gone) { Say 'GAME EXITED'; break }
      continue
    }
    if ((Get-Date) -ge $stop) {
      Say 'WATCHDOG timeout: asking game to close'
      $p | ForEach-Object { [void]$_.CloseMainWindow() }
      Start-Sleep -Seconds 20
      $p = @(Get-Process -Name afop -ErrorAction SilentlyContinue)
      if ($p.Count -gt 0) { Say 'WATCHDOG: forcing game to stop'; $p | Stop-Process -Force }
      Say 'GAME EXITED (watchdog)'
      break
    }
  }
  Start-Sleep -Seconds 2
  Report-Crashes $launchedAt
  Archive-Logs 'after'
  Say 'DONE'
} catch {
  Say "ERROR $($_.Exception.Message)"
  exit 1
}
