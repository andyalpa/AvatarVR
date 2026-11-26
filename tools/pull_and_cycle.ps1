# Pull latest from origin, then run the build/deploy/launch cycle .
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
  git pull --ff-only
  if ($LASTEXITCODE -ne 0) {
    Write-Host 'git pull --ff-only failed. Resolve local changes and retry.'
    exit $LASTEXITCODE
  }
  & (Join-Path $PSScriptRoot 'run_cycle.ps1') @args
  exit $LASTEXITCODE
} finally {
  Pop-Location
}
