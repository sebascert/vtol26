param(
  [switch]$VerboseOutput
)

$ErrorActionPreference = "Stop"

$repoFirmware = Split-Path -Parent $PSScriptRoot
Set-Location $repoFirmware

function Get-PlatformIoCmd {
  if (Get-Command pio -ErrorAction SilentlyContinue) {
    return "pio"
  }

  if (Get-Command platformio -ErrorAction SilentlyContinue) {
    return "platformio"
  }

  throw "No se encontro PlatformIO CLI. Instala PlatformIO Core y asegurate de tener 'pio' en PATH."
}

$pioCmd = Get-PlatformIoCmd

Write-Host "== Smoke tests (PlatformIO native) =="
Write-Host "Directorio: $repoFirmware"
Write-Host "CLI: $pioCmd"

$args = @("test", "-e", "native")
if ($VerboseOutput) {
  $args += "-v"
}

& $pioCmd @args

if ($LASTEXITCODE -ne 0) {
  throw "Smoke tests fallaron (exit code: $LASTEXITCODE)."
}

Write-Host "Todos los smoke tests pasaron correctamente."
