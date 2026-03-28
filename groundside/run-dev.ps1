$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$backendDir = Join-Path $root "backend"
$frontendDir = Join-Path $root "frontend"

function Get-PythonCommand {
  if ($env:PYTHON) {
    return $env:PYTHON
  }

  $candidates = @(
    (Join-Path $root ".venv\Scripts\python.exe"),
    (Join-Path (Split-Path $root -Parent) ".venv\Scripts\python.exe")
  )

  foreach ($candidate in $candidates) {
    if (Test-Path $candidate) {
      return $candidate
    }
  }

  if (Get-Command python -ErrorAction SilentlyContinue) {
    return "python"
  }

  if (Get-Command py -ErrorAction SilentlyContinue) {
    return "py"
  }

  throw "No Python interpreter found."
}

if (-not (Get-Command pnpm -ErrorAction SilentlyContinue)) {
  throw "pnpm is required but was not found in PATH."
}

function Start-LoggedProcess {
  param(
    [string]$Name,
    [string]$Color,
    [string]$FilePath,
    [string[]]$Arguments,
    [string]$WorkingDirectory
  )

  $startInfo = [System.Diagnostics.ProcessStartInfo]::new()
  $startInfo.FileName = $FilePath
  $startInfo.WorkingDirectory = $WorkingDirectory
  $startInfo.UseShellExecute = $false
  $startInfo.RedirectStandardOutput = $true
  $startInfo.RedirectStandardError = $true
  $startInfo.CreateNoWindow = $true

  $startInfo.Arguments = ($Arguments | ForEach-Object {
    if ($_ -match '\s') { '"' + $_ + '"' } else { $_ }
  }) -join ' '

  $process = [System.Diagnostics.Process]::new()
  $process.StartInfo = $startInfo

  $outputEvent = Register-ObjectEvent -InputObject $process -EventName OutputDataReceived -MessageData @{ Name = $Name; Color = $Color } -Action {
    if ($EventArgs.Data) {
      Write-Host "[$($Event.MessageData.Name)] $($EventArgs.Data)" -ForegroundColor $Event.MessageData.Color
    }
  }

  $errorEvent = Register-ObjectEvent -InputObject $process -EventName ErrorDataReceived -MessageData @{ Name = $Name; Color = $Color } -Action {
    if ($EventArgs.Data) {
      Write-Host "[$($Event.MessageData.Name)] $($EventArgs.Data)" -ForegroundColor $Event.MessageData.Color
    }
  }

  [void]$process.Start()
  $process.BeginOutputReadLine()
  $process.BeginErrorReadLine()

  [PSCustomObject]@{
    Name = $Name
    Process = $process
    OutputEvent = $outputEvent
    ErrorEvent = $errorEvent
  }
}

function Stop-LoggedProcess {
  param($Entry)

  foreach ($subscription in @($Entry.OutputEvent, $Entry.ErrorEvent)) {
    if ($subscription) {
      Unregister-Event -SubscriptionId $subscription.Id -ErrorAction SilentlyContinue
      Remove-Job -Id $subscription.Id -Force -ErrorAction SilentlyContinue
    }
  }

  if ($Entry.Process -and -not $Entry.Process.HasExited) {
    $Entry.Process.Kill($true)
  }
}

$python = Get-PythonCommand

Write-Host "[info] backend:  http://127.0.0.1:5000" -ForegroundColor Yellow
Write-Host "[info] frontend: http://127.0.0.1:5173" -ForegroundColor Yellow
Write-Host "[info] press Ctrl+C to stop both services" -ForegroundColor Yellow

$processes = @(
  (Start-LoggedProcess -Name "backend" -Color "Cyan" -FilePath $python -Arguments @("app.py") -WorkingDirectory $backendDir),
  (Start-LoggedProcess -Name "frontend" -Color "Green" -FilePath "cmd.exe" -Arguments @("/c", "pnpm", "dev") -WorkingDirectory $frontendDir)
)

try {
  while ($true) {
    foreach ($entry in $processes) {
      if ($entry.Process.HasExited) {
        throw "$($entry.Name) exited with code $($entry.Process.ExitCode)."
      }
    }

    Start-Sleep -Milliseconds 200
  }
}
finally {
  foreach ($entry in $processes) {
    Stop-LoggedProcess -Entry $entry
  }
}
