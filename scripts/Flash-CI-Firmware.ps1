[CmdletBinding()]
param(
    [string]$Port = '',
    [switch]$ListOnly,
    [switch]$SelfTest
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$Repo = 'waveshareteam/ESP32-C6-Touch-AMOLED-2.06'
$DefaultStartIndex = 1
$Items = @(
    [pscustomobject]@{ Index = 1; Workflow = 'esp-idf-examples.yml'; Artifact = 'firmware-esp-idf-01-axp2101-v5.5.5'; Framework = 'esp-idf'; Version = 'v5.5.5' },
    [pscustomobject]@{ Index = 2; Workflow = 'esp-idf-examples.yml'; Artifact = 'firmware-esp-idf-01-axp2101-v6.0.2'; Framework = 'esp-idf'; Version = 'v6.0.2' },
    [pscustomobject]@{ Index = 3; Workflow = 'esp-idf-examples.yml'; Artifact = 'firmware-esp-idf-02-lvgl-demo-v9-v5.5.5'; Framework = 'esp-idf'; Version = 'v5.5.5' },
    [pscustomobject]@{ Index = 4; Workflow = 'esp-idf-examples.yml'; Artifact = 'firmware-esp-idf-02-lvgl-demo-v9-v6.0.2'; Framework = 'esp-idf'; Version = 'v6.0.2' },
    [pscustomobject]@{ Index = 5; Workflow = 'esp-idf-examples.yml'; Artifact = 'firmware-esp-idf-03-esp-brookesia-v5.5.5'; Framework = 'esp-idf'; Version = 'v5.5.5' },
    [pscustomobject]@{ Index = 6; Workflow = 'esp-idf-examples.yml'; Artifact = 'firmware-esp-idf-03-esp-brookesia-v6.0.2'; Framework = 'esp-idf'; Version = 'v6.0.2' },
    [pscustomobject]@{ Index = 7; Workflow = 'esp-idf-examples.yml'; Artifact = 'firmware-esp-idf-04-qmi8658-v5.5.5'; Framework = 'esp-idf'; Version = 'v5.5.5' },
    [pscustomobject]@{ Index = 8; Workflow = 'esp-idf-examples.yml'; Artifact = 'firmware-esp-idf-04-qmi8658-v6.0.2'; Framework = 'esp-idf'; Version = 'v6.0.2' },
    [pscustomobject]@{ Index = 9; Workflow = 'esp-idf-examples.yml'; Artifact = 'firmware-esp-idf-05-spec-analyzer-v5.5.5'; Framework = 'esp-idf'; Version = 'v5.5.5' },
    [pscustomobject]@{ Index = 10; Workflow = 'esp-idf-examples.yml'; Artifact = 'firmware-esp-idf-05-spec-analyzer-v6.0.2'; Framework = 'esp-idf'; Version = 'v6.0.2' },
    [pscustomobject]@{ Index = 11; Workflow = 'arduino-examples.yml'; Artifact = 'firmware-arduino-01-helloworld-3.3.11'; Framework = 'arduino-esp32'; Version = '3.3.11' },
    [pscustomobject]@{ Index = 12; Workflow = 'arduino-examples.yml'; Artifact = 'firmware-arduino-02-gfx-asciitable-3.3.11'; Framework = 'arduino-esp32'; Version = '3.3.11' },
    [pscustomobject]@{ Index = 13; Workflow = 'arduino-examples.yml'; Artifact = 'firmware-arduino-03-lvgl-pcf85063-simpletime-3.3.11'; Framework = 'arduino-esp32'; Version = '3.3.11' },
    [pscustomobject]@{ Index = 14; Workflow = 'arduino-examples.yml'; Artifact = 'firmware-arduino-04-lvgl-qmi8658-ui-3.3.11'; Framework = 'arduino-esp32'; Version = '3.3.11' },
    [pscustomobject]@{ Index = 15; Workflow = 'arduino-examples.yml'; Artifact = 'firmware-arduino-05-lvgl-axp2101-adc-data-3.3.11'; Framework = 'arduino-esp32'; Version = '3.3.11' },
    [pscustomobject]@{ Index = 16; Workflow = 'arduino-examples.yml'; Artifact = 'firmware-arduino-06-lvgl-arduino-v9-3.3.11'; Framework = 'arduino-esp32'; Version = '3.3.11' }
)
$SourceProjects = @(
    'examples/esp-idf/01_AXP2101', 'examples/esp-idf/01_AXP2101', 'examples/esp-idf/02_lvgl_demo_v9', 'examples/esp-idf/02_lvgl_demo_v9',
    'examples/esp-idf/03_esp-brookesia', 'examples/esp-idf/03_esp-brookesia', 'examples/esp-idf/04_qmi8658', 'examples/esp-idf/04_qmi8658',
    'examples/esp-idf/05_Spec_Analyzer', 'examples/esp-idf/05_Spec_Analyzer', 'examples/arduino/01_HelloWorld', 'examples/arduino/02_GFX_AsciiTable',
    'examples/arduino/03_LVGL_PCF85063_simpleTime', 'examples/arduino/04_LVGL_QMI8658_ui', 'examples/arduino/05_LVGL_AXP2101_ADC_Data', 'examples/arduino/06_LVGL_Arduino_v9'
)
for ($itemIndex = 0; $itemIndex -lt $Items.Count; ++$itemIndex) { $Items[$itemIndex] | Add-Member -NotePropertyName SourceProject -NotePropertyValue $SourceProjects[$itemIndex] }

function Test-Port([string]$Value) { return $Value -match '^COM\d+$' }
function Get-NextProgress([int]$CurrentIndex, [int[]]$ConfirmedIndexes, [int]$ItemCount) {
    if ($ItemCount -lt 1 -or $CurrentIndex -lt 1 -or $CurrentIndex -gt $ItemCount) { throw 'Progress indexes must be within the item range.' }
    $confirmed = @($ConfirmedIndexes + $CurrentIndex | Where-Object { $_ -ge 1 -and $_ -le $ItemCount } | Sort-Object -Unique)
    return [pscustomobject]@{ CurrentIndex = if ($CurrentIndex -eq $ItemCount) { $CurrentIndex } else { $CurrentIndex + 1 }; ConfirmedIndexes = $confirmed; Completed = $CurrentIndex -eq $ItemCount }
}
function Get-StateForFinalSha($Saved, [string]$ExpectedSha, [string]$DefaultPort) {
    if (-not $Saved -or -not $Saved.PSObject.Properties['FinalSha'] -or -not $Saved.PSObject.Properties['CurrentIndex'] -or -not $Saved.PSObject.Properties['ConfirmedIndexes'] -or [string]$Saved.FinalSha -ne $ExpectedSha) {
        return [pscustomobject]@{ CurrentIndex = $DefaultStartIndex; ConfirmedIndexes = @(); Port = $DefaultPort }
    }
    $index = [int]$Saved.CurrentIndex
    if ($index -lt 1 -or $index -gt $Items.Count) { throw "Saved CurrentIndex is outside 1..$($Items.Count)." }
    return [pscustomobject]@{ CurrentIndex = $index; ConfirmedIndexes = @($Saved.ConfirmedIndexes | ForEach-Object { [int]$_ } | Where-Object { $_ -ge 1 -and $_ -le $Items.Count } | Sort-Object -Unique); Port = $DefaultPort }
}
function Test-RelativePackagePath([string]$PackageRoot, [string]$RelativePath) {
    if ([string]::IsNullOrWhiteSpace($RelativePath) -or [System.IO.Path]::IsPathRooted($RelativePath)) { return $false }
    $root = [System.IO.Path]::GetFullPath($PackageRoot).TrimEnd([System.IO.Path]::DirectorySeparatorChar, [System.IO.Path]::AltDirectorySeparatorChar) + [System.IO.Path]::DirectorySeparatorChar
    $candidate = [System.IO.Path]::GetFullPath((Join-Path $PackageRoot $RelativePath))
    return $candidate.StartsWith($root, [System.StringComparison]::OrdinalIgnoreCase)
}

if ($SelfTest) {
    $current = $DefaultStartIndex; $confirmed = @(); $transitions = 0
    while ($current -lt $Items.Count) { $next = Get-NextProgress $current $confirmed $Items.Count; if ($next.Completed -or $next.CurrentIndex -ne ($current + 1)) { throw 'SelfTest expected one-item progress.' }; $current = $next.CurrentIndex; $confirmed = @($next.ConfirmedIndexes); $transitions++ }
    $last = Get-NextProgress $current $confirmed $Items.Count
    if (-not $last.Completed -or @($last.ConfirmedIndexes).Count -ne $Items.Count) { throw 'SelfTest did not complete every item.' }
    $reset = Get-StateForFinalSha ([pscustomobject]@{ FinalSha = 'different'; CurrentIndex = 4; ConfirmedIndexes = @(1,2,3); Port = 'ignored' }) 'expected' ''
    if ($reset.CurrentIndex -ne 1 -or @($reset.ConfirmedIndexes).Count -ne 0) { throw 'SelfTest did not reset state for a new SHA.' }
    $missing = Get-StateForFinalSha ([pscustomobject]@{ FinalSha = 'expected' }) 'expected' ''
    if ($missing.CurrentIndex -ne 1 -or @($missing.ConfirmedIndexes).Count -ne 0) { throw 'SelfTest did not reset incomplete state.' }
    if ((Test-RelativePackagePath 'C:\package' '..\escape.bin') -or (Test-RelativePackagePath 'C:\package' 'C:\escape.bin') -or -not (Test-RelativePackagePath 'C:\package' 'firmware\app.bin')) { throw 'SelfTest relative manifest path validation failed.' }
    Write-Output 'SELF_TEST_OK startIndex=1 transitions=15 completed=16'
    return
}
if ($ListOnly) {
    Write-Output 'finalSHA=resolved-at-runtime'
    Write-Output 'defaultPort=auto-detect-at-runtime'
    Write-Output "startIndex=$DefaultStartIndex"
    foreach ($item in $Items) { Write-Output ('{0}: workflow={1} run=resolved-at-runtime artifact={2}' -f $item.Index, $item.Workflow, $item.Artifact) }
    return
}
function Resolve-DefaultPort {
    $pnpPorts = @(Get-CimInstance Win32_PnPEntity -ErrorAction SilentlyContinue | Where-Object { $_.PNPDeviceID -match 'VID_303A&PID_1001' -and $_.Name -match '\(COM\d+\)' } | ForEach-Object { [regex]::Match($_.Name, '\((COM\d+)\)').Groups[1].Value } | Sort-Object -Unique)
    if ($pnpPorts.Count -eq 1) { return $pnpPorts[0] }
    throw 'Unable to identify exactly one ESP32-C6 USB serial port; pass -Port COMx.'
}

$RepoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$StateRoot = Join-Path $env:LOCALAPPDATA 'Waveshare\ESP32-C6-Touch-AMOLED-2.06\ci-firmware'
$StatePath = Join-Path $StateRoot 'state-v3.json'
function Resolve-Executable([string]$Name, [string[]]$Fallbacks) {
    $command = Get-Command $Name -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($command -and $command.Source) { return $command.Source }
    foreach ($candidate in $Fallbacks) { if (Test-Path -LiteralPath $candidate -PathType Leaf) { return $candidate } }
    throw "$Name was not found on PATH or in the supported fallback locations."
}
function Resolve-Git { return Resolve-Executable 'git' @((Join-Path ${env:ProgramFiles} 'Git\cmd\git.exe'), (Join-Path ${env:ProgramFiles} 'Git\bin\git.exe'), 'C:\Git\cmd\git.exe', 'D:\Git\cmd\git.exe') }
function Resolve-Gh { return Resolve-Executable 'gh' @((Join-Path ${env:ProgramFiles} 'GitHub CLI\gh.exe'), (Join-Path ${env:ProgramFiles} 'GitHub CLI\bin\gh.exe')) }
function Resolve-PythonWithEsptool {
    $command = Get-Command python -ErrorAction SilentlyContinue | Select-Object -First 1
    $candidates = @(); if ($command -and $command.Source) { $candidates += $command.Source }
    foreach ($programFiles in (@($env:ProgramFiles, ${env:ProgramFiles(x86)}) | Where-Object { $_ })) {
        $candidates += @(Get-ChildItem -Path (Join-Path $programFiles 'Python*') -File -Filter python.exe -ErrorAction SilentlyContinue | ForEach-Object FullName)
    }
    foreach ($root in @((Join-Path $env:USERPROFILE '.espressif\python_env'), 'C:\Espressif', 'D:\espressif')) {
        if (Test-Path -LiteralPath $root) { $candidates += @(Get-ChildItem -LiteralPath $root -Recurse -File -Filter python.exe -ErrorAction SilentlyContinue | Where-Object { $_.FullName -match '[\\/]python_env[\\/].+[\\/]Scripts[\\/]python\.exe$' } | ForEach-Object FullName) }
    }
    foreach ($candidate in @($candidates | Select-Object -Unique)) { & $candidate -c 'import esptool' 2>$null; if ($LASTEXITCODE -eq 0) { return $candidate } }
    throw 'No Python interpreter with esptool was found.'
}
function Resolve-FinalSha([string]$GitExe) {
    $sha = (& $GitExe -C $RepoRoot rev-parse HEAD 2>&1 | Out-String).Trim()
    if ($LASTEXITCODE -ne 0 -or $sha -notmatch '^[0-9a-fA-F]{40}$') { throw 'Unable to resolve a full local git HEAD SHA.' }
    return $sha.ToLowerInvariant()
}
function Assert-CleanWorktree([string]$GitExe) {
    $status = (& $GitExe -C $RepoRoot status --porcelain=v1 --untracked-files=all 2>&1 | Out-String)
    if ($LASTEXITCODE -ne 0) { throw 'Unable to determine whether the working tree is clean.' }
    if (-not [string]::IsNullOrWhiteSpace($status)) { throw 'Refusing to continue: the working tree has staged, unstaged, or untracked changes.' }
}
function Resolve-CurrentBranch([string]$GitExe) {
    $branch = (& $GitExe -C $RepoRoot symbolic-ref --quiet --short HEAD 2>&1 | Out-String).Trim()
    if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($branch)) { throw 'Refusing to continue: check out a non-detached branch first.' }
    return $branch
}
function Assert-ReadyPullRequest([string]$GhExe, [string]$Branch, [string]$FinalSha) {
    $raw = (& $GhExe pr list --repo $Repo --head $Branch --state open --limit 2 --json number,state,isDraft,headRefName,headRefOid 2>&1 | Out-String)
    if ($LASTEXITCODE -ne 0) { throw 'Unable to query the open pull request for the current branch.' }
    $pullRequests = @($raw | ConvertFrom-Json)
    if ($pullRequests.Count -ne 1) { throw 'Refusing to continue: the current branch must have exactly one open pull request.' }
    $pullRequest = $pullRequests[0]
    if ([string]$pullRequest.state -ine 'OPEN' -or [bool]$pullRequest.isDraft -or [string]$pullRequest.headRefName -ne $Branch) { throw 'Refusing to continue: the pull request must be open, ready for review, and belong to the current branch.' }
    $headRefOid = [string]$pullRequest.headRefOid
    if ($headRefOid -notmatch '^[0-9a-fA-F]{40}$' -or -not [string]::Equals($headRefOid, $FinalSha, [System.StringComparison]::OrdinalIgnoreCase)) { throw 'Refusing to continue: the pull request head must be the complete local HEAD SHA.' }
}
function Resolve-ArtifactRuns([string]$GhExe, [string]$FinalSha) {
    $runByWorkflow = @{}
    foreach ($workflow in @($Items.Workflow | Sort-Object -Unique)) {
        $raw = (& $GhExe run list --repo $Repo --workflow $workflow --commit $FinalSha --status success --limit 20 --json databaseId,headSha,createdAt 2>&1 | Out-String)
        if ($LASTEXITCODE -ne 0) { throw "Unable to list successful $workflow runs: $raw" }
        $runs = @($raw | ConvertFrom-Json | Where-Object { $_.headSha -eq $FinalSha } | Sort-Object createdAt -Descending)
        if ($runs.Count -lt 1) { throw "No successful $workflow run exists for local HEAD $FinalSha; refusing to use another artifact." }
        if ([string]$runs[0].headSha -ne $FinalSha) { throw "Resolved $workflow run does not match local HEAD $FinalSha." }
        $runByWorkflow[$workflow] = [string]$runs[0].databaseId
    }
    foreach ($item in $Items) { $item | Add-Member -NotePropertyName Run -NotePropertyValue $runByWorkflow[$item.Workflow] -Force }
}
function Ensure-StateRoot { if (-not (Test-Path -LiteralPath $StateRoot)) { New-Item -ItemType Directory -Path $StateRoot | Out-Null } }
function Read-State([string]$FinalSha) {
    $saved = if (Test-Path -LiteralPath $StatePath) { Get-Content -LiteralPath $StatePath -Raw | ConvertFrom-Json } else { $null }
    return Get-StateForFinalSha $saved $FinalSha $Port
}
function Save-State([int]$CurrentIndex, [int[]]$ConfirmedIndexes, [string]$SavedPort, [string]$FinalSha) {
    Ensure-StateRoot
    [pscustomobject]@{ CurrentIndex = $CurrentIndex; ConfirmedIndexes = @($ConfirmedIndexes | Sort-Object -Unique); Port = $SavedPort; UpdatedAt = (Get-Date).ToString('o'); Repository = $Repo; FinalSha = $FinalSha } | ConvertTo-Json | Set-Content -LiteralPath $StatePath -Encoding UTF8
}
function New-RunPaths {
    Ensure-StateRoot; $stamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'; $downloadRoot = Join-Path $StateRoot 'downloads'; $logRoot = Join-Path $StateRoot 'logs'
    foreach ($dir in @($downloadRoot, $logRoot)) { if (-not (Test-Path -LiteralPath $dir)) { New-Item -ItemType Directory -Path $dir | Out-Null } }
    $downloadDir = Join-Path $downloadRoot $stamp; $logPath = Join-Path $logRoot ($stamp + '.log')
    if ((Test-Path -LiteralPath $downloadDir) -or (Test-Path -LiteralPath $logPath)) { throw "Timestamp collision at $stamp; no existing files were changed." }
    New-Item -ItemType Directory -Path $downloadDir | Out-Null; New-Item -ItemType File -Path $logPath | Out-Null
    return [pscustomobject]@{ DownloadDir = $downloadDir; LogPath = $logPath }
}
function Add-RunLog([string]$Path, [string]$Text) { Add-Content -LiteralPath $Path -Value $Text -Encoding UTF8 }
function Find-PackageDirectory([string]$DownloadDir) {
    $zips = @(Get-ChildItem -LiteralPath $DownloadDir -Recurse -File -Filter '*.zip')
    foreach ($zip in $zips) { $destination = Join-Path $zip.DirectoryName ($zip.BaseName + '-unzipped'); if (Test-Path -LiteralPath $destination) { throw "Refusing to overwrite extraction directory: $destination" }; Expand-Archive -LiteralPath $zip.FullName -DestinationPath $destination -ErrorAction Stop }
    $manifests = @(Get-ChildItem -LiteralPath $DownloadDir -Recurse -File -Filter 'manifest.json')
    if ($manifests.Count -ne 1) { throw 'Expected exactly one manifest.json in the downloaded artifact.' }
    return $manifests[0].DirectoryName
}
function Test-PackageManifest([string]$PackageDir, $Item, [string]$FinalSha) {
    $manifestPath = Join-Path $PackageDir 'manifest.json'
    if (-not (Test-Path -LiteralPath $manifestPath -PathType Leaf)) { throw 'Package manifest.json is missing.' }
    $manifest = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
    if ($manifest.schema_version -ne 1 -or $manifest.board -ne 'ESP32-C6-Touch-AMOLED-2.06' -or $manifest.git_sha -ne $FinalSha -or $manifest.chip -ne 'esp32c6' -or $manifest.framework -ne $Item.Framework -or $manifest.framework_version -ne $Item.Version -or $manifest.source_project -ne $Item.SourceProject) { throw 'Package manifest identity does not match the selected item and local HEAD.' }
    if ($manifest.flash.baud -ne 921600 -or @($manifest.files).Count -lt 1) { throw 'Package manifest flash metadata is incomplete or unsafe.' }
    $plan = @(); $offsets = @{}
    foreach ($file in @($manifest.files)) {
        $relativePath = [string]$file.archive_path
        if (-not (Test-RelativePackagePath $PackageDir $relativePath) -or [string]$file.sha256 -notmatch '^[0-9a-fA-F]{64}$' -or [int64]$file.size -le 0) { throw "Manifest file metadata is unsafe: $relativePath" }
        $fullPath = Join-Path $PackageDir $relativePath
        if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) { throw "Manifest file is missing: $relativePath" }
        $actual = Get-FileHash -LiteralPath $fullPath -Algorithm SHA256
        if ($actual.Hash -ne [string]$file.sha256 -or [int64](Get-Item -LiteralPath $fullPath).Length -ne [int64]$file.size) { throw "Manifest checksum or size verification failed: $relativePath" }
        if ($null -ne $file.offset -and [string]$file.offset -ne '') {
            if ([string]$file.offset -notmatch '^0x[0-9a-fA-F]+$') { throw "Manifest flash offset is invalid: $relativePath" }
            $offset = [Convert]::ToInt64(([string]$file.offset).Substring(2), 16)
            if ($offsets.ContainsKey($offset) -or $offset + [int64]$file.size -gt 16MB) { throw "Manifest flash range is unsafe: $relativePath" }
            $offsets[$offset] = $true; $plan += [pscustomobject]@{ Offset = $offset; Size = [int64]$file.size; Path = $fullPath }
        }
    }
    if ($plan.Count -lt 1) { throw 'Package manifest contains no flashable files.' }
    $orderedPlan = @($plan | Sort-Object Offset)
    for ($index = 1; $index -lt $orderedPlan.Count; ++$index) {
        $previous = $orderedPlan[$index - 1]
        if ($previous.Offset + $previous.Size -gt $orderedPlan[$index].Offset) {
            throw 'Package manifest contains overlapping flash ranges.'
        }
    }
    return $orderedPlan
}
function Invoke-CurrentFlash($Item, [string]$SelectedPort, [string]$GhExe, [string]$PythonExe, [string]$FinalSha) {
    $paths = New-RunPaths; Add-RunLog $paths.LogPath "finalSHA=$FinalSha index=$($Item.Index) artifact=$($Item.Artifact) run=$($Item.Run) port=$SelectedPort"
    $downloadOutput = (& $GhExe run download $Item.Run --repo $Repo --name $Item.Artifact --dir $paths.DownloadDir 2>&1 | Out-String); $downloadExit = $LASTEXITCODE; Add-RunLog $paths.LogPath $downloadOutput
    if ($downloadExit -ne 0) { throw "Artifact download failed with exit code $downloadExit. Log: $($paths.LogPath)" }
    $packageDir = Find-PackageDirectory $paths.DownloadDir; $plan = Test-PackageManifest $packageDir $Item $FinalSha
    $flashArguments = @('-m', 'esptool', '--port', $SelectedPort, '--chip', 'esp32c6', '--baud', '921600', 'write_flash')
    foreach ($entry in $plan) { $flashArguments += ('0x{0:X}' -f $entry.Offset); $flashArguments += $entry.Path }
    $flashOutput = (& $PythonExe @flashArguments 2>&1 | Out-String); $flashExit = $LASTEXITCODE
    Add-RunLog $paths.LogPath $flashOutput
    $verified = ($flashExit -eq 0) -and $flashOutput.Contains('Hash of data verified')
    return [pscustomobject]@{ Success = $verified; Output = $flashOutput; LogPath = $paths.LogPath; Detail = if ($verified) { 'Flash completed and Hash of data verified was found.' } else { 'Flash did not meet the required exit-code and hash-verification condition.' } }
}

$GitExe = Resolve-Git; $FinalSha = Resolve-FinalSha $GitExe; Assert-CleanWorktree $GitExe; $Branch = Resolve-CurrentBranch $GitExe
$GhExe = Resolve-Gh; Assert-ReadyPullRequest $GhExe $Branch $FinalSha; $PythonExe = Resolve-PythonWithEsptool
if ([string]::IsNullOrWhiteSpace($Port)) { $Port = Resolve-DefaultPort }
$Port = $Port.Trim().ToUpperInvariant()
if (-not (Test-Port $Port)) { throw 'Port must be COM followed by digits, for example COMx.' }
Resolve-ArtifactRuns $GhExe $FinalSha
Add-Type -AssemblyName System.Windows.Forms; Add-Type -AssemblyName System.Drawing
$state = Read-State $FinalSha; $script:CurrentIndex = $state.CurrentIndex; $script:ConfirmedIndexes = @($state.ConfirmedIndexes); $script:CurrentFlashVerified = $false
$form = New-Object System.Windows.Forms.Form; $form.Text = 'CI Firmware Flasher'; $form.StartPosition = 'CenterScreen'; $form.ClientSize = New-Object System.Drawing.Size(820, 670); $form.FormBorderStyle = 'FixedDialog'; $form.MaximizeBox = $false
function Add-Label([string]$Text, [int]$X, [int]$Y, [int]$Width = 780) { $label = New-Object System.Windows.Forms.Label; $label.Text = $Text; $label.Location = New-Object System.Drawing.Point($X, $Y); $label.Size = New-Object System.Drawing.Size($Width, 20); $form.Controls.Add($label); return $label }
$repoLabel = Add-Label "Repository: $Repo" 15 15; $shaLabel = Add-Label "Final SHA: $FinalSha" 15 40; $portCaption = Add-Label 'Port:' 15 70 45
$portBox = New-Object System.Windows.Forms.TextBox; $portBox.Text = $state.Port; $portBox.Location = New-Object System.Drawing.Point(65, 67); $portBox.Size = New-Object System.Drawing.Size(110, 22); $form.Controls.Add($portBox)
$currentLabel = Add-Label '' 15 100; $statusLabel = Add-Label 'Status: Select Flash current to begin.' 15 125
$progressList = New-Object System.Windows.Forms.ListBox; $progressList.Font = New-Object System.Drawing.Font('Consolas', 9); $progressList.Location = New-Object System.Drawing.Point(15, 155); $progressList.Size = New-Object System.Drawing.Size(790, 250); $form.Controls.Add($progressList)
$outputBox = New-Object System.Windows.Forms.TextBox; $outputBox.Multiline = $true; $outputBox.ReadOnly = $true; $outputBox.ScrollBars = 'Both'; $outputBox.WordWrap = $false; $outputBox.Font = New-Object System.Drawing.Font('Consolas', 9); $outputBox.Location = New-Object System.Drawing.Point(15, 415); $outputBox.Size = New-Object System.Drawing.Size(790, 190); $form.Controls.Add($outputBox)
$flashButton = New-Object System.Windows.Forms.Button; $flashButton.Text = 'Flash current'; $flashButton.Location = New-Object System.Drawing.Point(15, 620); $flashButton.Size = New-Object System.Drawing.Size(145, 32); $form.Controls.Add($flashButton)
$confirmButton = New-Object System.Windows.Forms.Button; $confirmButton.Text = 'Mark PASS and flash next'; $confirmButton.Location = New-Object System.Drawing.Point(170, 620); $confirmButton.Size = New-Object System.Drawing.Size(215, 32); $confirmButton.Enabled = $false; $form.Controls.Add($confirmButton)
$exitButton = New-Object System.Windows.Forms.Button; $exitButton.Text = 'Exit'; $exitButton.Location = New-Object System.Drawing.Point(685, 620); $exitButton.Size = New-Object System.Drawing.Size(120, 32); $form.Controls.Add($exitButton)
function Update-CurrentDisplay { $item = $Items[$script:CurrentIndex - 1]; $currentLabel.Text = "Current: $($item.Index)/$($Items.Count) Artifact: $($item.Artifact) Run: $($item.Run)"; $progressList.Items.Clear(); foreach ($progressItem in $Items) { $prefix = if ($script:ConfirmedIndexes -contains $progressItem.Index) { '[PASS]' } elseif ($progressItem.Index -eq $script:CurrentIndex) { '[CURRENT]' } else { '[WAIT]' }; [void]$progressList.Items.Add(('{0} {1}: {2}' -f $prefix, $progressItem.Index, $progressItem.Artifact)) }; $progressList.SelectedIndex = $script:CurrentIndex - 1 }
function Set-Busy([bool]$Busy) { $complete = $script:CurrentIndex -eq $Items.Count -and $script:ConfirmedIndexes -contains $Items.Count; $flashButton.Enabled = (-not $Busy) -and (-not $complete); $confirmButton.Enabled = (-not $Busy) -and $script:CurrentFlashVerified -and (-not $complete); $exitButton.Enabled = -not $Busy; $portBox.Enabled = -not $Busy; $form.UseWaitCursor = $Busy; [System.Windows.Forms.Application]::DoEvents() }
function Flash-CurrentItem { $selectedPort = $portBox.Text.Trim().ToUpperInvariant(); if (-not (Test-Port $selectedPort)) { [System.Windows.Forms.MessageBox]::Show('Port must be COM followed by digits, for example COMx.', 'Invalid port') | Out-Null; return }; $script:CurrentFlashVerified = $false; Set-Busy $true; $item = $Items[$script:CurrentIndex - 1]; $statusLabel.Text = "Status: Flashing item $($item.Index) on $selectedPort..."; try { $result = Invoke-CurrentFlash $item $selectedPort $GhExe $PythonExe $FinalSha; $outputBox.Text = "Log: $($result.LogPath)`r`n`r`n$($result.Output)"; if ($result.Success) { Save-State $script:CurrentIndex $script:ConfirmedIndexes $selectedPort $FinalSha; $statusLabel.Text = "Status: $($result.Detail) Confirm after checking the device."; $script:CurrentFlashVerified = $true } else { $statusLabel.Text = "Status: $($result.Detail) Current item was not advanced. Log: $($result.LogPath)" } } catch { $outputBox.Text = $_ | Out-String; $statusLabel.Text = "Status: Error. Current item was not advanced. $($_.Exception.Message)" } finally { Set-Busy $false } }
$flashButton.Add_Click({ Flash-CurrentItem })
$confirmButton.Add_Click({ if (-not $script:CurrentFlashVerified) { return }; $selectedPort = $portBox.Text.Trim().ToUpperInvariant(); $next = Get-NextProgress $script:CurrentIndex $script:ConfirmedIndexes $Items.Count; $script:CurrentIndex = $next.CurrentIndex; $script:ConfirmedIndexes = @($next.ConfirmedIndexes); $script:CurrentFlashVerified = $false; Save-State $script:CurrentIndex $script:ConfirmedIndexes $selectedPort $FinalSha; Update-CurrentDisplay; if ($next.Completed) { Set-Busy $false; $statusLabel.Text = "Status: All $($Items.Count) items are confirmed."; return }; Flash-CurrentItem })
$exitButton.Add_Click({ $form.Close() }); $progressList.Add_SelectedIndexChanged({ if ($progressList.SelectedIndex -ne ($script:CurrentIndex - 1)) { $progressList.SelectedIndex = $script:CurrentIndex - 1 } })
Update-CurrentDisplay; if ($script:CurrentIndex -eq $Items.Count -and $script:ConfirmedIndexes -contains $Items.Count) { Set-Busy $false; $statusLabel.Text = "Status: All $($Items.Count) items are confirmed." }; [void]$form.ShowDialog()
