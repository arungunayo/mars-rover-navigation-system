param([string]$Controller = "rover_controller", [string]$WebotsHome = "$env:LOCALAPPDATA\Programs\Webots")
$ErrorActionPreference='Stop'
Set-Location $PSScriptRoot
$vswhere="${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$vs=& $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (!$vs) { throw 'Visual Studio C++ tools not found' }
$setup=Join-Path $vs 'VC\Auxiliary\Build\vcvars64.bat'
$include=Join-Path $WebotsHome 'include\controller\c'
$lib=Join-Path $WebotsHome 'lib\controller\Controller.lib'
$dest="webots\controllers\$Controller"
if (!(Test-Path $lib)) { throw "Controller library not found at $lib" }
$source=Join-Path $dest "$Controller.cpp"
$exe=Join-Path $dest "$Controller.exe"
$obj=Join-Path $dest "$Controller.obj"
$command='"'+$setup+'" && cl /nologo /std:c++17 /EHsc /O2 /W4 /I "'+$include+'" "'+$source+'" /Fe:"'+$exe+'" /Fo:"'+$obj+'" /link "'+$lib+'"'
& cmd.exe /d /s /c $command
if ($LASTEXITCODE -ne 0) { throw 'Webots controller build failed' }
Write-Host "Built $Controller. Open the matching world under webots/worlds."

