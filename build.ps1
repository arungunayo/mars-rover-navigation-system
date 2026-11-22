$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot
New-Item -ItemType Directory -Force build | Out-Null
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (!(Test-Path $vswhere)) { throw 'Visual Studio Installer / vswhere not found' }
$vs = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (!$vs) { throw 'Install the Visual Studio Desktop development with C++ workload' }
$setup = Join-Path $vs 'VC\Auxiliary\Build\vcvars64.bat'
$command = '"' + $setup + '" && cl /nologo /std:c++17 /EHsc /O2 /W4 /I include src\main.cpp /Fe:build\rover_demo.exe /Fo:build\main.obj && cl /nologo /std:c++17 /EHsc /O2 /W4 /I include tests\planner_tests.cpp /Fe:build\planner_tests.exe /Fo:build\tests.obj'
& cmd.exe /d /s /c $command
if ($LASTEXITCODE -ne 0) { throw 'Visual Studio build failed' }
& ./build/planner_tests.exe
if ($LASTEXITCODE -ne 0) { throw 'Tests failed' }
& ./build/rover_demo.exe
if ($LASTEXITCODE -ne 0) { throw 'Demo failed' }
