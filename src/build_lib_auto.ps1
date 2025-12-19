# Build script to create SphereTrace.lib static library
# Automatically finds and sets up Visual Studio environment

Write-Host "Building SphereTrace.lib static library..." -ForegroundColor Green
Write-Host ""

# Try to find Visual Studio installation
$vsPaths = @(
    "${env:ProgramFiles}\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat",
    "${env:ProgramFiles}\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvarsall.bat",
    "${env:ProgramFiles}\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvarsall.bat",
    "${env:ProgramFiles(x86)}\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat",
    "${env:ProgramFiles(x86)}\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvarsall.bat",
    "${env:ProgramFiles(x86)}\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvarsall.bat",
    "${env:ProgramFiles}\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat",
    "${env:ProgramFiles}\Microsoft Visual Studio\2019\Professional\VC\Auxiliary\Build\vcvarsall.bat",
    "${env:ProgramFiles}\Microsoft Visual Studio\2019\Enterprise\VC\Auxiliary\Build\vcvarsall.bat",
    "${env:ProgramFiles(x86)}\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat",
    "${env:ProgramFiles(x86)}\Microsoft Visual Studio\2019\Professional\VC\Auxiliary\Build\vcvarsall.bat",
    "${env:ProgramFiles(x86)}\Microsoft Visual Studio\2019\Enterprise\VC\Auxiliary\Build\vcvarsall.bat"
)

$vcvarsPath = $null
foreach ($path in $vsPaths) {
    if (Test-Path $path) {
        $vcvarsPath = $path
        break
    }
}

if (-not $vcvarsPath) {
    Write-Host "Error: Could not find Visual Studio installation." -ForegroundColor Red
    Write-Host "Please ensure Visual Studio with C++ tools is installed." -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Alternatively, you can:" -ForegroundColor Yellow
    Write-Host "1. Open a Visual Studio Developer Command Prompt and run build_lib.bat" -ForegroundColor Cyan
    Write-Host "2. Or use CMake (if installed) with the provided CMakeLists.txt" -ForegroundColor Cyan
    exit 1
}

Write-Host "Found Visual Studio at: $vcvarsPath" -ForegroundColor Cyan
Write-Host "Setting up environment..." -ForegroundColor Cyan

# Create lib folder if it doesn't exist
$libFolder = Join-Path $PSScriptRoot "lib"
if (-not (Test-Path $libFolder)) {
    New-Item -ItemType Directory -Path $libFolder | Out-Null
    Write-Host "Created lib folder" -ForegroundColor Cyan
}

# Create a temporary batch file to set up environment and build
$tempBat = @"
@echo off
call "$vcvarsPath" x64 >nul 2>&1
if errorlevel 1 call "$vcvarsPath" x86 >nul 2>&1

echo Compiling all source files...
cl.exe /c /O2 /W3 /EHsc /MD SphereTrace.c SphereTraceAI.c SphereTraceAllocator.c SphereTraceCollider.c SphereTraceColliderBox.c SphereTraceColliderPlane.c SphereTraceColliderSphere.c SphereTraceColliderTerrain.c SphereTraceColliderTriangle.c SphereTraceLists.c SphereTraceMaterial.c SphereTraceMath.c SphereTraceRigidBody.c SphereTraceSpacialPartition.c SphereTraceTag.c

if errorlevel 1 (
    echo Compilation failed!
    exit /b 1
)

echo.
echo Creating static library lib\SphereTrace.lib...
lib.exe /OUT:lib\SphereTrace.lib SphereTrace.obj SphereTraceAI.obj SphereTraceAllocator.obj SphereTraceCollider.obj SphereTraceColliderBox.obj SphereTraceColliderPlane.obj SphereTraceColliderSphere.obj SphereTraceColliderTerrain.obj SphereTraceColliderTriangle.obj SphereTraceLists.obj SphereTraceMaterial.obj SphereTraceMath.obj SphereTraceRigidBody.obj SphereTraceSpacialPartition.obj SphereTraceTag.obj

if errorlevel 1 (
    echo Library creation failed!
    exit /b 1
)

echo.
echo Cleaning up object files...
del *.obj 2>nul

echo.
echo Success! lib\SphereTrace.lib has been created.
"@

$tempBatPath = Join-Path $PSScriptRoot "temp_build.bat"
$tempBat | Out-File -FilePath $tempBatPath -Encoding ASCII

# Run the batch file
& cmd.exe /c $tempBatPath
$buildResult = $LASTEXITCODE

# Clean up
Remove-Item $tempBatPath -ErrorAction SilentlyContinue

if ($buildResult -ne 0) {
    Write-Host "Build failed!" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "Success! lib\SphereTrace.lib has been created." -ForegroundColor Green

