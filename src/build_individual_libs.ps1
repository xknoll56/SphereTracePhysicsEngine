# Build script to create individual static libraries for each source file
# Automatically finds and sets up Visual Studio environment

Write-Host "Building individual static libraries for each source file..." -ForegroundColor Green
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

# List of all source files
$sourceFiles = @(
    "SphereTrace.c",
    "SphereTraceAI.c",
    "SphereTraceAllocator.c",
    "SphereTraceCollider.c",
    "SphereTraceColliderBox.c",
    "SphereTraceColliderPlane.c",
    "SphereTraceColliderSphere.c",
    "SphereTraceColliderTerrain.c",
    "SphereTraceColliderTriangle.c",
    "SphereTraceLists.c",
    "SphereTraceMaterial.c",
    "SphereTraceMath.c",
    "SphereTraceRigidBody.c",
    "SphereTraceSpacialPartition.c",
    "SphereTraceTag.c"
)

# Create a temporary batch file to set up environment and build
$buildCommands = @"
@echo off
call "$vcvarsPath" x64 >nul 2>&1
if errorlevel 1 call "$vcvarsPath" x86 >nul 2>&1

echo Compiling individual source files...
"@

# Add compile commands for each file
foreach ($sourceFile in $sourceFiles) {
    $buildCommands += "`ncl.exe /c /O2 /W3 /EHsc /MD $sourceFile"
    $buildCommands += "`nif errorlevel 1 ("
    $buildCommands += "`n    echo Compilation of $sourceFile failed!"
    $buildCommands += "`n    exit /b 1"
    $buildCommands += "`n)"
}

$buildCommands += "`n`necho."
$buildCommands += "`necho Creating individual static libraries..."

# Add library creation commands for each file
foreach ($sourceFile in $sourceFiles) {
    $baseName = [System.IO.Path]::GetFileNameWithoutExtension($sourceFile)
    $objFile = "$baseName.obj"
    $libFile = "lib\$baseName.lib"
    $buildCommands += "`necho Creating $libFile..."
    $buildCommands += "`nlib.exe /OUT:$libFile $objFile"
    $buildCommands += "`nif errorlevel 1 ("
    $buildCommands += "`n    echo Library creation for $libFile failed!"
    $buildCommands += "`n    exit /b 1"
    $buildCommands += "`n)"
}

$buildCommands += "`n`necho."
$buildCommands += "`necho Cleaning up object files..."
$buildCommands += "`ndel *.obj 2>nul"
$buildCommands += "`n`necho."
$buildCommands += "`necho Success! All individual static libraries have been created."

$tempBatPath = Join-Path $PSScriptRoot "temp_build_individual.bat"
$buildCommands | Out-File -FilePath $tempBatPath -Encoding ASCII

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
Write-Host "Success! All individual static libraries have been created in lib folder:" -ForegroundColor Green
foreach ($sourceFile in $sourceFiles) {
    $baseName = [System.IO.Path]::GetFileNameWithoutExtension($sourceFile)
    $libFile = Join-Path $libFolder "$baseName.lib"
    if (Test-Path $libFile) {
        $fileInfo = Get-Item $libFile
        Write-Host "  - lib\$baseName.lib ($([math]::Round($fileInfo.Length / 1KB, 2)) KB)" -ForegroundColor Cyan
    }
}

