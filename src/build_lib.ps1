# Build script to create SphereTrace.lib static library

Write-Host "Building SphereTrace.lib static library..." -ForegroundColor Green
Write-Host ""

# Check if MSVC tools are available
$clPath = Get-Command cl.exe -ErrorAction SilentlyContinue
if (-not $clPath) {
    Write-Host "Error: MSVC compiler (cl.exe) not found in PATH." -ForegroundColor Red
    Write-Host "Please run this from a Visual Studio Developer Command Prompt," -ForegroundColor Yellow
    Write-Host "or run 'vcvarsall.bat' to set up the environment." -ForegroundColor Yellow
    exit 1
}

Write-Host "Compiling all source files..." -ForegroundColor Cyan

# Compile all .c files to object files
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

$compileArgs = @("/c", "/O2", "/W3", "/EHsc", "/MD") + $sourceFiles
& cl.exe $compileArgs

if ($LASTEXITCODE -ne 0) {
    Write-Host "Compilation failed!" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "Creating static library SphereTrace.lib..." -ForegroundColor Cyan

# Create the static library from all object files
$objectFiles = $sourceFiles -replace '\.c$', '.obj'
$libArgs = @("/OUT:SphereTrace.lib") + $objectFiles
& lib.exe $libArgs

if ($LASTEXITCODE -ne 0) {
    Write-Host "Library creation failed!" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "Cleaning up object files..." -ForegroundColor Cyan
Remove-Item *.obj -ErrorAction SilentlyContinue

Write-Host ""
Write-Host "Success! SphereTrace.lib has been created." -ForegroundColor Green

