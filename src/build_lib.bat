@echo off
echo Building SphereTrace.lib static library...
echo.

echo Compiling all source files...
cl.exe /c /O2 /W3 /EHsc /MD SphereTrace.c SphereTraceAI.c SphereTraceAllocator.c SphereTraceCollider.c SphereTraceColliderBox.c SphereTraceColliderPlane.c SphereTraceColliderSphere.c SphereTraceColliderTerrain.c SphereTraceColliderTriangle.c SphereTraceLists.c SphereTraceMaterial.c SphereTraceMath.c SphereTraceRigidBody.c SphereTraceSpacialPartition.c SphereTraceTag.c

if errorlevel 1 (
    echo Compilation failed!
    echo Please ensure you are running from a Visual Studio Developer Command Prompt.
    pause
    exit /b 1
)

echo.
echo Creating static library SphereTrace.lib...
lib.exe /OUT:SphereTrace.lib SphereTrace.obj SphereTraceAI.obj SphereTraceAllocator.obj SphereTraceCollider.obj SphereTraceColliderBox.obj SphereTraceColliderPlane.obj SphereTraceColliderSphere.obj SphereTraceColliderTerrain.obj SphereTraceColliderTriangle.obj SphereTraceLists.obj SphereTraceMaterial.obj SphereTraceMath.obj SphereTraceRigidBody.obj SphereTraceSpacialPartition.obj SphereTraceTag.obj

if errorlevel 1 (
    echo Library creation failed!
    pause
    exit /b 1
)

echo.
echo Cleaning up object files...
del *.obj 2>nul

echo.
echo Success! SphereTrace.lib has been created.
pause

