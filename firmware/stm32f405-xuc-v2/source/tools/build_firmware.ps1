param(
    [string]$CubeRoot,
    [string]$HalRoot,
    [string]$BuildDirectory,
    [ValidateSet('Debug', 'Release')]
    [string]$Configuration = 'Release'
)

$ErrorActionPreference = 'Stop'

$repository = Split-Path -Parent $PSScriptRoot
if (-not $CubeRoot) {
    $CubeRoot = Join-Path (Split-Path -Parent $repository) `
        'toolchains\STM32CubeF4-v1.24.1'
}
if (-not $BuildDirectory) {
    $BuildDirectory = Join-Path $repository 'build-arm'
}
if (-not $HalRoot) {
    $HalRoot = Join-Path (Split-Path -Parent $repository) `
        'toolchains\stm32f4-hal-cube-v1.14.0\Drivers\STM32F4xx_HAL_Driver'
}

$gccRoot = 'C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10'
$gcc = Join-Path $gccRoot 'bin\arm-none-eabi-gcc.exe'
$nm = Join-Path $gccRoot 'bin\arm-none-eabi-nm.exe'
if (-not (Test-Path -LiteralPath $gcc)) {
    throw "Required GCC 10.3.1 toolchain not found at $gccRoot"
}
if (-not (Test-Path -LiteralPath $CubeRoot)) {
    throw "STM32CubeF4 v1.24.1 not found at $CubeRoot"
}
if (-not (Test-Path -LiteralPath $HalRoot)) {
    throw "Legacy STM32F4 HAL package not found at $HalRoot"
}

$expectedCubeCommit = 'b5abca20c9676b04f8d2885a668a9b653ee65705'
$expectedHalCommit = 'b6e8197518e97d3fe9243fe49a692033b9d7d734'
$halRepository = Split-Path -Parent (Split-Path -Parent $HalRoot)
$cubeCommit = (& git -C $CubeRoot rev-parse HEAD).Trim()
$halCommit = (& git -C $halRepository rev-parse HEAD).Trim()
if ($cubeCommit -ne $expectedCubeCommit) {
    throw "Unexpected STM32CubeF4 commit $cubeCommit; expected $expectedCubeCommit"
}
if ($halCommit -ne $expectedHalCommit) {
    throw "Unexpected legacy HAL commit $halCommit; expected $expectedHalCommit"
}

$toolchainFile = Join-Path $repository 'cmake\arm-none-eabi-gcc.cmake'
$env:ARM_GCC_ROOT = $gccRoot

Write-Host "REPOSITORY=$repository"
Write-Host "GCC_ROOT=$gccRoot"
Write-Host "CUBE_ROOT=$CubeRoot"
Write-Host "HAL_ROOT=$HalRoot"
Write-Host "BUILD_DIRECTORY=$BuildDirectory"
$gccVersion = (& $gcc --version | Select-Object -First 1)
if ($gccVersion -notmatch '10\.3\.1 20210824') {
    throw "Unexpected compiler version: $gccVersion"
}
Write-Host $gccVersion
Write-Host "CUBE_COMMIT=$cubeCommit"
Write-Host "HAL_COMMIT=$halCommit"

& cmake `
    --fresh `
    -S $repository `
    -B $BuildDirectory `
    -G Ninja `
    "-DCMAKE_TOOLCHAIN_FILE=$toolchainFile" `
    "-DARM_GCC_ROOT=$gccRoot" `
    "-DSTM32_CUBE_F4_ROOT=$CubeRoot" `
    "-DSTM32_HAL_F4_ROOT=$HalRoot" `
    "-DCMAKE_BUILD_TYPE=$Configuration"
if ($LASTEXITCODE -ne 0) {
    throw "CMake configure failed with exit code $LASTEXITCODE"
}

& cmake --build $BuildDirectory --target stm32f405_xuc_v2
if ($LASTEXITCODE -ne 0) {
    throw "Firmware build failed with exit code $LASTEXITCODE"
}

$elf = Join-Path $BuildDirectory 'stm32f405_xuc_v2.elf'
$bin = Join-Path $BuildDirectory 'stm32f405_xuc_v2.bin'
$hex = Join-Path $BuildDirectory 'stm32f405_xuc_v2.hex'
$manifestLines = @(
    "GCC=$gccVersion",
    "STM32_CUBE_F4_COMMIT=$cubeCommit",
    "STM32_LEGACY_HAL_COMMIT=$halCommit"
)
foreach ($artifact in @($elf, $bin, $hex)) {
    if (-not (Test-Path -LiteralPath $artifact)) {
        throw "Expected build artifact is missing: $artifact"
    }
    $hash = Get-FileHash -Algorithm SHA256 -LiteralPath $artifact
    Write-Host "ARTIFACT=$artifact"
    Write-Host "SHA256=$($hash.Hash)"
    $manifestLines += "SHA256_$([System.IO.Path]::GetFileName($artifact))=$($hash.Hash)"
}

$binaryBytes = [System.IO.File]::ReadAllBytes($bin)
$initialStackPointer = [BitConverter]::ToUInt32($binaryBytes, 0)
$resetVector = [BitConverter]::ToUInt32($binaryBytes, 4)
if ($initialStackPointer -ne 0x20020000) {
    throw ('Unexpected initial stack pointer 0x{0:X8}' -f $initialStackPointer)
}
if (($resetVector -lt 0x08000001) -or
    ($resetVector -ge 0x08100000) -or
    (($resetVector -band 1) -ne 1)) {
    throw ('Invalid reset vector 0x{0:X8}' -f $resetVector)
}

$symbols = & $nm -n $elf
foreach ($requiredSymbol in @(
    'Reset_Handler',
    'main',
    'UART4_IRQHandler',
    'USART6_IRQHandler',
    'USART2_IRQHandler',
    'SysTick_Handler',
    'vTaskStartScheduler'
)) {
    if (-not ($symbols | Select-String "\s$requiredSymbol$" -Quiet)) {
        throw "Required firmware symbol is missing: $requiredSymbol"
    }
}

$manifestLines += ('INITIAL_SP=0x{0:X8}' -f $initialStackPointer)
$manifestLines += ('RESET_VECTOR=0x{0:X8}' -f $resetVector)
$manifestPath = Join-Path $BuildDirectory 'build-manifest.txt'
$manifestLines | Set-Content -LiteralPath $manifestPath -Encoding ascii

Write-Host ('INITIAL_SP=0x{0:X8}' -f $initialStackPointer)
Write-Host ('RESET_VECTOR=0x{0:X8}' -f $resetVector)
Write-Host 'REQUIRED_SYMBOLS=PASS'
Write-Host "BUILD_MANIFEST=$manifestPath"

Write-Host 'STM32_FIRMWARE_BUILD=PASS'
