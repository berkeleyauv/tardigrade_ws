param(
    [string]$SimPlayer = $env:SIM_PLAYER
)

$ErrorActionPreference = "Stop"
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
if ([string]::IsNullOrWhiteSpace($SimPlayer)) {
    $SimPlayer = Join-Path $RepoRoot "..\tardigrade_unity_world\Builds\Windows\TardigradeSim.exe"
}
if (-not (Test-Path $SimPlayer -PathType Leaf)) {
    throw "Unity player not found: $SimPlayer. Build it first or set SIM_PLAYER."
}

Push-Location $RepoRoot
try {
    docker compose -f docker/compose.yaml up -d tardigrade
    docker compose -f docker/compose.yaml exec -d tardigrade bash -lc `
        "source /opt/ros/foxy/setup.bash && source /ws/install/setup.bash && ros2 launch tardigrade_bringup unity_sil.launch.py"
    & $SimPlayer
}
finally {
    docker compose -f docker/compose.yaml exec -T tardigrade `
        pkill -f unity_sil.launch.py 2>$null
    Pop-Location
}
