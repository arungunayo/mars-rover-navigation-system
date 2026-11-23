$ErrorActionPreference='Stop'
. "$PSScriptRoot/terrain-profile.ps1"
function Assert-Profile([double[]]$values,[int]$width,[int]$height,[double]$spacing,[double]$limit) {
    for($y=0;$y -lt $height;$y++) { for($x=0;$x -lt $width;$x++) {
        $i=$y*$width+$x
        foreach($dy in 0..1) { foreach($dx in -1..1) {
            if(($dx -eq 0 -and $dy -eq 0) -or $x+$dx -lt 0 -or $x+$dx -ge $width -or $y+$dy -ge $height) { continue }
            $grade=[Math]::Abs($values[$i]-$values[($y+$dy)*$width+$x+$dx])/($spacing*[Math]::Sqrt($dx*$dx+$dy*$dy))
            if($grade -gt $limit+1e-9) { throw "Slope bound failed: $grade > $limit" }
        } }
        if($x -lt $width-1 -and $y -lt $height-1) {
            $gx=($values[$i+1]-$values[$i])/$spacing; $gy=($values[$i+$width]-$values[$i])/$spacing
            if([Math]::Sqrt($gx*$gx+$gy*$gy) -gt $limit+1e-9) { throw 'Terrain triangle is too steep' }
        }
    } }
}
$peak=[double[]]::new(25); $peak[12]=10
$result=ConvertToConnectedTerrain $peak 5 5 1 .25
Assert-Profile $result 5 5 1 .25
if([Math]::Abs(($result | Measure-Object -Sum).Sum-10) -gt 1e-9 -or $peak[12] -ne 10) { throw 'Altitude mean or input preservation failed' }
$plane=[double[]]::new(25)
for($y=0;$y -lt 5;$y++) { for($x=0;$x -lt 5;$x++) { $plane[$y*5+$x]=3+.05*$x+.05*$y } }
$same=ConvertToConnectedTerrain $plane 5 5 1 .25
for($i=0;$i -lt 25;$i++) { if([Math]::Abs($same[$i]-$plane[$i]) -gt 1e-9) { throw 'Already gentle plane was changed' } }
$profile=Get-Content "$PSScriptRoot/../results/scenario/terrain_profile.json" -Raw | ConvertFrom-Json
Assert-Profile $profile.heights $profile.width $profile.height $profile.spacing $profile.maxGrade
if($profile.profile -eq 'original-reduced') {
    $original=Get-Content "$PSScriptRoot/../webots/terrain/moose_original.json" -Raw | ConvertFrom-Json
    $base=($original.heights | Measure-Object -Minimum).Minimum
    for($i=0;$i -lt $profile.heights.Count;$i++) {
        $expected=$base+($original.heights[$i]-$base)*$profile.heightScale
        if([Math]::Abs($profile.heights[$i]-$expected) -gt 1e-10) { throw 'Original terrain variation was changed' }
    }
    $before=Get-TerrainSlopeStats $original.heights $original.width $original.height $original.spacing
    $after=Get-TerrainSlopeStats $profile.heights $profile.width $profile.height $profile.spacing
    if([Math]::Abs($before.maxGridAngle-$after.maxGridAngle-$profile.slopeReductionDegrees) -gt 1e-9) { throw 'Requested slope reduction was not achieved' }
    Write-Output '[TEST] original shape preserved by uniform height scaling; requested maximum grid slope reduction: passed'
}
$span=$profile.heights | Measure-Object -Minimum -Maximum
Write-Output "[TEST] peak smoothing, unchanged gentle plane, mean altitude, input preservation, actual terrain edges and triangles: passed"
Write-Output "[TERRAIN] min_height=$($span.Minimum) max_height=$($span.Maximum) elevation_span=$($span.Maximum-$span.Minimum) max_grade_bound=$($profile.maxGrade) max_angle_bound_deg=$([Math]::Atan($profile.maxGrade)*180/[Math]::PI)"
