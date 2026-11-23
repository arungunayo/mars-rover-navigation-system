param([double]$StartX=3.125,[double]$StartY=-1.5625,[double]$GoalX=-13.28125,[double]$GoalY=-14.90085,[int]$RockCount=12,[int]$Seed=43,[ValidateSet('connected','original','original-reduced')][string]$TerrainProfile='original-reduced',[double]$MaxTerrainGrade=.45,[double]$SlopeReductionDegrees=3,[double]$PlannerMaxGrade=.5)
# Reference: https://cyberbotics.com/doc/guide/moose
# Uses the demo's shared ElevationGrid visual/collision pattern and CFM/ERP,
# Connected profile spreads the original cliffs into traversable hills. The
# original height asset remains untouched; -TerrainProfile original restores it.
$ErrorActionPreference='Stop'
Set-Location $PSScriptRoot
$culture=[Globalization.CultureInfo]::InvariantCulture
function F([double]$v) { $v.ToString('0.000000',$culture) }
$source=Get-Content webots/terrain/moose_original.json -Raw | ConvertFrom-Json
$dim=[int]$source.width; $spacing=[double]$source.spacing; $origin=[double]$source.originX
. "$PSScriptRoot/scripts/terrain-profile.ps1"
$originalHeights=[double[]]$source.heights.Clone()
$heightScale=1.0; $originalStats=$null
if($TerrainProfile -eq 'connected') {
    $source.heights=ConvertToConnectedTerrain -Heights $source.heights -Width $dim -Height $dim -Spacing $spacing -MaxGrade $MaxTerrainGrade
    $heightScale=$null
} elseif($TerrainProfile -eq 'original-reduced') {
    $originalStats=Get-TerrainSlopeStats $originalHeights $dim $dim $spacing
    if($SlopeReductionDegrees -lt 0 -or $SlopeReductionDegrees -ge $originalStats.maxGridAngle) { throw 'Slope reduction must be nonnegative and below the original maximum grid slope' }
    $heightScale=[Math]::Tan(($originalStats.maxGridAngle-$SlopeReductionDegrees)*[Math]::PI/180)/$originalStats.maxEdgeGrade
    $base=($originalHeights | Measure-Object -Minimum).Minimum
    $scaled=[double[]]::new($originalHeights.Count)
    for($i=0;$i -lt $scaled.Count;$i++) { $scaled[$i]=$base+($originalHeights[$i]-$base)*$heightScale }
    $source.heights=$scaled
}
New-Item -ItemType Directory -Force results/scenario | Out-Null
$plannerMaxGrade=$PlannerMaxGrade
$stats=Get-TerrainSlopeStats $source.heights $dim $dim $spacing
$bound=[Math]::Max($stats.maxEdgeGrade,$stats.maxTriangleGrade)
@{ profile=$TerrainProfile; maxGrade=$bound; connectedGrade=$MaxTerrainGrade; plannerMaxGrade=$plannerMaxGrade; slopeReductionDegrees=$SlopeReductionDegrees; heightScale=$heightScale; maxGridAngle=$stats.maxGridAngle; width=$dim; height=$dim; spacing=$spacing; originX=$source.originX; originY=$source.originY; originZ=$source.originZ; heights=$source.heights } | ConvertTo-Json -Depth 5 -Compress | Set-Content results/scenario/terrain_profile.json
function Height([double]$x,[double]$y) {
    $gx=[Math]::Clamp(($x-$source.originX)/$spacing,0,$dim-1.000001)
    $gy=[Math]::Clamp(($y-$source.originY)/$spacing,0,$dim-1.000001)
    $cx=[int][Math]::Floor($gx); $cy=[int][Math]::Floor($gy); $u=$gx-$cx; $v=$gy-$cy
    $h00=[double]$source.heights[$cy*$dim+$cx]; $h10=[double]$source.heights[$cy*$dim+$cx+1]
    $h01=[double]$source.heights[($cy+1)*$dim+$cx]; $h11=[double]$source.heights[($cy+1)*$dim+$cx+1]
    (1-$v)*(($h00*(1-$u))+$h10*$u)+$v*(($h01*(1-$u))+$h11*$u)+[double]$source.originZ
}
foreach($v in @($StartX,$StartY,$GoalX,$GoalY)) {
    if($v -lt -23 -or $v -gt 23) { throw 'Keep mission points within [-23,23] metres' }
}
$heights=[Collections.Generic.List[string]]::new()
$points=[Collections.Generic.List[string]]::new()
$lines=[Collections.Generic.List[string]]::new()
for($row=0;$row -lt $dim;$row++) {
    for($col=0;$col -lt $dim;$col++) {
        $x=$origin+$col*$spacing; $y=$origin+$row*$spacing; $z=Height $x $y
        $heights.Add(([double]$source.heights[$row*$dim+$col]).ToString("R",$culture))
        $points.Add("$(F $x) $(F $y) $(F ($z+.06))")
    }
}
# One visible line per original DEM row and column, at 0.78125 m spacing.
for($row=0;$row -lt $dim;$row++) { $ids=for($col=0;$col -lt $dim;$col++) { $row*$dim+$col }; $lines.Add(($ids -join ' ')+ ' -1') }
for($col=0;$col -lt $dim;$col++) { $ids=for($row=0;$row -lt $dim;$row++) { $row*$dim+$col }; $lines.Add(($ids -join ' ')+ ' -1') }
$spawn=F ((Height $StartX $StartY)+.16)
$startZ=F ((Height $StartX $StartY)+.025); $goalZ=F ((Height $GoalX $GoalY)+.025)
$random=[Random]::new($Seed)
$rocks=[Collections.Generic.List[string]]::new()
$placements=[Collections.Generic.List[object]]::new()
$rockFaces='0 5 4 3 2 1 -1, '
for($i=0;$i -lt 6;$i++) { $j=($i+1)%6; $rockFaces+="$i $j $($j+6) $($i+6) -1, $($i+6) $($j+6) 12 -1, " }
for($attempt=0;$placements.Count -lt $RockCount -and $attempt -lt 5000;$attempt++) {
    # Scatter half the rocks near the mission corridor, the rest across the map.
    if($placements.Count -lt [Math]::Min(6,[Math]::Ceiling($RockCount/2))) {
        $t=.18+.64*$random.NextDouble(); $offset=($random.NextDouble()-.5)*5
        $length=[Math]::Sqrt([Math]::Pow($GoalX-$StartX,2)+[Math]::Pow($GoalY-$StartY,2))
        $rx=$StartX+($GoalX-$StartX)*$t-($GoalY-$StartY)/$length*$offset
        $ry=$StartY+($GoalY-$StartY)*$t+($GoalX-$StartX)/$length*$offset
    } else { $rx=-21+42*$random.NextDouble(); $ry=-21+42*$random.NextDouble() }
    if([Math]::Sqrt([Math]::Pow($rx-$StartX,2)+[Math]::Pow($ry-$StartY,2)) -lt 2 -or [Math]::Sqrt([Math]::Pow($rx-$GoalX,2)+[Math]::Pow($ry-$GoalY,2)) -lt 2) { continue }
    $near=$false; foreach($placed in $placements) { if([Math]::Sqrt([Math]::Pow($rx-$placed.x,2)+[Math]::Pow($ry-$placed.y,2)) -lt 2.1) { $near=$true; break } }; if($near) { continue }
    $radius=.27+.16*$random.NextDouble(); $rockHeight=1.3+.4*$random.NextDouble()
    $vertices=[Collections.Generic.List[string]]::new()
    for($layer=0;$layer -lt 2;$layer++) { for($i=0;$i -lt 6;$i++) {
        $angle=$i*[Math]::PI/3; $rr=$radius*(.8+.2*$layer)*(.85+.3*$random.NextDouble())
        $vertices.Add("$(F ($rr*[Math]::Cos($angle))) $(F ($rr*[Math]::Sin($angle))) $(F ($rockHeight*.45*$layer))")
    } }
    $vertices.Add("0 0 $(F $rockHeight)")
    # Bury the base slightly to avoid floating on sloped triangles.
    $z=Height $rx $ry
    foreach($ox in @(-$radius,$radius)) { foreach($oy in @(-$radius,$radius)) { $z=[Math]::Min($z,(Height ($rx+$ox) ($ry+$oy))) } }
    $id=$placements.Count
    $rocks.Add(@"
DEF ROCK_$id Solid {
  translation $(F $rx) $(F $ry) $(F ($z-.08))
  rotation 0 0 1 $(F ($random.NextDouble()*6.283185307))
  name "Random rock $id"
  children [ Shape {
    appearance PBRAppearance { baseColor .32 .29 .26 roughness 1 metalness 0 }
    geometry DEF ROCK_MESH_$id IndexedFaceSet { coord Coordinate { point [ $($vertices -join ', ') ] } coordIndex [ $rockFaces ] creaseAngle .15 }
  } ]
  boundingObject USE ROCK_MESH_$id
  locked TRUE
}
"@)
    $placements.Add([pscustomobject]@{id=$id;x=$rx;y=$ry;height=$rockHeight;radius=$radius})
}
if($placements.Count -ne $RockCount) { throw 'Unable to place requested rocks with start/goal clearance; reduce RockCount' }
New-Item -ItemType Directory -Force results/scenario | Out-Null
$placements | Export-Csv results/scenario/rocks.csv -NoTypeInformation
$world=@"
#VRML_SIM R2025a utf8
EXTERNPROTO "../protos/Sojourner.proto"
WorldInfo {
  title "Moose terrain - $TerrainProfile slope profile"
  info [ "Moose-derived ElevationGrid; profile=$TerrainProfile. White grid, blue discovered cells, orange sensing circle, live LIDAR panel." ]
  basicTimeStep 16
  gravity 3.73
  CFM 0.0003
  ERP 0.4
}
Viewpoint {
  orientation -0.2231159133 0.0582567053 0.9730495596 2.643997548
  position $(F ($StartX+10.76725)) $(F ($StartY-5.473)) $(F ((Height $StartX $StartY)+6.568))
  follow "Sojourner"
  followType "Tracking Shot"
}
Background { skyColor [ 0.12 0.16 0.22 ] }
DirectionalLight { direction -1 -1 -2 intensity 1 }
DEF TERRAIN Solid {
  translation -25 -25 -0.01
  children [ Shape {
    appearance PBRAppearance { baseColor 0.55 0.32 0.18 roughness 1 metalness 0 }
    geometry DEF TERRAIN_HEIGHTS ElevationGrid {
      xDimension $dim yDimension $dim xSpacing $spacing ySpacing $spacing
      height [ $($heights -join ' ') ]
      thickness 1
    }
  } ]
  name "Uneven terrain"
  boundingObject USE TERRAIN_HEIGHTS
  locked TRUE
}
DEF GRID_OVERLAY Shape {
  appearance Appearance { material Material { emissiveColor 0.9 0.9 0.75 } }
  geometry IndexedLineSet {
    coord Coordinate { point [ $($points -join ', ') ] }
    coordIndex [ $($lines -join ', ') ]
  }
}
DEF START Solid {
  translation $(F $StartX) $(F $StartY) $startZ
  children [
    Shape { appearance PBRAppearance { baseColor 0.05 0.9 0.3 roughness 1 metalness 0 } geometry Cylinder { radius .4 height .015 } }
  ]
  name "START marker (visual only)"
}
DEF GOAL Solid {
  translation $(F $GoalX) $(F $GoalY) $goalZ
  children [
    Shape { appearance PBRAppearance { baseColor 1 0.12 0.12 roughness 1 metalness 0 } geometry Cylinder { radius .4 height .015 } }
  ]
  name "GOAL marker (visual only)"
}
DEF ROVER Sojourner {
  translation $(F $StartX) $(F $StartY) $spawn
  controller "rover_terrain_controller"
  customData "maxGrade=$(F $plannerMaxGrade)"
  wheelMaxSpeed 6
  supervisor TRUE
  extensionSlot [
    Display { name "lidar_view" width 240 height 240 }
    Display { name "discovery_view" width 320 height 320 }
    GPS { name "gps" }
    InertialUnit { name "imu" }
    Lidar { translation 0 0 .8 name "lidar" horizontalResolution 360 fieldOfView 6.283185307 numberOfLayers 1 minRange .1 maxRange 4 }
  ]
}
$($rocks -join "`n")
"@
Set-Content -LiteralPath webots/worlds/rover_terrain.wbt -Value $world -Encoding utf8
Write-Host "Generated rover_terrain.wbt: $($dim*$dim) terrain vertices; START=($StartX,$StartY) GOAL=($GoalX,$GoalY)"
Write-Host "Rocks=$RockCount seed=$Seed; cruise wheel speed=6 rad/s; terrain profile=$TerrainProfile"
Write-Host "START-to-GOAL separation=$([Math]::Sqrt([Math]::Pow($GoalX-$StartX,2)+[Math]::Pow($GoalY-$StartY,2))) m"
if($originalStats) { Write-Host "Original max grid slope=$($originalStats.maxGridAngle) degrees; reduced=$($stats.maxGridAngle) degrees; height scale=$heightScale" }


