param([switch]$PhysicalOnly)
$root=Split-Path $PSScriptRoot -Parent
$s=Get-Content "$root/webots/terrain/moose_original.json" -Raw | ConvertFrom-Json
$rocks=@(Import-Csv "$root/results/scenario/rocks.csv")
$world=Get-Content "$root/webots/worlds/rover_terrain.wbt" -Raw
$limitMatch=[regex]::Match($world,'maxGrade=([\d.eE+-]+)')
$slopeLimit=.3; if($limitMatch.Success) { $slopeLimit=[double]$limitMatch.Groups[1].Value }
$heightText=[regex]::Match($world,'height\s*\[([^\]]+)\]').Groups[1].Value
$s.heights=@($heightText -split '[,\s]+' | Where-Object {$_} | ForEach-Object {[double]::Parse($_,[Globalization.CultureInfo]::InvariantCulture)})
function MarkerNode([string]$name) {
  $match=[regex]::Match($world,"DEF $name Solid\s*\{\s*translation\s+([-\d.eE]+)\s+([-\d.eE]+)")
  if(!$match.Success) { throw "Marker $name not found" }
  [int][Math]::Round(([double]$match.Groups[2].Value+25)/.78125)*65+[int][Math]::Round(([double]$match.Groups[1].Value+25)/.78125)
}
$start=MarkerNode 'START'; $goal=MarkerNode 'GOAL'
$safe=[bool[]]::new(4225)
for($y=1;$y -lt 64;$y++) { for($x=1;$x -lt 64;$x++) {
  $i=$y*65+$x; $gx=($s.heights[$i+1]-$s.heights[$i-1])/1.5625; $gy=($s.heights[$i+65]-$s.heights[$i-65])/1.5625
  $safe[$i]=[Math]::Sqrt($gx*$gx+$gy*$gy) -le $slopeLimit
} }
$oldBlocked=[bool[]]::new(4225)
foreach($r in $rocks) { for($a=0;$a -lt 32;$a++) {
  $theta=$a*2*[Math]::PI/32; $radius=[double]$r.radius*1.15
  $cx=[int][Math]::Round(([double]$r.x+$radius*[Math]::Cos($theta)+25)/.78125)
  $cy=[int][Math]::Round(([double]$r.y+$radius*[Math]::Sin($theta)+25)/.78125)
  foreach($dy in -1..1) { foreach($dx in -1..1) { $x=$cx+$dx; $y=$cy+$dy; if($x -ge 0 -and $y -ge 0 -and $x -lt 65 -and $y -lt 65) { $oldBlocked[$y*65+$x]=$true } } }
} }
function CircleCollision([double]$ax,[double]$ay,[double]$bx,[double]$by) {
  foreach($r in $rocks) {
    $vx=$bx-$ax; $vy=$by-$ay; $len=$vx*$vx+$vy*$vy
    $t=0.0; if($len -gt 0) { $t=[Math]::Clamp((([double]$r.x-$ax)*$vx+([double]$r.y-$ay)*$vy)/$len,0,1) }
    $dx=$ax+$t*$vx-[double]$r.x; $dy=$ay+$t*$vy-[double]$r.y; $radius=[double]$r.radius*1.15+.65
    if($dx*$dx+$dy*$dy -le $radius*$radius) { return $true }
  }
  return $false
}
$modes=@('physical_clearance'); if(!$PhysicalOnly) { $modes=@('old_vertex_inflation','physical_clearance') }
foreach($mode in $modes) {
  $allowed=[bool[]]$safe.Clone()
  for($i=0;$i -lt 4225;$i++) { if($allowed[$i]) {
    $x=-25+($i%65)*.78125; $y=-25+[Math]::Floor($i/65)*.78125
    if(($mode -eq 'old_vertex_inflation' -and $oldBlocked[$i]) -or ($mode -eq 'physical_clearance' -and (CircleCollision $x $y $x $y))) { $allowed[$i]=$false }
  } }
  $seen=[bool[]]::new(4225); $q=[Collections.Generic.Queue[int]]::new(); $q.Enqueue($start); $seen[$start]=$true
  while($q.Count) {
    $u=$q.Dequeue(); $x=$u%65; $y=[int][Math]::Floor($u/65)
    foreach($dy in -1..1) { foreach($dx in -1..1) {
      $nx=$x+$dx; $ny=$y+$dy
      if(($dx -eq 0 -and $dy -eq 0) -or $nx -lt 1 -or $ny -lt 1 -or $nx -gt 63 -or $ny -gt 63) { continue }
      $v=$ny*65+$nx
      if($seen[$v] -or !$allowed[$v] -or ($dx -ne 0 -and $dy -ne 0 -and (!$allowed[$y*65+$nx] -or !$allowed[$ny*65+$x]))) { continue }
      if([Math]::Abs($s.heights[$v]-$s.heights[$u])/(.78125*[Math]::Sqrt($dx*$dx+$dy*$dy)) -gt $slopeLimit) { continue }
      if($mode -eq 'physical_clearance' -and (CircleCollision (-25+$x*.78125) (-25+$y*.78125) (-25+$nx*.78125) (-25+$ny*.78125))) { continue }
      $seen[$v]=$true; $q.Enqueue($v)
    } }
  }
  $reachable=@($seen | Where-Object {$_}).Count; $free=@($allowed | Where-Object {$_}).Count
  Write-Output "$mode goal_reachable=$($seen[$goal]) reachable_nodes=$reachable free_nodes=$free all_free_nodes_connected=$($reachable -eq $free)"
}
