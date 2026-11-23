$s=Get-Content "$PSScriptRoot/../webots/terrain/moose_original.json" -Raw | ConvertFrom-Json
$safe=[bool[]]::new(4225); $grade=[double[]]::new(4225)
for($y=2;$y -lt 63;$y++) { for($x=2;$x -lt 63;$x++) {
  $i=$y*65+$x; $gx=($s.heights[$i+1]-$s.heights[$i-1])/1.5625; $gy=($s.heights[$i+65]-$s.heights[$i-65])/1.5625
  $grade[$i]=[Math]::Sqrt($gx*$gx+$gy*$gy); $safe[$i]=$grade[$i] -le .3
} }
$seen=[bool[]]::new(4225); $steps=[int[]]::new(4225)
$q=[Collections.Generic.Queue[int]]::new(); $q.Enqueue(2623); $seen[2623]=$true
while($q.Count) {
  $u=$q.Dequeue(); $x=$u%65; $y=[int][Math]::Floor($u/65)
  foreach($dy in -1..1) { foreach($dx in -1..1) {
    $nx=$x+$dx; $ny=$y+$dy
    if(($dx -eq 0 -and $dy -eq 0) -or $nx -lt 2 -or $ny -lt 2 -or $nx -gt 62 -or $ny -gt 62) { continue }
    if($dx -ne 0 -and $dy -ne 0 -and (!$safe[$y*65+$nx] -or !$safe[$ny*65+$x])) { continue }
    $v=$ny*65+$nx
    if(!$seen[$v] -and $safe[$v] -and [Math]::Abs($s.heights[$v]-$s.heights[$u])/(.78125*[Math]::Sqrt($dx*$dx+$dy*$dy)) -le .3) {
      $seen[$v]=$true; $steps[$v]=$steps[$u]+1; $q.Enqueue($v)
    }
  } }
}
$candidates=for($i=0;$i -lt 4225;$i++) {
  if(!$seen[$i]) { continue }; $x=$i%65; $y=[int][Math]::Floor($i/65)
  $distance=.78125*[Math]::Sqrt(($x-23)*($x-23)+($y-40)*($y-40))
  if($distance -ge 7.5 -and $distance -le 10 -and $grade[$i] -lt .25) {
    [pscustomobject]@{col=$x;row=$y;x=-25+$x*.78125;y=-25+$y*.78125;distance=$distance;steps=$steps[$i];grade=$grade[$i]}
  }
}
$candidates | Sort-Object steps,distance | Select-Object -First 12 | Format-Table -AutoSize
