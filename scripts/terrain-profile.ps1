function Get-TerrainSlopeStats {
    param([double[]]$Heights,[int]$Width,[int]$Height,[double]$Spacing)
    $edgeMax=0.0; $triangleMax=0.0
    for($y=0;$y -lt $Height;$y++) { for($x=0;$x -lt $Width;$x++) {
        $i=$y*$Width+$x
        foreach($direction in @(@(1,0),@(0,1),@(1,1),@(-1,1))) {
            $nx=$x+$direction[0]; $ny=$y+$direction[1]
            if($nx -lt 0 -or $nx -ge $Width -or $ny -ge $Height) { continue }
            $grade=[Math]::Abs($Heights[$ny*$Width+$nx]-$Heights[$i])/($Spacing*[Math]::Sqrt($direction[0]*$direction[0]+$direction[1]*$direction[1]))
            $edgeMax=[Math]::Max($edgeMax,$grade)
        }
        if($x -lt $Width-1 -and $y -lt $Height-1) {
            $bottom=($Heights[$i+1]-$Heights[$i])/$Spacing
            $top=($Heights[$i+$Width+1]-$Heights[$i+$Width])/$Spacing
            $left=($Heights[$i+$Width]-$Heights[$i])/$Spacing
            $right=($Heights[$i+$Width+1]-$Heights[$i+1])/$Spacing
            foreach($gx in @($bottom,$top)) { foreach($gy in @($left,$right)) { $triangleMax=[Math]::Max($triangleMax,[Math]::Sqrt($gx*$gx+$gy*$gy)) } }
        }
    } }
    @{ maxEdgeGrade=$edgeMax; maxTriangleGrade=$triangleMax; maxGridAngle=[Math]::Atan($edgeMax)*180/[Math]::PI }
}

function ConvertToConnectedTerrain {
    param([double[]]$Heights,[int]$Width,[int]$Height,[double]$Spacing,[double]$MaxGrade=.25)
    if($Heights.Count -ne $Width*$Height -or $Spacing -le 0 -or $MaxGrade -le 0 -or $MaxGrade -gt .5) {
        throw 'Invalid terrain profile: require matching dimensions and 0 < MaxGrade <= 0.5'
    }
    # Bound each axis by MaxGrade/sqrt(2). Every triangle, central-difference
    # node gradient and eight-neighbour edge is then bounded by MaxGrade.
    $step=$Spacing*$MaxGrade/[Math]::Sqrt(2)
    function Get-LowerEnvelope([double[]]$Values) {
        $result=[double[]]$Values.Clone()
        for($y=0;$y -lt $Height;$y++) { for($x=0;$x -lt $Width;$x++) {
            $i=$y*$Width+$x
            if($x -gt 0) { $result[$i]=[Math]::Min($result[$i],$result[$i-1]+$step) }
            if($y -gt 0) { $result[$i]=[Math]::Min($result[$i],$result[$i-$Width]+$step) }
        } }
        for($y=$Height-1;$y -ge 0;$y--) { for($x=$Width-1;$x -ge 0;$x--) {
            $i=$y*$Width+$x
            if($x -lt $Width-1) { $result[$i]=[Math]::Min($result[$i],$result[$i+1]+$step) }
            if($y -lt $Height-1) { $result[$i]=[Math]::Min($result[$i],$result[$i+$Width]+$step) }
        } }
        return ,$result
    }
    $lower=Get-LowerEnvelope $Heights
    $negative=[double[]]::new($Heights.Count)
    for($i=0;$i -lt $Heights.Count;$i++) { $negative[$i]=-$Heights[$i] }
    $negativeUpper=Get-LowerEnvelope $negative
    $profile=[double[]]::new($Heights.Count); $sumOriginal=0.0; $sumProfile=0.0
    for($i=0;$i -lt $Heights.Count;$i++) {
        $profile[$i]=($lower[$i]-$negativeUpper[$i])/2
        $sumOriginal+=$Heights[$i]; $sumProfile+=$profile[$i]
    }
    # Preserve the original mean altitude while spreading steep cliffs into hills.
    $offset=($sumOriginal-$sumProfile)/$Heights.Count
    for($i=0;$i -lt $profile.Count;$i++) { $profile[$i]+=$offset }
    return ,$profile
}
