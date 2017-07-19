module m4flushscrew(h) {
  // screw shaft, 4.1mm diameter for close fit
  translate([0,0,-h-1]) cylinder(h=h+2, d=4.1 + $overbore);
  // 3mm tall conical screw head, widens to 8mm
  translate([0,0,-3]) cylinder(h=3.01, r1=2.05 + $overbore/2, r2=4.05 + $overbore/2);
}

module m3flushscrew(h) {
  // screw shaft, 3.2mm diameter for close fit
  translate([0,0,-h-1]) cylinder(h=h+2, d=3.2 + $overbore);
  // 2mm tall conical screw head, widens to 6mm
  translate([0,0,-2]) cylinder(h=2.01, r1=1.6 + $overbore/2, r2=3.05 + $overbore/2);
}
