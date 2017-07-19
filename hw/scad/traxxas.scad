include <fasteners.scad>

// all measurements in mm
$overbore = 0.25;
$fs = 0.2;
$fa = 12;

ServoPlateLength = 124;
ServoPlateWidthL = 45;
ServoPlateWidthR = 74;

Servoplate_screws = [
  [ServoPlateLength/2 - 5, -ServoPlateWidthL/2 + 5, 4],
  [ServoPlateLength/2 - 5,  ServoPlateWidthL/2 - 5, 4],
  [-ServoPlateLength/2 + 3.3 + 3/2, -ServoPlateWidthR/2 + 3.3 + 3/2, 3],
  [-ServoPlateLength/2 + 3.3 + 3/2,  ServoPlateWidthR/2 - 3.3 - 3/2, 3]
];

Servoplate_cutouts = [
  [-ServoPlateLength/2 - 2, 48.0, 3.84 + 2],
  [-ServoPlateLength/2 - 2, 15.5, 7.65 + 2]];

Servoplate_doohicky = [
  [ServoPlateLength/2 - 9.7, 4.3, 12.2, 6.88],
  [ServoPlateLength/2 - 8.4, 7.1, 3, 11.96]];

    // translate([l/2 + r - 9.7 - 4.3, -12.2/2, -0.1]) cube([4.3, 12.2, 6.88]);
    // translate([l/2 + r - 7.1 - 8.4, -3/2, -0.1]) cube([7.1, 3, 11.96]);

module servoholder() {
  r = 4.3;
  // TODO: verify these
  w1 = ServoPlateWidthR - 2*r;
  w2 = ServoPlateWidthL - 2*r;
  l = ServoPlateLength - 2*r;
  l1 = 44.5;
  l2 = 25.4;
  color("gray") {
    translate([0,0,-4.3]) difference() {
      // outer polygon
      linear_extrude(4.3) minkowski() {
        polygon(points = [
            [-l/2, w1/2], [-l/2 + l1, w1/2], [l/2 - l2, w2/2], [l/2, w2/2],
            [l/2, -w2/2], [l/2 - l2, -w2/2], [-l/2 + l1, -w1/2], [-l/2, -w1/2]]);
        circle(r=r);
      }

      // cutouts where battery goes in
      for (c = Servoplate_cutouts) {
        translate([c[0], -c[1]/2, -0.01]) cube([c[2], c[1], 4.4]);
      }

      // screw holes
      for (h = Servoplate_screws) {
        translate([h[0], h[1], 4.3]) {
          if (h[2] == 3) m3flushscrew(h=20);
          else m4flushscrew(h=20);
        }
      }
    }

    // servo horn clearance bumps
    translate([-l/2 - r + 15.2, w1/2 + r - 19, -0.1]) cube([21.6, 19, 4.4]);
    translate([-l/2 - r + 15.2, -w1/2 - r, -0.1]) cube([21.6, 19, 4.4]);

    // weird doohicky on the front
    for (d = Servoplate_doohicky) {
      translate([d[0] - d[1], -d[2]/2, -0.1])
        cube([d[1], d[2], d[3]]);
    }
  }
}
