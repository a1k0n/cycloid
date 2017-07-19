include <fasteners.scad>
include <traxxas.scad>

$overbore = 0.25;

// All screws are #4-40, 2.2606 internal diameter
BasePlateMountingScrews = [
  [-55, 20], [-55, -20],
  [-30, 10], [-30, -10]
];

RPi3MountingScrews = [
  [3.5, 3.5], [3.5 + 58, 3.5],
  [3.5, 3.5 + 49], [3.5 + 58, 3.5 + 49]
];

PWMBoardScrews = [
  [3.3, 3.3], [59.2, 3.3],
  [3.3, 22.4], [59.2, 22.4]
];

IMUBoardScrews = [
  [2.5, 2.5], [2.5, 23]
];

PowerBoostScrews = [
  [2.6, 25], [2.6 + 17.65, 25], [2.6 + 17.65 / 2, 3]
];

module IMUMount() {
  // dimensions of 9DOF PCB
  // FIXME for MPU9250
  width = 22.1;
  length = 16.85;
  depth = 1.66;  // PCB is 1.66mm thick or so
  height = 3;  // stand off 3mm
}

module BasePlate() {
  thick = 4;
  difference() {
    union() {
      linear_extrude(thick) minkowski() {
        polygon(points = [
            [Servoplate_screws[0][0], Servoplate_screws[0][1]],
            [40, -10],
            [-55, -10],
            [Servoplate_screws[2][0], Servoplate_screws[2][1]],
            [Servoplate_screws[3][0], Servoplate_screws[3][1]],
            [-55, 10],
            [40, 10],
            [Servoplate_screws[1][0], Servoplate_screws[1][1]]]);
        circle(r=5);
      }
      for (h = BasePlateMountingScrews) {
        translate([h[0], h[1], thick-0.1])
          cylinder(d=6, h=6);
      }
    }

    for (c = Servoplate_cutouts) {
      translate([c[0], -c[1]/2, -0.01]) cube([c[2], c[1], 4.4]);
    }

    for (d = Servoplate_doohicky) {
      margin = 1;
      translate([d[0] - d[1] - margin, -d[2]/2 - margin, -0.1])
        cube([d[1] + 2*margin, d[2] + 2*margin, d[3]]);
    }

    for (s = Servoplate_screws) {
      translate([s[0], s[1], thick + 0.001]) {
        if (s[2] == 3) m3flushscrew(h=20);
        else m4flushscrew(h=20);
      }
    }

    for (h = BasePlateMountingScrews) {
      translate([h[0], h[1], -0.1])
        cylinder(d=2.2606 + $overbore, h=thick+20);
    }
  }
}

module ElectronicsPlate() {
  thick = 3;
  totalsize = [120, 90, thick];
  mounting_offset = [130, totalsize[1] / 2, 0];
  difference() {
    union() {
      translate(-mounting_offset) {
        cube(totalsize);
        translate([120, 90, 0]) rotate([0, 0, 180]) {
          for (h = RPi3MountingScrews) {
            translate([h[0], h[1], thick-0.1]) cylinder(d=5, h=5);
          }
          translate([0, 61, 0])
            for (h = PWMBoardScrews) {
              translate([h[0], h[1], thick-0.1]) cylinder(d=5, h=5);
            }
          translate([68, 61, 0])
            for (h = IMUBoardScrews) {
              translate([h[0], h[1], thick-0.1]) cylinder(d=5, h=5);
            }
          translate([88, 61, 0])
            for (h = PowerBoostScrews) {
              translate([h[0], h[1], thick-0.1]) cylinder(d=5, h=5);
            }
          translate([90, 2, 1.9]) {
            translate([0, 38, 0]) rotate([90, 0, 90]) {
              linear_extrude(height=30) polygon(points=[[0,0], [-1,0], [-1,8], [-3,10], [-2,10], [0,8]]);
            }
            rotate([90, 0, 90]) {
              linear_extrude(height=30) polygon(points=[[0,0], [1,0], [1,8], [3,10], [2,10], [0,8]]);
            }
          }
        }
      }
    }
    for (h = BasePlateMountingScrews) {
      translate([h[0], h[1], -0.1])
        cylinder(d=2.2606 + $overbore, h=thick+0.2);
    }
    translate(-mounting_offset) {
      translate([120, 90, 0]) rotate([0, 0, 180]) {
        for (h = RPi3MountingScrews) {
          // #4-40 screws
          translate([h[0], h[1], -0.1]) cylinder(d=2.2606 + $overbore, h=thick + 6);
        }
        translate([0, 61, 0])
          for (h = PWMBoardScrews) {
            // PCA9685 uses M2 screws
            // translate([h[0], h[1], -0.1]) cylinder(d=1.6 + $overbore, h=thick + 6);
            // but my replacement board is M2.5 / #4-40 screws
            translate([h[0], h[1], -0.1]) cylinder(d=2.2606 + $overbore, h=thick + 6);
          }
        translate([68, 61, 0])
          for (h = IMUBoardScrews) {
            // #4-40
            translate([h[0], h[1], -0.1]) cylinder(d=2.2606 + $overbore, h=thick + 6);
          }
        translate([88, 61, 0])
          for (h = PowerBoostScrews) {
            // M2
            translate([h[0], h[1], -0.1]) cylinder(d=1.6 + $overbore, h=thick + 6);
          }
      }
    }
  }
};

translate([0,0,-0.01]) %servoholder();
%BasePlate();
translate([0, 0, 4+6]) ElectronicsPlate();
