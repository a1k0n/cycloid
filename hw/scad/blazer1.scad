thickness = 3;
cam_border = 3;
arducam_width = 37;
arducam_height = 37;
arducam_mount = [
  [-28.5/2, 28.5/2],
  [-28.5/2, -28.5/2],
  [28.5/2, -28.5/2],
  [28.5/2, 28.5/2],
];
arducam_lens_diam = 18;
screw_diam = 2.26;  // #4-40
screw_wall_thickness = 1.6;  // must be a multiple of nozzle size 0.4
support_offset = 3.5;
body_post_hole_dia = 6.5;

module FrontPostMount() {
  totalhigh = arducam_height + cam_border*2;
  difference() {
    union() {
      translate([-82/2, -10/2, 0])
        cube([82, 13, thickness]);
      translate([-82/2, 5, 0])
        cube([82, thickness, totalhigh]);
      translate([-82/2, -10/2, totalhigh])
        cube([82, 13, thickness]);
      for (h = arducam_mount) {
        translate([h[0], 5, totalhigh/2 + h[1]]) {
          rotate([90,0,0])
            cylinder(d=screw_diam + 2*screw_wall_thickness, h=support_offset + 0.1, $fn=20);
        }
      }
    }

    // front post holes
    translate([-70/2, 0, -0.1])
      cylinder(d=body_post_hole_dia, h=100, $fn=30);
    translate([70/2, 0, -0.1])
      cylinder(d=body_post_hole_dia, h=100, $fn=30);

    translate([-arducam_width/2-2, -5.1, -0.1])
      cube([arducam_width+4, 7, thickness+1]);
    translate([-23/2, 2.5, totalhigh/2 - 5/2])
      cube([23, 4.2, 5]);
    for (h = arducam_mount) {
      translate([h[0], -5, totalhigh/2 + h[1]]) {
        rotate([-90,0,0])
          cylinder(d=screw_diam, h=thickness + 11, $fn=10);
      }
    }
    // hole for lens
    translate([0, 0, totalhigh/2])
      rotate([-90,0,0])
      cylinder(d=arducam_lens_diam, h=100);
  }
}

module ShockTowerAdapter() {
  t1 = 5;  // vertical thickness
  t2 = 2.5; // thickness around CF shock tower
  t3 = 2.8;  // thickness of gap for CF shock tower
  l1 = 18.6;  // length along CF shock tower
  difference() {
    union() {
      translate([-t3/2-t2, -l1/2, 0])
        cube([t3+t2*2, l1, 9.4+t1]);
    }
    // remove gap for shock tower
    translate([-t3/2, -l1/2-0.1, -0.1])
      cube([t3, l1+0.2, 9.5]);

    // holes for screws on top
    for (side = [-1, 1]) {
      translate([0, 12*side/2, 0])
        cylinder(d=screw_diam, h=9.4+t1+0.1, $fn=30);
    }

    // screw holes going through
    for (side = [-1, 1]) {
    translate([-50, side*11.4/2, 9.4-5.7])
      rotate([0, 90, 0])
      cylinder(d=screw_diam, h=100, $fn=30);
    translate([0, side*11.4/2, 9.4-5.7])
      rotate([0, 90, 0])
      cylinder(d=screw_diam*1.4, h=100, $fn=30);
    }
  }
}

module FrontSprocketMount() {
  height = 27;
  spacing = 24;
  thick = 2.5;
  width = spacing + 3*thick;
  x1 = 2*thick + screw_diam;
  difference() {
    union() {
      translate([-width/2, 0, 0])
        cube([width, height, thick]);
      for (side = [-width/2, width/2-x1]) {
        translate([side, 0, thick-0.1])
          cube([x1, thick, x1+0.1]);
      }
      translate([-width/2, height-2*thick, 0])
        cube([width, 2*thick, x1+thick]);
    }

    translate([-width/2+x1, -0.1, -0.1])
      cube([width-2*x1, height-2*thick+0.1, thick+0.2]);

    for (side = [-1, 1]) {
      translate([side*spacing/2, -0.1, thick+x1/2])
        rotate([-90, 0, 0])
        cylinder(d=screw_diam, h=100, $fn=30);
      translate([side*spacing/2, -0.1, thick+x1/2])
        rotate([-90, 0, 0])
        cylinder(d=3.8, h=thick*3, $fn=30);
    }

  }
}

BaseScrews =[
  [-36.5/2, 12], [-36.5/2, -12],
  [36.5/2, 6], [36.5/2, -6]
];

RPi3MountingScrews = [
  [3.5, 3.5], [3.5 + 58, 3.5],
  [3.5, 3.5 + 49], [3.5 + 58, 3.5 + 49]
];

module BasePlateWithTeensy() {
  thick = 2.5;
  standoff = 4;

  basesize = [80, 85, thick];
  mountoffset = [-10, 0, 0];  // distance from center to adjust screws

  PWMBoardScrews = [
    [3.3, 3.3], [59.2, 3.3],
    [3.3, 22.4], [59.2, 22.4]
  ];

  difference() {
    union() {
      translate(mountoffset - [basesize[0]/2, basesize[1]/2, 0]) {
        cube(basesize);
        for (s = RPi3MountingScrews) {
          translate([s[0], s[1], thick-0.1])
            cylinder(d=5, h=standoff, $fn=30);
        }
      }

      translate(mountoffset - [basesize[0]/2, -basesize[1]/2, 0]) {
        for (s = PWMBoardScrews) {
          translate([s[0], -s[1], thick-0.1])
            cylinder(d=5, h=standoff, $fn=30);
        }
      }
    }

    for (s = BaseScrews) {
      translate([s[0], s[1], -0.1])
        cylinder(d=screw_diam*1.4, h=thick+0.2, $fn=30);
    }

    translate(mountoffset - [basesize[0]/2, basesize[1]/2, 0])
      for (s = RPi3MountingScrews) {
        translate([s[0], s[1], -0.1])
          cylinder(d=screw_diam, h=thick+standoff+0.2, $fn=30);
      }

    translate(mountoffset - [basesize[0]/2, -basesize[1]/2, 0]) {
      for (s = PWMBoardScrews) {
        translate([s[0], -s[1], -0.1])
          cylinder(d=screw_diam, h=thick+standoff+0.2, $fn=30);
      }
    }
  }
}

module BasePlate() {  // raspi only, with HAT board
  thick = 2.5;
  standoff = 4;

  basesize = [65, 56, thick];
  mountoffset = [-10, 0, 0];  // distance from center to adjust screws

  difference() {
    union() {
      translate(mountoffset - [basesize[0]/2, basesize[1]/2, 0]) {
        cube(basesize);
        for (s = RPi3MountingScrews) {
          translate([s[0], s[1], thick-0.1])
            cylinder(d=5, h=standoff, $fn=30);
        }
      }
    }

    for (s = BaseScrews) {
      translate([s[0], s[1], -0.1])
        cylinder(d=screw_diam*1.4, h=thick+0.2, $fn=30);
    }

    translate(mountoffset - [basesize[0]/2, basesize[1]/2, 0])
      for (s = RPi3MountingScrews) {
        translate([s[0], s[1], -0.1])
          cylinder(d=screw_diam, h=thick+standoff+0.2, $fn=30);
      }
  }
}

// rotate([-90, 0, 0]) FrontPostMount();
//rotate([0, 180, 0]) ShockTowerAdapter();
//FrontSprocketMount();
BasePlate();
