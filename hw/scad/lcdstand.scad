$fs = 0.05;
thickness = 2;

screw_diam = 2.5;
screw_drill = 2.5;
screw_clearance = 3.4;
screw_wall_thickness = 1.6;  // must be a multiple of nozzle size 0.4

dispW = 34.087;
dispL = 61.189;
disp_mount = [
  [0, 0],
  [dispW, 0],
  [dispW, dispL],
  [0, dispL]
];

disp_angle = 20;
disp_rise = 11;  // standoff height for header is 11mm
disp_fall = 3;  // amount to slide "down" at the angle
disp_offset = 2.54;  // amount to slide left

module mountpoint(x, y) {
  translate([x, y, -2]) cylinder(d=screw_diam-0.2, h=2);
}

module MountRef() {
  for (m = disp_mount) {
    mountpoint(m[0], m[1]);
  }
}

module flatcapsule(p0, p1, r) {
  linear_extrude(height=thickness) {
    hull() {
      translate(p0) circle(r=r);
      translate(p1) circle(r=r);
    }
  }
}

module OffsetMount(p0, p1, r) {
  difference() {
    flatcapsule(p0, p1, r);
    translate([p0[0], p0[1], -0.1]) cylinder(d=screw_clearance, h=thickness+0.2);
  }
}

module anglebeamorig(p0, p1, r) {
  difference() {
    hull() {
      translate(p0) sphere(r=r);
      translate([0, 0, disp_rise]) rotate([0, -disp_angle, 0]) translate([-disp_fall, disp_offset, 0]) translate(p1) sphere(r=r);
    }
    translate(p0) translate([0, 0, -r-0.1]) cylinder(h=r+0.1, r=r+0.1);
    translate([0, 0, disp_rise]) rotate([0, -disp_angle, 0]) translate([-disp_fall, disp_offset, 0]) translate(p1) cylinder(h=r+0.1, r=10);
  }
}

module anglebeam(p0, p1, r) {
  hull() {
    translate(p0) cylinder(h=4, r=r);
    translate([0, 0, disp_rise]) rotate([0, -disp_angle, 0]) translate([-disp_fall, disp_offset, -4]) translate(p1) cylinder(h=4, r=r);
  }
}

module Riser() {
  oy = 8;
  ox = 4;
  r = screw_wall_thickness + screw_drill/2;

  difference() {
    union() {
      flatcapsule([0, 0], [ox, -oy], r);
      flatcapsule([ox, -oy], [dispW-ox, -oy], r);
      flatcapsule([dispW-ox, -oy], [dispW, 0], r);

      flatcapsule([0, dispL], [ox, dispL+oy], r);
      flatcapsule([ox, dispL+oy], [dispW-ox, dispL+oy], r);
      flatcapsule([dispW-ox, dispL+oy], [dispW, dispL], r);
      for (m = disp_mount) {
        anglebeam(m, m, r);
      }
    }
    for (m = disp_mount) {
      translate([m[0], m[1], -0.1]) cylinder(h=5.1, d=screw_drill);
      translate([0, 0, disp_rise]) rotate([0, -disp_angle, 0])
        translate([-disp_fall, disp_offset, -5]) translate(m)
        cylinder(h=5.1, d=screw_drill);
    }
  }
}

%MountRef();
translate([17.043, -0.864, 11.0/2]) %cube([20.32+2.54, 2.54, 11], center=true);
translate([0, 0, disp_rise]) rotate([0, -disp_angle, 0]) translate([-disp_fall, disp_offset, 0]) %MountRef();

Riser();
