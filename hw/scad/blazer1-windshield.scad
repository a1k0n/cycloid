$fs = 0.05;
thickness = 2;
cam_border = 3;

rpi_mount_height = 23.5;
rpi_mount_x = -3;

cam_mount_height = 45;
cam_mount_x = 185;
cam_mount_rotate = [0, 22, 0];

r2 = 1.0/sqrt(2);
function rot45(x) = [x[0]*r2 + x[1]*r2, -x[0]*r2 + x[1]*r2];

arducam_width = 37;
arducam_height = 37;
arducam_mount = [
  [-28.5/2, 28.5/2],
  [-28.5/2, -28.5/2],
  [28.5/2, -28.5/2],
  [28.5/2, 28.5/2],
];
arducam_lens_diam = 18;

RPi3MountingScrews = [
  [3.5, -49.0/2], [3.5 + 58, -49.0/2],
  [3.5, 49.0/2], [3.5 + 58, 49.0/2]
];

// #4-40 sizes:
//screw_diam = ???
//screw_drill = .0890 * 25.4;  // (2.2606)
//screw_clearance = ???

screw_diam = 2.5;
//screw_drill = 2.05;
// technically it should be drilled to 2.05mm and tapped, but there's exactly
// the right amount of over-extrusion to catch screw threads if we just make a
// 2.5mm hole, with no drilling/tapping.
screw_drill = 2.5;
screw_clearance = 3.4;
screw_wall_thickness = 1.6;  // must be a multiple of nozzle size 0.4

module mountpoint(x, y) {
  translate([x, y, -2]) cylinder(d=screw_diam-0.2, h=2);
}

module CarMountingReference() {
  // Put little cylinders where all the mounting holes are in the base plate
  fork_x = [0, 8.8, 21.5, 39];
  for (x = fork_x) {
    mountpoint(x, -12);
    mountpoint(x, 12);
  }
  for (x = [75.5, 111.5, 127.5, 138.5]) {
    mountpoint(x, 0);
  }
}

module RaspiMountReference() {
  for (s = RPi3MountingScrews) {
    mountpoint(s[0], s[1]);
  }
}

module CamMountReference() {
  rotate(cam_mount_rotate) {
    for (s = arducam_mount) {
      mountpoint(s[0], s[1]);
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

module beam_flat(p0, p1, r) {
  dx = p1[0] - p0[0];
  dy = p1[1] - p0[1];
  dz = p1[2] - p0[2];
  M = [
    [1, 0, dx/dz, 0],
    [0, 1, dy/dz, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
  ];
  translate(p0) multmatrix(M) cylinder(h=dz, r=r);
}

module beam(p0, p1, r) {
  dx = p1[0] - p0[0];
  dy = p1[1] - p0[1];
  dz = p1[2] - p0[2];
  intersection() {
    hull() {
      translate(p0) sphere(r=r);
      translate(p1) sphere(r=r);
    }
    translate([
        min(p0[0], p1[0]) - r,
        min(p0[1], p1[1]) - r,
        min(p0[2], p1[2])])
      cube([abs(dx)+2*r, abs(dy)+2*r, abs(dz)]);
  }
}

module OffsetMount(p0, p1, r) {
  difference() {
    linear_extrude(height=thickness) {
      hull() {
        translate(p0) circle(r=r);
        translate(p1) circle(r=r);
      }
    }
    translate([p0[0], p0[1], -0.1]) cylinder(d=screw_clearance, h=thickness+0.2);
  }
}

module RpiBoardThing() {
  r = screw_wall_thickness + screw_drill/2;
  o=5;
  stand = 4;

  beam_size = r;

  OffsetMount([0, -12], [0+o, -12-o], r);
  OffsetMount([39, -12], [39-o, -12-o], r);
  OffsetMount([0, 12], [0+o, 12+o], r);
  OffsetMount([39, 12], [39-o, 12+o], r);

  difference() {
    h = rpi_mount_height - stand;
    union() {
      for (i = [0, 1]) {
        beam([0+o, -12-o, 0], [rpi_mount_x + RPi3MountingScrews[i][0], RPi3MountingScrews[i][1], h], beam_size);
        beam([39-o, -12-o, 0], [rpi_mount_x + RPi3MountingScrews[i][0], RPi3MountingScrews[i][1], h], beam_size);
      }
      for (i = [2, 3]) {
        beam([0+o, 12+o, 0], [rpi_mount_x + RPi3MountingScrews[i][0], RPi3MountingScrews[i][1], h], beam_size);
        beam([39-o, 12+o, 0], [rpi_mount_x + RPi3MountingScrews[i][0], RPi3MountingScrews[i][1], h], beam_size);
      }
      for (s = RPi3MountingScrews) {
        translate([rpi_mount_x + s[0], s[1], rpi_mount_height - stand - 0.1])
          cylinder(r=r, h=stand+0.1);
      }
    }
    for (s = RPi3MountingScrews) {
      translate([rpi_mount_x + s[0], s[1], rpi_mount_height - 20])
        cylinder(d=screw_drill, h=20.1);
    }
  }
}

module anglebeam(p0, p1ref, p1off, rot, r) {
  hull() {
    // top half of bottom sphere
    translate(p0) intersection() { sphere(r=r); translate([-r, -r, 0]) cube([2*r, 2*r, r]); }
    // bottom half of top sphere
    translate(p1ref) rotate(rot) translate(p1off) intersection() { sphere(r=r); translate([-r, -r, -r]) cube([2*r, 2*r, r]); }
  }
}

module CamBoardThing() {
  r = screw_wall_thickness + screw_drill/2;

  mountpts = [
    [168, 0, 0],
    [168+33, 12, 0], [168+33, -12, 0],
    [168+33+9, 12, 0], [168+33+9, -12, 0]
  ];
  offpts = [[165, 7, 0], [165, -9, 0], [205.5, 19, 0], [205.5, -19, 0]];

  beam_size = r;

  OffsetMount(mountpts[0], offpts[0], r);
  OffsetMount(mountpts[0], offpts[1], r);
  OffsetMount(mountpts[1], offpts[2], r);
  OffsetMount(mountpts[3], offpts[2], r);
  OffsetMount(mountpts[2], offpts[3], r);
  OffsetMount(mountpts[4], offpts[3], r);

  // bottom 2 top attachment configuration
  b2t = [
    [0, 0], [0, 3],
    [2, 0], [2, 3],
    [1, 1], [1, 2],
    [3, 1], [3, 2],
  ];
  difference() {
    union() {
      for (bt = b2t) {
        anglebeam(offpts[bt[0]], [cam_mount_x, 0, cam_mount_height], arducam_mount[bt[1]], cam_mount_rotate, r);
      }
    }
    // cut screw holes
    for (p1 = arducam_mount) {
      translate([cam_mount_x, 0, cam_mount_height])
        rotate(cam_mount_rotate)
        translate([p1[0], p1[1], -14.1])
        cylinder(h=14.2, d=screw_drill);
    }
    // make extra room for the belt
    translate([0, -6, 1.0]) rotate([0, 88, 0]) cylinder(r=3, h=200);
  }
}

%CarMountingReference();
%translate([rpi_mount_x, 0, rpi_mount_height]) RaspiMountReference();
%translate([cam_mount_x, 0, cam_mount_height]) CamMountReference();

*RpiBoardThing();
CamBoardThing();
