$fs=0.1;

RPi3MountingScrews = [
  [-58/2, -49/2, 0], [58/2, -49/2, 0],
  [-58/2, 49/2, 0], [58/2, 49/2, 0]
];

arducam_width = 37;
arducam_height = 37;
arducam_mount = [
  [-28.5/2, 28.5/2],
  [-28.5/2, -28.5/2],
  [28.5/2, -28.5/2],
  [28.5/2, 28.5/2],
];

rpi3_pos = [10, 26, 25];

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

module rpimount() {
  rearplate_holes = [
   [-19.7/2, -21.9],
   [19.7/2, -21.9],
   [-84/2, 0],
   [84/2, 0]
  ];

  beam_mounts = [
    [0, -21.9, 0],
    [-30, -8, 0],
    [30, -8, 0]
  ];

  difference() {
    thick = 3;
    union() {
      for (h = [[2,0, 1], [0,1, 0], [1,3, 2]]) {
        hull() {
          translate(rearplate_holes[h[0]]) cylinder(h=thick, d=8);
          translate(rearplate_holes[h[1]]) cylinder(h=thick, d=8);
          translate(beam_mounts[h[2]]) cylinder(h=thick, d=8);
        }
      };

      for (h = [[0, 2], [0, 3], [1, 0], [1, 2], [2, 1], [2, 3]]) {
        beam(beam_mounts[h[0]], rpi3_pos + RPi3MountingScrews[h[1]], 4);
      }
      for (h = RPi3MountingScrews) {
        translate(rpi3_pos) translate(h) cylinder(h=2, d=5.5);
      }
    }

    for (h = rearplate_holes) {
      d2 = 3 + thick*(5.6 - 3)/1.8;
      translate([h[0], h[1], -0.1]) cylinder(h=10, d=3.4);
      translate([h[0], h[1], -0.1]) cylinder(h=thick+.2, d1=3.2, d2=d2);
    };
    for (h = RPi3MountingScrews) {
      translate(rpi3_pos) translate([h[0], h[1], -11]) cylinder(h=20, d=2.5);
    }
  }
}

// rpimount();

module bumperbuck() {
  w = 43;
  w1 = w/2 - 1;
  w2 = w/2;
  hull() {
    translate([-w1, 2, 0]) cylinder(h=2.8, r=1);
    translate([w1, 2, 0]) cylinder(h=2.8, r=1);
    translate([w2, -6, 0]) cylinder(h=2.8, r=0.4);
    translate([-w2, -6, 0]) cylinder(h=2.8, r=0.4);
  }
}

module cameraboard() {
  translate([0, 20, 30]) rotate([90, 0, 0]) {
    difference() {
      translate([-arducam_width/2, -arducam_height/2, 0]) cube([arducam_width, arducam_height, 1]);
      for (h = arducam_mount) {
        translate([h[0], h[1], -0.1]) cylinder(h=10, d=2.5);
      }
    }
  }
}

module bumper() {
  diam = 5;
  r = diam/2;
  pts = [
    [25, r, r],
    [80, 40, r],
    [50, 50, r],
    [0, 7, r],
    [0, 55, r],
  ];
  vts = [[0, 1], [0, 2], [1, 2], [3, 2], [2, 4], [3, 4]];
  for (v = vts) {
    hull() {
      translate(pts[v[0]]) sphere(r=r);
      translate(pts[v[1]]) sphere(r=r);
    }
    hull() {
      p1 = pts[v[0]];
      translate([-p1[0], p1[1], p1[2]]) sphere(r=r);
      p2 = pts[v[1]];
      translate([-p2[0], p2[1], p2[2]]) sphere(r=r);
    }
  }
}

module camerathing() {
  r = 2;
  difference() {
    union() {
      for (h = arducam_mount) {
        hull() {
          translate([-25, 3, r]) sphere(r=r);
          translate([0, 20-2, 30]) rotate([90, 0, 0]) translate(h) {
            sphere(r=r);
          }
        }
        hull() {
          translate([25, 3, r]) sphere(r=r);
          translate([0, 20-2, 30]) rotate([90, 0, 0]) translate(h) {
            sphere(r=r);
          }
        }
      }
      translate([0, 20-1, 30]) rotate([90, 0, 0]) {
        for (h = arducam_mount) {
          translate([h[0], h[1], 0]) cylinder(h=5, d=5);
        }
      }
    }
    translate([0, 20-1, 30]) rotate([90, 0, 0]) {
      translate([-100, -100, -10]) cube([200, 200, 10]);
      for (h = arducam_mount) {
        translate([h[0], h[1], -0.1]) cylinder(h=6, d=2.5);
      }
    }
  }
}

module bumpermount() {
  difference() {
    union() {
      difference() {
        union() {
          // translate([-30, -2.9, 0]) cube([60, 23, 4]);
          r=2;
          hull() {
            translate([-30+r, -2.9+r, r]) sphere(r=r);
            translate([30-r, -2.9+r, r]) sphere(r=r);
            translate([30-r, -2.9+15, r]) sphere(r=r);
            translate([-30+r, -2.9+15, r]) sphere(r=r);
          }
          camerathing();
          bumper();
        }
        bumperbuck();
      }
      // screw bosses
      translate([-15, 0, 3]) cylinder(h=7, d=6);
      translate([15, 0, 3]) cylinder(h=7, d=6);
    }
    translate([-15, 0, -0.1]) cylinder(h=100, d=3);
    translate([15, 0, -0.1]) cylinder(h=100, d=3);
  }
}

%cameraboard();
%bumpermount();

translate([0, -100, 0]) rpimount();
