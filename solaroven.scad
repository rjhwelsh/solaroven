include<parabola.scad>
t=50;
l=360;
d=600;
f=150;

union() {
translate([0,0,(f+t)*sin(45)/2])
rotate([-45,0,0])
translate([0,0,d/4])
cube([f,f,d/2+t],center=true);
    
translate([0,d/4,f/2])
cube([f,d,f],center=true);
};

translate([0,0,(d+f/2)*sin(45)])
rotate([-45,0,0])
union() {
// reflector
ParabolicSection(f,300,d+f/2,t,-l,l);

// tube
translate([0,0,f+t])
rotate([-90,0,0])
union() {
    cylinder(r=75,h=d);
    translate([0,0,d])
        sphere(d=f);
};

// stand
translate([0,f/2,f/2+t])
cube([t,t,f],center=true);
translate([0,d-f/2,f/2+t])
cube([t,t,f],center=true);

};