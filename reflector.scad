include<parabola.scad>

// global vars
f=150; // parabola focus
t=80;  // section thickness
d=50;  // depth of profile

t1=10;  // wall thickness / pin diameter
d1=15; // pin depth
h1=20;  // pin height off surface.
e1=20;  // pin offset offend
f1=15;  // offset to diagonal track
g1=30;  // length of diagonal track
a1=45;
j1=20;  // length of track parallel to end
p1=t1*1.1; // pin latch radius


module panelBody(x,t,d,l) {
    if(2*d1>d){
           echo("<font color='red'>Error: d1 must be less than half of d!</font>");
           assert(false);
    };
    
    
    // Line up beginning of track
    dm=(t-h1-t1);
    th=na(f,x);
    ta=da(f,x);
    dy=dm*sin(th);
    dx=dm*cos(th);
    y=fx(f,x);
    
    // Line up end of track
    x2=xn(f,x,l);
    y2=fx(f,x2);
    th2=na(f,x2);
    ta2=da(f,x);
    dy2=dm*sin(th2);
    dx2=dm*cos(th2);
    m2=m(x2-x,y2-y);
    
    // Calculations for diagonal
    dm=(t-h1-t1);
    x3=xn(f,x2,-f1-t1);
    y3=fx(f,x3);
    th3=na(f,x3);
    dy3=dm*sin(th3);
    dx3=dm*cos(th3);
    m3=g1*sin(a1)-t1*2/3; //distance across parabola
    l3=g1*cos(a1)-t1/3; //distance along parabola
    
    // Calculations for straight track (parallel)
    dm4=(t-h1-t1-m3);
    x4=xn(f,x2,-f1-t1-l3);
    y4=fx(f,x4);
    th4=na(f,x4);
    dy4=dm4*sin(th4);
    dx4=dm4*cos(th4);
    
    // Calculations for straight track (perpendicular)
    dm5=(t-h1-t1-m3-j1);
    x5=xn(f,x2,-f1-t1-l3-t1/2);
    y5=fx(f,x5);
    th5=na(f,x5);
    dy5=dm5*sin(th5);
    dx5=dm5*cos(th5);
     
    difference(){
        ParabolicLength(f,x,t,d,l);
        // pin travel
        translate([dx,-1,dy])
            ParabolicLength(f,x-1,t1,d1+1,m2-e1);
        
        // Straight tracks (diagonal)     
        translate([x3+dx3,-1,y3+dy3])
        rotate([0,-th2-a1,0]) 
        cube([t1,d1+1,g1]);
        
        translate([x4+dx4,-1,y4+dy4])
        rotate([0,-th2-90,0])
            union(){
                cube([t1,d1+1,j1]);
                translate([t1/2,0,j1])
                rotate([-90,0,0])
                cylinder(r=p1/2,h=d1+1);
            }
    }
    


}

panelBody(0,t,d,200);



//ParabolicLength(150,189.7,50,60,200);
//ParabolicLength(150,333.57,50,70,200);

