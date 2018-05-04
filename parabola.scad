// Returns the y coord at x
// for y = a*x**2
// where a = 1/(4*f);
function fx(f,x)
    = 1/4/f*pow(x,2);

// Returns the derivative of x 
// dx/dy @ x
function dx(f,x)
    = 2/4/f*pow(x,1);
  
// returns the slope of the angle.
function da(f,x)
    = atan(dx(f,x)/x);
    
module ParabolicSolid(f, w, d) {
// ax**2 = y
// f = height of focus 
// w = y_max
// d = depth of extrusion    
a=1/4/f;    
p=1/(2*a);
ConeHeight = 0.5*pow(2,0.5)*(w+p);
ConeRadius = ConeHeight;
translate([0,d,0])
rotate([90,0,0])    
linear_extrude(height=d) 
projection(cut=true)
    translate([0,0,p])  
    rotate([atan(ConeHeight/ConeRadius),0,0])
    translate([0,0,-ConeHeight])  
    {
    cylinder(h=ConeHeight, r1=ConeRadius, r2=0);
    }
};

module ParabolicTrough(f,w,d,t) {
    // t = the thickness between parabolas    
    difference() {
    ParabolicSolid(f,w,d);
    translate([0,-1,t])
        ParabolicSolid(f/(1+t/w),w,d+3);
    }
};

module ParabolicSection(f,w,d,t,x1,x2) {
    // Returns a section of parabola between x1 and x2 
    
    //Handle zero exceptions
    x1=x1+1e-9;
    x2=x2+1e-9;
    
    a=1/4/f;    
    p=1/(2*a);
    
    // maximum x coord    
    x0=pow(w/a,0.5);
    dx=x2-x1;
    
    //y1,y2 at end points
    y1=pow(x1,2)*a;
    y2=pow(x2,2)*a;
    y0=pow(x0,2)*a; //i.e. w
    
    // Resultant absolute vector lengths
    m1=pow(pow(y1,2)+pow(x1,2),0.5);
    m2=pow(pow(y2,2)+pow(x2,2),0.5);
    m0=pow(pow(y0,2)+pow(x0,2),0.5);
    // and angles
    a1=atan(y1/x1);
    a2=atan(y2/x2);
    a0=atan(y0/x0);
         
    // overall angle
    theta=atan((y2-y1)/dx);
    // angle at end points
    th1 = atan(2*a*x1);
    th2 = atan(2*a*x2);
    
    // cube lengths for x2>x1>0;
    L1=m1*cos(90-th1+a1)+m0;
    L2=m0*cos(a0-a2)-m1;
    
    // ConeHeight (copied from above)
    ConeHeight = 0.5*pow(2,0.5)*(w+p);
    ConeRadius = ConeHeight;
    
    // Cube sizes 
    r1 = x0 + x1;
    r2 = x0 - x2;
        
   rotate(a=[0,th1,0])
    translate([-x1,0,-pow(x1,2)*a])
    difference() {
    ParabolicTrough(f,w,d,t);
//Subtract first portion
    translate([-x0,-1,-1])
        translate([r1-r1*cos(th1),0,y1-r1*sin(th1)])
        rotate([0,-th1,0])
            translate([-r1,0,0])
            cube([2*r1,d+2,L1+2]);
//Subtract second portion        
    translate([x2,-1,-1]) 
     translate([0,0,y2])   
        rotate([0,-th2,0])
            cube([L2+2+r2,d+2,w+2]);
    };
};

