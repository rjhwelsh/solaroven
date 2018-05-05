function a(f)
    = 1/4/f;

// Returns the y coord at x
// for y = a*x**2
// where a = 1/(4*f);
function fx(f,x)
    = a(f)*pow(x,2);   
    
// returns the focus at thickness, t.    
function fxt(f,x,t)
    = f/(1+t/fx(f,max([x,1e-9])));
    
// Returns the positive x-value
// for sqrt(y/a) = x
function fy(f,y)
    = pow(y/a(f),0.5);
    
function root1(a,b,c)
    = (-b+pow((b*b-4*a*c),0.5))/2/a;
    
function root2(a,b,c)
    = (-b+pow((b*b-4*a*c),0.5))/2/a;

// Returns the derivative of x 
// dx/dy @ x
function dx(f,x)
    = 2*a(f)*pow(x,1);
  
// returns the slope of the angle.
function da(f,x)
    = atan(dx(f,x));
    
// returns the normal angle to the slope at x.   
function na(f,x)
    = da(f,x) +90;
    
// returns the magnitude of the vector (x,y).
function m(x,y)
    = pow(pow(x,2) + pow(y,2),0.5);
    
// returns the next x coord to get a vector length of m. (estimate)
function xn(f,x,m)
    = m*cos(da(f,x+0.5*m))+x;   
     
module ParabolicSolid(f, y_max, d) {
// ax**2 = y
// f = height of focus 
// y_max = y_max
// d = depth of extrusion    
a=a(f);    
p=1/(2*a);
ConeHeight = 0.5*pow(2,0.5)*(y_max+p);
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

module ParabolicTrough(f,y_max,d,t) {
    // t = the thickness between parabolas    
    difference() {
    ParabolicSolid(f,y_max,d);
    translate([0,-1,t])
        ParabolicSolid(f/(1+t/y_max),y_max,d+3);
    }
};

module ParabolicSection(f,y_max,d,t,x1,x2) {
    // Returns a section of parabola between x1 and x2 
    
    //Handle zero exceptions
    x1=x1+1e-9;
    x2=x2+1e-9;
    
    a=a(f);    
    p=1/(2*a);
    
    // maximum x coord    
    x0=pow(y_max/a,0.5);
    dx=x2-x1;
    
    //y1,y2 at end points
    y1=fx(f,x1);
    y2=fx(f,x2);
    y0=fx(f,x0); //i.e. y_max
    
    // Resultant absolute vector lengths
    m1=m(x1,y1);
    m2=m(x2,y2);
    m0=m(x0,y0);
    
    // and angles
    a1=da(f,x1);
    a2=da(f,x2);
    a0=da(f,x0);
         
    // overall angle
    theta=atan((y2-y1)/dx);
    // angle at end points
    th1 = atan(2*a*x1);
    th2 = atan(2*a*x2);
    
    // cube lengths for x2>x1>0;
    L1=m1*cos(90-th1+a1)+m0;
    L2=m0*cos(a0-a2)-m1;
    
    // ConeHeight (copied from above)
    ConeHeight = 0.5*pow(2,0.5)*(y_max+p);
    ConeRadius = ConeHeight;
    
    // Cube sizes 
    r1 = x0 + x1;
    r2 = x0 - x2;
        
   //rotate(a=[0,th1,0])
   // translate([-x1,0,-pow(x1,2)*a])
    difference() {
    ParabolicTrough(f,y_max,d,t);   
        
               
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
            cube([L2+2+r2,d+2,y_max+2]);
        
        
// Subtract unrotated range.
    translate([-x0,-1,-1])
    translate([-r1-t,0,0])
    cube([2*r1,d+2,L1+2]);
    translate([x2+t,-1,-1])
    cube([L2+2+r2,d+2,y_max+2]);
    };
    

};

// Produces a parabola section of estimated length, l
module ParabolicLength(f,x,t,d,l) {
    x2=xn(f,x,l);
    echo("x2=",x2);
    ParabolicSection(f,fx(f,x2+l),d,t,x,x2);  
}
