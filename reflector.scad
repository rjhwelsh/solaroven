include<parabola.scad>
include<fillet.scad>

$fn=100;
// global vars
//f=150; // parabola focus
t=90;  // section thickness
d=90;  // depth of profile
ymax=600;

f=focus_adjust(150,t,ymax); //adjust focus 
fil1=5; // fillet radius 

t1=10+2*fil1;  // wall thickness / pin diameter
t2=0.9;         // sheet metal thickness

pc1=0.2;  // pin clearance
d1=15+fil1; // pin depth
h1=10;  // pin height off surface.
e1=10;  // pin offset offend

h2=10;  // under cut from pin slot.

t0=h1+h2+t1; // thickness of thin section.

f1=5;  // offset to diagonal track
g1=15;  // length of diagonal track
a1=45;  // angle of diagonal track
j1=40;  // length of track parallel to end
p1=t1*1.1; // pin latch radius

nob=3; // number of bolts



module panelBody(x,t,d,l) {
    
    // Error Checking
    if(2*d1>d){
           echo("<font color='red'>Error: d1 must be less than half of d!</font>");
           assert(false);
    };
       
    xf=xn(f,x,l); // final x coord
    
    // Line up at pin slot
    xp=xn(f,x,l-e1-f1);  // xp location of pin
    yp=fx(f,xp);   // yp location of pin
    ap=da(f,xf);  // angle of tangent @ end face   
    np=t-t1-h1; // the magnitude along normal
    dx=np*sin(-ap);
    dy=np*cos(-ap);
    
    // Line up at pin body
    xb=xn(f,x,-e1-f1); // xb location of body
    yb=fx(f,xb);  // yb location of pin
    ab=da(f,x); //angle of tangent @ start face
    dxb=np*sin(-ab);
    dyb=np*cos(-ab);
    
    
     // Sculpting
    difference(){
    offset_3d(r=fil1)
    difference(){
        
        // Parabolic section; initial block
        ParabolicSection(f,fx(f,ymax),d,t,x,xn(f,x,l));
        
        // Cut-away for pin slot
        difference(){
            ParabolicSection(f,fx(f,ymax),fil1,t,x,xn(f,x,l));
            ParabolicLength(f,x-0.1,t-h1-t1,d1+1,l-e1-f1-g1*sin(a1)-p1/2-h2);
        }
        
        // pin travel guide
        translate([0,-1,0])
        intersection(){
            ParabolicLength(f,x-0.1,t-h1,d1+1,l-e1+fil1); //
            
        difference(){
            ParabolicLength(f,x-0.1,t,d1+1,l-e1+fil1); //
            ParabolicLength(f,x-0.1,t-h1-t1,d1+1,l-e1+fil1); //
        }
        }          
      
      // pin slot  
         pinSlot(xp+dx,yp+dy,-ap);
        
     // Parabolic section; undercut
        translate([0.1,-0.1,-0.1])
        ParabolicLength(f,x-1,t-h1-t1-h2,d+1,l-e1-f1-g1*sin(a1)-p1/2-h2/2); 
        
    };

    // Bolt holes
    for (i = [1 : nob ]) {
            bl=(l-e1-f1-g1*sin(a1)-p1/2-h2)/(nob+1)*i;
            bx=xn(f,x,bl); // location of bolt
            by=fx(f,bx);  
            bz=(d-2*d1)*1/2+2*d1;  
            ba=da(f,bx); //angle of tangent 
            bdx=(t-h1-t1-h2-fil1)*sin(-ab);
            bdy=(t-h1-t1-h2-fil1)*cos(-ab);
        
            // 8mm bolt
            boltHole(4,8,10,bx+bdx,by+bdy,bz,-ba);
    }
    
   translate([0,2*d1,0])
        intersection(){
            ParabolicLength(f,x-0.1-2*fil1,t-h1,d,l+4*fil1); //
            
        difference(){
            ParabolicLength(f,x-0.1-2*fil1,t,d,l+4*fil1); //
            ParabolicLength(f,x-0.1-2*fil1,t-h1-t2-pc1,d,l+4*fil1); //
        }
    }
    
    
};
    if (x>0) {
    // Pin Body
        offset_3d(r=fil1)  // fillet w/ 3doffset
        difference() {            
            pinBody(xb+dxb,yb+dyb,-ab);
            translate([0,-t1-1,0])
            difference() {                
                // Parabolic section; initial block
                ParabolicSection(f,fx(f,ymax),d1+1,2*t,x,xn(f,x,l));
                     // Parabolic section; undercut
                translate([0.1,-0.1,-0.1])
                ParabolicLength(f,x-1,t-h1-t1,d1+1,l-e1-f1-g1*sin(a1)-p1/2-h2/2); 
            }
        }
    }
}

module pinSlot(x,y,a) {
    
    xtravel=t1*sin(a1/2)*sin(a1/2)-g1*sin(a1);
    ytravel=t1*sin(a1/2)*cos(a1/2)-g1*cos(a1)-j1;
    
    if(ytravel<t0){
           echo("<font color='red'>Error: The ytravel for the pin slot must be more than t!</font>");
           assert(false);
    };
    
    // Generates a pin slot at the required location.
    translate([x,0,y])
    rotate([0,a,0])
    
        // Set to zero
        translate([t1*sin(a1/2)*sin(a1/2),0,t1*sin(a1/2)*cos(a1/2)])
        translate([-g1*sin(a1),0,-g1*cos(a1)])  
        translate([0,-0.9,-j1]) 
            union(){
        
                // Parallel slide
                //compensate for edges/angles
                translate([-t1*sin(a1/2)*sin(a1/2),0,-t1*sin(a1/2)*cos(a1/2)]) 
                //center on end of diagonal
                translate([g1*sin(a1),0,g1*cos(a1)])    
                translate([-t1/2,0,0])
                        cube([t1,d1+1,j1+t1/2]);
        
                // Angular slide
                rotate([0,a1,0])
                translate([-t1/2,0,0])
                    cube([t1,d1+1,g1]);
        
                // Pinhole
                rotate([-90,0,0])
                    cylinder(r=p1/2,h=d1+1);
           }
}


module pinBody(x,y,a) {
    // Generates a pin body at the required location.
    xtravel=t1*sin(a1/2)*sin(a1/2)-g1*sin(a1);
    ytravel=t1*sin(a1/2)*cos(a1/2)-g1*cos(a1)-j1;
    
    at=t1; // arm thickness

    translate([x,0,y])
    rotate([0,a,0])     
    translate([xtravel,-pc1,ytravel]) 
        union() {  
            // arm
            rotate([0,45,0])
            translate([0,-at+pc1,-t1/2+pc1])
            ParabolicSolid(1,t+t1,at);
                     
            // pin connection
            rotate([-90,0,0])
            cylinder(r=pc1,h=d1+pc1-fil1);
        }
}

module boltHole(r1,r2,c1,x,y,z,a){
    translate([x,z,y])
    rotate([0,a,0])     
    union() {
    cylinder(r=r1,h=t); // Thru hole
    cylinder(r=r2,h=c1); // Counter Sink
    }
}

//offset_3d(r=fil1)
//union(){
//panelBody(0,t,d,200);
//mirror([x,0,0])
//panelBody(0,t,d,200);
//}
//panelBody(190,t,d,200);
panelBody(334,t,d,200);



