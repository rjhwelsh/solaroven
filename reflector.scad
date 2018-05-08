use<parabola.scad>
use<fillet.scad>
use<MCAD/metric_fastners.scad>

$fn=121;

scale=0.25;     // model scaled to this size on slic3r
tol=0.2/scale;  // print manufacturing best tolerance
htol=0.2/scale/2+tol; // hole clearance

// Overall Dimensions
f=150; // parabola focus height, mm
fillet=5; // fillet radius, mm 

a=ParabolaFocus(f);
w=100; // overall part width, mm
t=14*2; // nominal thickness, mm
l=800; // reflector length available, mm

x_max=ParabolaTravel(a=a,x=0,l=l/2,step=0,n=1000); // maximum x dimension, mm       
y_max=Parabola(a=a,x=x_max)[1];

no_of_sections = 8; // The no. of sections to make the reflector in 
l_sect=l/(no_of_sections); // The length of a single section.
x_sect=cat(
            [ for( i = [ 0 : no_of_sections ] ) 
            ParabolaTravel(a=a,x=-x_max,l=i*l_sect,step=0,n=1000) ],
            [x_max]
            );
echo("x max=",x_max,"y max", y_max);   
echo("x sections=",x_sect); 
echo("l sections=",l_sect);      


// PLA Strength Calc
ali_density=2700; // kg/m^3
ali_t=0.9;        //mm
ali_density2=ali_density*ali_t*1e-3; // kg/m^2
reflector_panel_length=600; //mm
ali_density1=ali_density2*reflector_panel_length*1e-3; //kg/m         
Fw=ali_density1*l_sect*9.8; // N , Weight per panel 
Fw2=Fw+(8/1000)*6*2*9.8; // N, Weight per bolt assembly per panel 
uts_pla=50; // MPa, N/mm^2
FOS=10;      // Factor of Safety
dist_a=l_sect/2; //mm
dist_b=l_sect/2;       //mm
dist_c=2*dist_a-dist_b; //mm

F1=dist_a/dist_b*Fw; // N
F2=(dist_a*Fw + dist_c*F1)/dist_b; // N
F3=(dist_a*Fw + dist_c*F2)/dist_b; // N

echo("F1=",F1);
echo("F2=",F2);
echo("F3=",F3);
      
function Fmax(a,b,c,Fw,Fp=0,n=0) =
    ( n==0 ? Fp :
        let( Fn = (a*Fw + c*Fp)/b )
            Fmax(a,b,c,Fw,Fn,n=n-1) );
            
            
Fm=Fmax(dist_a,dist_b,dist_c,Fw2,Fp=0,n=no_of_sections/2);
echo("Fmax=",Fm);

ReqArea=Fm*FOS/uts_pla;  // Required Area for UTS

echo("Area=",ReqArea); //mm^2
echo("Square=",pow(ReqArea,0.5)); //mm
echo("Radius=",pow(ReqArea/3.14,0.5)); //mm

module basePart(p=1,  // Part no.
                a=a,  // Parabolic a from a*x^2
                w=w-2*fillet,  // Extrusion, width, w
                o=[fillet,t-fillet],  // Normal vector from surface parabola at 0
                f=[fillet,-fillet]) { // Tangent Length from each edge, x0+f, x1+f
                                        // (follows curve)
      translate([0,-fillet,0])
        offset_3d(r=fillet){
        rotate([90,0,0])
        linear_extrude(height=w)
        ParabolaPolygon(a=a,
            x=[x_sect[p-1],x_sect[p]],
            o=o,
            f=f);
        }
}

module partNo(i) {
// The main function for generating each part.
    rotate([90,0,0])
    difference()
    {
    union() {
        basePart(i);
        pinConn1(pn=i,a=atan(htol/(t/4)));
        partEmbossID(pn=i);
    }   
        pinHole1(pn=i+1);
        boltHoles(pn=i);
    }
}

function partCoord(p=1,o=0,f=0,n=$fn) =
    ( len(p) > 1 ?       // Take an average of all partNos provided.
        let(ps=[ for (i=p) x_sect[i-1]] )
        let(fn=Parabola(a,x=sumv(ps)/len(ps),o=o,f=f,n=n))
     [ fn[0],
        0,
      fn[1]]:   // Otherwise         
        let(fn=Parabola(a,x=x_sect[p-1],o=o,f=f,n=n))
    [ fn[0],
        0,
      fn[1]]);

function partAngle(p=1) =
         ( len(p) > 1 ?       // Take an average of all partNos provided.
                let(ps=[ for (i=p) x_sect[i-1]] )
                ParabolaGradient(a,x=sumv(ps)/len(ps),angle=true)[0]:
                // otherwise
         ParabolaGradient(a,x=x_sect[p-1],angle=true)[0]);

module pinConn1(pn=1, 
                pin_radius=t/4, // Pin_radius thickness
                pin_neck=t/3,  // Pin_neck thickness
                a=0,           // Compensation angle   
                cut_thru=1.5, // Cut thru depth
                cci=1 // cylinder cube interface dimension
                        // negative values = longer cylinder
                        // positive values = longer neck vs cylinder
                ){
    // Translation to nominal part
    translate([0,cut_thru,0])
    translate(partCoord(p=pn,o=t/2,f=-t/4-t/4,n=100))                   
    rotate([0,-partAngle(pn),0])
    // Compensation angle for tolerances
        translate([t/2,0,0])
        rotate(a=a,v=[0,1,0])
        translate([-t/2,0,0]) 
    // Pin construction           
    rotate([90,0,0])
    union(){
        translate([t/2,0,w/2+cut_thru])
            cube([t,pin_neck,w+cut_thru*2+cci/2],center=true);
        translate([0,0,cci/4])
        cylinder(r=pin_radius,h=w+cut_thru*2-cci/2);
}
}

module pinHole1(pn=1){
    pinConn1(pn,
        pin_radius=t/4+htol,
        pin_neck=t/3+htol
        );
}

// Maps to the center of the specified partno.
module partTranslate(pn=1, xyz=[0,0,0], center=true) {
    if(center) {
        // Translate to center
        coord = partCoord(p=[pn,pn+1],o=-xyz[1],f=xyz[0],n=100);
        translate([0,-w/2+xyz[2],0])
        translate(coord)
        rotate([0,
                -ParabolaGradient(a,x=coord[0],angle=true)[0],
                0])
        children();
        }
    else {
        // Translate to edge
        coord = partCoord(p=pn,,o=-xyz[1],f=xyz[0],n=100);
        translate([0,xyz[2],0])
        translate(coord)
        rotate([0,
                -ParabolaGradient(a,x=coord[0],angle=true)[0],
                0])
        children();
        }
}

// bolt Assembly
module boltAssembly(pn,
            nb=3, // Number of bolts along parabola
            wb=2,  // Number of bolts along width
            db=htol-htol,   // Bolt depth, positioning 
            bolt_size=[ 5,20],  // Bolt spec 5mm dia,  M2.5, 20mm deep
            hole=false
            ) { 
    
    for (k=[1:wb]) {
    for (i=[1:nb]) {
        
    xyz=[-l_sect/2+l_sect*(i/(nb+1)),
        -db,
        -w/2+w*(k/(wb+1))];
              
    // Map to Part
    partTranslate(pn=pn, xyz=xyz, center=true) 
    
    // Bolt assembly contruction
    translate([0,0,-t])
    union() {
        
        if (!hole) {
            bolt(bolt_size[0]
                ,bolt_size[1]); // 5mm Bolt, M2.5, weight 8g
        }
        else {
            dia=bolt_size[0];
            e=1.8*dia;
            k=0.7*dia;
            c=0.2*dia;
            
            union() {
            translate([0,0,-k])
            cylinder(r=e/2,h=2*k,$fn=6);
            cylinder(r=dia/2,h=bolt_size[1]);
            }
        }
                
        translate([0,0,bolt_size[1]-bolt_size[0]*0.8])
        union(){
            
          if (!hole) {
                translate([0,0,bolt_size[0]*0.1])
                flat_nut(bolt_size[0]); // M2.5 NUT
               washer(bolt_size[0]); // M2.5 Flat Washer
            }
            
          if (hole) { 
                translate([0,0,0])
                cylinder(h=bolt_size[0]*0.3,r=bolt_size[0]); 
              } //  Clearance           
        }
    }
    }
}
}
module boltHoles(pn) {
    boltAssembly(pn,
            db=tol,   // Bolt depth 
            bolt_size=[ 5+2*htol,t+0.7*(5+2*htol)+tol],
            hole=true);  // Bolt spec 5mm  M2.5, 20mm deep);
}

// Creates an embossed id for each part number
module partEmbossID(pn,
                    spacing=4,                     // The spacing between imprints
                    fillet=fillet,                 // The fillet to apply
                    size=l_sect/no_of_sections/4, // The size of each imprint
                    ) {
    total_length=size*pn+spacing*(pn-1);
    for (j=[1:pn]) {
        xyz=[l_sect/2-total_length/2+j/pn*total_length,
                                -t/2,
                                0]; 
        offset_3d(r=fillet/2){
    partTranslate(pn=pn, 
                    xyz=xyz, 
                        center=false) {
    if (size < fillet) {cube(1e-5,center=true);}
    else {cube(size-fillet,center=true);}
    }
    }
    } 
}

for (i=[1:no_of_sections]) {
partNo(3);
}



