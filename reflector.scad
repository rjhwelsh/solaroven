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
    //rotate([90,0,0])
    difference()
    {
    union() {
        basePart(i);
        pinConn1(pn=i,a=atan(htol/(t/4)));
        
    }   
        pinHole1(pn=i+1);
        
        boltHoles(pn=i);
        boltNest(pn=i);
    
        translate([0,-w,0])
        partEmbossID(pn=i);
        partEmbossID(pn=i);
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
        // Translate to another part?
        if ( len(pn)==2 ) { 
                coord0= partCoord(p=[pn[0],pn[0]+1],o=-xyz[0][1],f=xyz[0][0],n=100);
                coord1= partCoord(p=[pn[1],pn[1]+1],o=-xyz[1][1],f=xyz[1][0],n=100);
                coord =  coord1 - coord0;                      
                        // Translates from pn=0 to pn=1
               angle0 =-ParabolaGradient(a,x=coord0[0],angle=true)[0];
               angle1 =-ParabolaGradient(a,x=coord1[0],angle=true)[0];
               angle = angle1 - angle0;
            
            // Translate only according to relative
                translate([0,xyz[1][2]-xyz[0][2],0])
                translate(coord1) 
                rotate(a=angle, v=[0, 1, 0])
                translate(-coord0) // coord0 is used as 0,0,0
                children();
            }  
        else {
                coord = partCoord(p=[pn,pn+1],o=-xyz[1],f=xyz[0],n=100);
                angle =-ParabolaGradient(a,x=coord[0],angle=true)[0];
                            
                // Translate to center
                translate([0,-w/2+xyz[2],0])
                translate(coord)
                rotate(a=angle, v=[0, 1, 0])
                children();
            }

        
        }
    else {
        if ( len(pn)==2 ) { 
            // Translates from pn=0 to pn=1
            coord0 = partCoord(p=pn[0],o=-xyz[0][1],f=xyz[0][0],n=100);
            coord1 = partCoord(p=pn[1],o=-xyz[1][1],f=xyz[1][0],n=100);
            coord = coord1 - coord0;
            // Rotates from pn=0 to pn=1  
            angle0 =-ParabolaGradient(a,x=coord0[0],angle=true)[0];
            angle1 =-ParabolaGradient(a,x=coord1[0],angle=true)[0];
            angle = angle1 - angle0;
            
                // Translate only according to relative'
                translate([0,xyz[1][2]-xyz[0][2],0])
                translate(coord1)
                rotate(a=angle, v=[0, 1, 0])
                translate(-coord0) // coord0 is used as 0,0,0
                children();
            }
            
        else {        
            coord = partCoord(p=pn,o=-xyz[1],f=xyz[0],n=100);
            angle =-ParabolaGradient(a,x=coord[0],angle=true)[0];
            
                    // Translate to edge
                translate([0,xyz[2],0])
                translate(coord)
                rotate(a=angle, v=[0, 1, 0])
                children();
            }
        }
}

// bolt Assembly
module boltAssembly(bolt_size=[ 5,20],  // Bolt spec 5mm dia,  M2.5, 20mm deep
                    hole=false,         // Produce hole for bolt
                    ){
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
            cylinder(r=dia/2,h=bolt_size[1]); // Double bolt head for cutting thru material
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
// Provides an offset pattern for nesting parts together.
    // i.e the protruding bolt shaft, nut and washer.

function boltWidthSpacing(wb) =
    w*(1/(wb+1));

function boltLengthSpacing(nb, pin_length = t/2+t/4 ) =
    let( l_array=l_sect-pin_length )
        l_array*(1/(nb+1));

function boltOffset(pn,bolt_size,wb) =
        pow(-1,pn)*(boltWidthSpacing(wb)/4);

// Array over the surface of a part
module partSurfaceArray(pn, nb, // Number along parabolic curve
                            wb, // Number along width
                            db, // Depth positioning
                            pin_length=t/2+t/4      // The amount of space used by pinConn1
    ) {             
    for (k=[1:wb]) {
    for (i=[1:nb]) {        
        xyz=[-l_sect/2+i*boltLengthSpacing(nb,pin_length),
            -db,
            -w/2+(k)*boltWidthSpacing(wb)];
        
        // Map to Part
        partTranslate(pn=pn, xyz=xyz, center=true) 
        children();
    }
}
}

// An array of bolts for assembly of the reflective material
module boltArray(pn,
            nb=3, // Number of bolts along parabola
            wb=2,  // Number of bolts along width
            db=0,   // Bolt depth, positioning 
            bolt_size=[ 5,20],  // Bolt spec 5mm dia,  M2.5, 20mm deep
            hole=false 
            ) { 
      
    echo("boltOffset=",boltOffset(pn,bolt_size,2));
                
    // Offset bolts (for nesting)
    translate([0,boltOffset(pn,bolt_size,wb),0])
    
    partSurfaceArray(pn, nb, wb, db) 
    // Bolt assembly contruction
    translate([0,0,-t])
    boltAssembly(bolt_size=bolt_size,  
                 hole=hole
                    );
    }

// An array of bolt holes to match the bolts.
module boltHoles(pn,
                bolt_size=[ 5,20],  // Bolt spec 5mm dia,  M2.5, 20mm deep
                ) {
      // bolt array                  
    boltArray(pn,
            db=tol,   // Bolt depth 
            bolt_size=[ bolt_size[0]+2*htol,t+0.7*(bolt_size[0]+2*htol)+tol],
            hole=true); 
}

// Provides a pattern for nesting bolts inside each subsequent parabola.
module boltNest(pn,
                bolt_size=[ 5,20],  // Bolt spec 5mm dia,  M2.5, 20mm deep
                ) {
                    
    echo("boltOffset=",boltOffset(pn+1,bolt_size,2));
    partTranslate(pn=[pn+1,pn], xyz=[[0,0,0],[0,0,0]], center=true) 
    // Offset bolts (for nesting)
    translate([0,boltOffset(pn+1,bolt_size,2),0])
    
    partSurfaceArray(pn=pn+1, nb=3, wb=2, db=tol) 
    // Bolt assembly contruction
    translate([0,0,-t])
    cylinder(r=bolt_size[0]+htol+tol,
             h=bolt_size[1]+htol+tol);
                    
}

// Creates an embossed id for each part number
module partEmbossID(pn,
                    width_spacing=[fillet,1],
                    length_spacing=[t/2+fillet,1,5],    // The spacing between edges, imprints and groups.
                    fillet=fillet/2,                 // The fillet to apply
                    length=l_sect-t/2-t/4,        // The total length of all the imprints
                    width=t,                        // The total width of all the imprints 
                    depth=w/100,                       // The impression depth into the part
                    group=[2,2],                        // Group into collections of 4x4
         // includes The compensation for the pinConn1
                    ) {
        
    // size of spacings
    size1=2*[length_spacing[0],  //edge 
             width_spacing[0],
             0] +                //imprint
                        [(no_of_sections/group[1]-1)*length_spacing[1],
                        (group[1]-1)*width_spacing[1],
                        0 ] +  
                                // group
                        [(no_of_sections/group[1]/group[0]-1)*length_spacing[2],
                        0,
                        0 ]  
                        ;
                        
    //echo("SIZE1",size1);
    
                        // size of cube
    size2=[length,width,0]-size1;
    //echo("SIZE2",size2); 
                        
    size3=[size2[0]*group[1]/no_of_sections,
           size2[1]/group[1],
                        0];
                                          
    // size of each imprint 
    size0=size3+[0,0,2*depth]; // The size of each imprint, l x w
    //echo("SIZE0",size0);
    
    // size of cube (-fillet compensation)                    
    size=[ max(size0[0]-2*fillet,1e-5),
           max(size0[1]-2*fillet,1e-5),
           max(size0[2]-2*fillet,1e-5)
                        ];
    //echo("SIZE=",size); 
 
    for (j=[1:pn]) {
        
        p=floor((j-1)/group[1]);
        q=(j-1)%group[1];
        r=floor((j-1)/(group[0]*group[1]));
        
        x0=size0[0]/2+length_spacing[0];
        y0=size0[1]/2+width_spacing[0];
        
        x=x0+p*(size0[0]+length_spacing[1])+r*(length_spacing[2]-length_spacing[1]);
        y=y0+q*(size0[1]+width_spacing[1]);
        
        offset_3d(r=fillet){
        partTranslate(pn=pn, 
                    xyz=[x,y-t,0], 
                        center=false) 
            cube(size=[size[0],
                       size[2],
                       size[1]], center=true);
    }
    } 
}





for (i=[1:no_of_sections]) {
    partNo(i);
}


