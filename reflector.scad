use<parabola.scad>
use<fillet.scad>
use<MCAD/metric_fastners.scad>

$fn=121;

scale=0.25;     // model scaled to this size on slic3r
tol=0.2/scale;  // print manufacturing best tolerance
htol=0.1/scale/2+tol; // hole clearance

// Overall Dimensions
f=150; // parabola focus height, mm
fillet=5; // fillet radius, mm 

a=ParabolaFocus(f);
w=70; // overall part width, mm
t=14*2; // nominal thickness, mm
l=800; // reflector length available, mm
tube=127.4; // Tube diameter, mm

x_max=ParabolaTravel(a=a,x=0,l=l/2,step=0,n=1000); // maximum x dimension, mm       
y_max=Parabola(a=a,x=x_max)[1];

no_of_sections = 8; // The no. of sections to make the reflector in 
l_sect=l/(no_of_sections); // The length of a single section.
x_sect=cat(
            [ for( i = [ 0 : no_of_sections ] ) 
            ParabolaTravel(a=a,x=-x_max,l=i*l_sect,step=0,n=1000) ],
            [x_max]
            );
            
            
feet=[3,6]; // The parabolic sections which will have the feet attached 
clasp=[no_of_sections/2,no_of_sections/2+1];; // Parabolic sections with a clasp attachment
            
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
                wb=w,  // Extrusion, width, wb
                o=[fillet,t-fillet],  // Normal vector from surface parabola at 0
                f=[fillet,-fillet],  // Tangent Length from each edge, x0+f, x1+f
                fillet=fillet,
                ) { 
                                        // (follows curve)
      translate([0,-fillet,0])
        offset_3d(r=fillet){
        rotate([90,0,0])
        linear_extrude(height=wb-2*fillet)
        ParabolaPolygon(a=a,
            x=[x_sect[p-1],x_sect[p]],
            o=o,
            f=f);
        }
}

module claspNo(i){
    // The main function for generated the tube clasps.
    TubeConn1(clasp[i]);
    
}

module legNo(i) {
// The main function for generating load bearing legs.
    partLeg(feet[i]);
}

module partNo(i) {
// The main function for generating each part.
    difference()
    {
    union() {
        basePart(i);
        
        // Compensation angle
        
        // Positive for free hanging surfaces
        // Negative values for weight bearing surfaces
        compAngle=atan(htol/(t/4));
        
        if ( i >= feet[0] && i < feet[1] ) {
            pinConn1(pn=i,a=-compAngle);
        }
        else {
            pinConn1(pn=i,a=compAngle);
        }
        
    }   
        pinHole1(pn=i+1);
        
        boltHoles(pn=i);
    
        // Nest with the centre parts at the top
        // Outer parts at the bottom
        if ( nestedPart(i) > 0 ) {
            boltNest(pn=i,nestPart=nestedPart(i));
            }

        translate([0,-w,0])
        partEmbossID(pn=i);
        partEmbossID(pn=i);
            
        // Slot for sheet metal.
        Slot(i);   
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
                
// Calculate distance and angle to x-axis and y-axis along normal
function partNormal(p=1,o=0,f=0,n=$fn,
                    a=0,  // Compensation angle
                    h=0  // extra height
                    ) =
    let(A=( p[0] <= no_of_sections/2 ? partAngle(p)-a : partAngle(p)+a),
        XYZ=partCoord(p,-o,f,n))
    let( xc = XYZ[0]+(XYZ[2]+h)*tan(A),
         yc = (XYZ[2]+h)+XYZ[0]/tan(A),
         Lnx = (XYZ[2]+h)/cos(A),
         Lny = XYZ[0]/sin(A)
                )
        [ xc,            // xc, xintersection
          yc,           // yc, yintersection
           Lnx,          // Lnx, Perpendicular Length to xc
           Lny          // Lny, Perpendicular Length to yc
                ];
                
                
// Generates a parabolic slot for part no
module Slot(i=1,
            gat=0.9, // Gauge thickness, here ali 0.9mm            
            cutthru=5 // Cutthru depth, use more than PinConn1 here
    ) 
{
    translate([0,cutthru,0])
    basePart(p=i,wb=w+2*tol+2*cutthru,o=[t/2-gat/2,t/2+gat/2],f=[tol,-tol],fillet=tol);
}
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
        cylinder(r1=pin_radius,r2=pin_radius+htol,h=w+cut_thru*2-cci/2);
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

// bolt assembly with tolerances for making a hole
module boltHole(bolt_size=[ 5,t],  // Bolt spec 5mm dia,  M2.5, 20mm deep
                center=false
                ) {
                    
     if ( center ) {
         translate([0,0,-(bolt_size[1]+tol)/2])
        // bolt assembly                  
        boltAssembly(bolt_size=[bolt_size[0]+2*htol,
                    bolt_size[1]+0.7*(bolt_size[0]+2*htol)+tol-bolt_size[0]*0.3],
                    hole=true); 
     } 
     else {
          boltAssembly(bolt_size=[bolt_size[0]+2*htol,
                        bolt_size[1]+0.7*(bolt_size[0]+2*htol)+tol-bolt_size[0]*0.3],
                        hole=true); 
     }
}

function boltWidthSpacing(wb) =
    w*(1/(wb+1));

function boltLengthSpacing(nb, pin_length = t/2+t/4 ) =
    let( l_array=l_sect-pin_length )
        l_array*(1/(nb+1));

function boltOffset(pn,bolt_size,wb) =
        ( wb == 1 ? pow(-1,pn)*(boltWidthSpacing(wb)/(2)):
                    pow(-1,pn)*(boltWidthSpacing(wb)/(4)));

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
            nb=2, // Number of bolts along parabola
            wb=1,  // Number of bolts along width
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
                t=t,
                da=0, // Depth adjustment
                ) {
      // bolt array                  
    boltArray(pn,
            db=da+tol,   // Bolt depth 
            bolt_size=[ bolt_size[0]+2*htol,t+0.7*(bolt_size[0]+2*htol)+tol],
            hole=true); 
}

// Provides the part no. to nest with part no. i
function nestedPart(i) =
    ( i < no_of_sections/2 ? 
        ( i > 1  ? i-1 : 0 ) :
        ( i < no_of_sections ? i+1 : 0 )
    );

// Generates a fillet for a cylinder
module cylinderFillet(r=1.0, fillet=5,tol=tol) {
    difference() {
        cylinder(r=r+fillet,
             h=fillet);

        translate([0,0,fillet+tol])
        rotate_extrude(convexity = 10)
        translate([r+fillet, 0, 0])
        circle(r = fillet, $fn=20);
    }
}

// Provides a pattern for nesting bolts inside each subsequent parabola.
module boltNest(pn,
                nestPart,
                bolt_size=[ 5,20],  // Bolt spec 5mm dia,  M2.5, 20mm deep        
                tol=[tol, htol],    // Tolerances for clearance
                fillet=fillet       // fillet on the bolt nest
                ) {
                    
    echo("boltOffset=",boltOffset(pn+1,bolt_size,2));
                    
    // Nests pn+1 into pn                 
    partTranslate(pn=[nestPart,pn], xyz=[[0,0,0],[0,0,0]], center=true) 
    // Offset bolts (for nesting)
    translate([0,boltOffset(nestPart,bolt_size,2),0])
    
    partSurfaceArray(pn=nestPart, nb=2, wb=1, db=tol[0]) 
    // Bolt assembly contruction
    translate([0,0,-t])
                    union() {
                        cylinder(r=bolt_size[0]+tol[1]+tol[0],
                                    h=bolt_size[1]+tol[1]+tol[0]);
                        if (fillet>0) { 
                            cylinderFillet(r=bolt_size[0]+tol[1]+tol[0],
                                            fillet=fillet);
                        }
                    }
                
}

// Creates an embossed id for each part number
module partEmbossID(pn,
                    width_spacing=[fillet,4],
                    length_spacing=[t/2+fillet,4,8],    // The spacing between edges, imprints and groups.
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








// Create a leg that connects to part pn
module partLeg(i,
                it=0.5*t,       // interface thickness
                compA=-10,        // compensation angle
                h=t+5,             // Additional height
                leg=[t/2,w-2*fillet,t/2],  // Leg dimensions, x,y,t 
                fillet=3,            // fillet radius
                foot=[21,21,4,3], // foot dimensions, [x,y,z,t], t>fillet
                ) {


        // Part coordinates
        pc=partCoord(p=[i,i+1],o=0,f=0,n=$fn);
        at=( i <= no_of_sections/2 ? partAngle(p=[i,i+1])-compA : partAngle(p=[i,i+1])+compA); // tangent
        an=90-at;  // normal
             
        // Embed into middle of interface o=-t-it/2
        N=partNormal(p=[i,i+1],o=-t-it/2,f=0,n=$fn,a=compA,h=h);
        xc=N[0];
        yc=N[1];
        Lnx=N[2];
        Lny=N[3];
                    
        ax=atan(Lnx/(leg[1]/2));         // Angle around the x-axis to centre
        Lnz=norm([Lnx, leg[1]/2]);       // The length of the extrusion required.

            
        // Interface section to part no. i
        if ( nestedPart(i) > 0 ) {
            boltNest(pn=i,nestPart=nestedPart(i),tol=[0,0]);
            }        
        basePart(i,o=[t+fillet, (t+it)-fillet]);
            
            
       // Leg main
       c1=abs(leg[0]*tan(at)/2); // Leg compensation to meet xc,0   
       c2=abs(leg[0]/tan(ax)/2); // Compensation to angle toward center foot  
       
       
       //main leg shapes
       LegCube1=[leg[0],leg[2],Lnz+c1+c2]-2*[fillet,fillet,fillet];   
       
       //ground cut-off
       LegCube2=[(leg[0]+(Lnz-Lnx)/abs(2*tan(at)))/cos(at) ,                 
                leg[1],
                (leg[0]+(Lnz-Lnx)/abs(2*tan(at)))*abs(sin(at))   
                ]+2*[fillet,fillet,fillet];  
           
      // foot
      FootCube0 = [ foot[0],foot[1],foot[2]];   //base
      FootCube1 = FootCube0 - 2*fillet*[1,1,0]; //interior
      FootCube2 = FootCube0 + foot[3]*[ 2, 2, 1 ] - 2*fillet*[1,1,1]; //exterior
          
       difference() {  
        offset_3d(r=fillet){  
            union() {    
                difference() {              
                    // main section  
                    translate([xc,0,-h])    // attach to foot
                    rotate([0,-at,0])       // rotate to meet parabola
                    translate([0,-w/2,0])   // center in parabola                 
                       union() {  
                           // legs

                           rotate([-(90-ax),0,0])
                           translate([0,-(LegCube1[1]/2),0])
                           translate([0,0,-c2-c1])  // compensation 
                           translate([0,0,LegCube1[2]/2]) // center
                           cube(LegCube1,center=true); //main
                          
                           rotate([90-ax,0,0])
                           translate([0,(LegCube1[1]/2),0])
                           translate([0,0,-c2-c1]) // compensation 
                           translate([0,0,LegCube1[2]/2]) // center
                           cube(LegCube1,center=true); //main
                       }
                       
                    // Cut-off cube  
                    translate([xc,-w/2,-LegCube2[2]/2+fillet-h])
                    cube(LegCube2,center=true);
                       
                   }
                   
           translate([xc,-w/2,FootCube2[2]/2+fillet-h])
           cube(FootCube2,center=true);
                }
            }
     // Foot
        offset_3d(r=fillet){  
            translate([xc,-w/2,FootCube1[2]/2-fillet-h])
            cube(FootCube1,center=true);
                }           
            }
}




// Create a connection for the solar tube
module TubeConn1(i,
        it=t/2,
        wb=1, // Number of bolts across width
        bolt_size=[ 5,20],
        wa=w/2, // The width of the Tube connection
        compA=0,
       // h=-tube/2,
        bolt_section=60,
        fillet=10
        ) {
          
       // Compensate for tube and bolt section.     
        h=tube/2+bolt_section;
        r=tube/2; // The radius of the tube section.    
        b=bolt_section; // The length of the bolt section. 
        mi=(i > no_of_sections/2 ? 1 : -1); // Mirror section var   
            
    // Part coordinates
        pc=partCoord(p=i+(1+mi)/2,o=-it/2,f=mi*-it/2,n=$fn);
        xi=pc[0];
        yi=pc[2];
            
        xj=mi*(r+it/2);
        yj=f;
            
        y0=f; // The focus point
        y1=f-r;   // where the bolt section will attach to the arc
        y2=f-r-b; // Where the parabola will attach to the bolt section.
        y3=f+r;
        y4=f+r+b;  // The top bolt section.
            
        ata=atan((yj-yi)/(xj-xi)); // Angle for parabolic extrusion to arc 
        atl=norm([yj-yi,xj-xi]); // Length
                               
        aw=atan(atl*2/w);  // Alpha angle, change in angle required (x-axis)
        atlw=pow(pow(atl,2)+pow(w/2,2),0.5); // The new length based on additional angle
        
        woffset=-w/2+boltOffset(i,bolt_size,wb);
            
    // Parabola interface to reflector
    // fillet for improved strength
    difference() {
    //fillet(r=10,steps=5) {  // uncomment for fillets
        union(){
    // Offset bolts (for nesting)
    translate([0,wa/2+woffset,0])
    basePart(i,
            wb=wa,
            o=[0-fillet, -it+fillet]);

    
   // Cuboid from section on face to clasp
    translate([xi,0,yi])
    translate([0,woffset,0])
    rotate([-mi*(90-aw)/2,0,0])
    rotate([0,mi*90-ata,0])   
    translate([0,0,-atlw/2]) 
    cube([it,it,atlw],center=true);         
 
   // First bolt section cuboid
    //translate([0,0,y2])
    translate([0,-w/2,0])
    translate([mi*it/2,0,y1/2])
    offset_3d(r=fillet/2){
        cube([it-fillet,wa-fillet,y1-fillet],center=true); 
     }
     
     // Top bolted section cubiod
    translate([0,0,y3])
    translate([0,-w/2,0])
    translate([mi*it/2,0,bolt_section/2])
    offset_3d(r=fillet/2){
        cube([it-fillet,wa-fillet,b-fillet],center=true); 
    }
    
   // Tube exterior
    translate([0,-w/2,0])
    translate([0,0,y0])
    translate([0,it,0])
    rotate([90,0,0])
    cylinder(r=tube/2+it,h=2*it);
    }

  // CUTS
  
  // Tube hole
    translate([0,-w/2+it/2,0])
    translate([0,0,y0])
    translate([0,it,0])
    rotate([90,0,0])
    union() {
    cylinder(r=tube/2,h=3*it);
     translate([-mi*(tube+3*it)/2,0,(3*it+tol)/2])
    cube([tube+3*it,tube+2*it+2*b,3*it+tol],center=true); 
    }
    
    // bolt holes
    translate([0,0,y2+b/2])
    translate([0,-w/2,0])
    rotate([0,-90,0])
    boltHole(bolt_size=[bolt_size[0],it*2],center=true);  

    translate([0,0,y3+b/2])
    translate([0,-w/2,0])
    rotate([0,-90,0])
    boltHole(bolt_size=[bolt_size[0],it*2],center=true); 
    
    // Matching bolt holes for parabola
    boltHoles(i,
                bolt_size,  // Bolt spec 5mm dia,  M2.5, 20mm deep
                t=t+it,
                da=0
                );
    } 
    

}
//
legNo(1);
partNo(4);

//for (i=[1:no_of_sections]) { partNo(i); };
//for (i=[1:2]) { legNo(i); };
//for (i=[1:2]) { claspNo(i); };


