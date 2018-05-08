// Set nan variable.
nan=1/0;

// Concatenate two vectors
function cat(L1, L2) = [for(L=[L1, L2], a=L) a];
    
// Sums a vector
function sumv(v,i=nan,s=0) = 
    (i==nan ? 
        let( i = len(v)) 
            sumv(v=v,i=i-1,s=s) :
    (i==s ? v[i] : v[i] + sumv(v,i-1,s))
    );


function ParabolaFocus(f)
    = 1/4/f;


// Returns the solution to the parabolic equation.
// ax^2+bx+c = y
// solve for x
// solve for y
// solve for x, + perpendicular offset to curve, o
// solve for x, + elongating the length of the curve, f
// solve for x, + o + f
function Parabola(a,b=0,c=0,x=nan,y=nan,o=0,f=0,n=$fn) =
    (f==0 ?
    (o==0 ?
        (y==nan ? [x,a*pow(x,2)+b*pow(x,1)+c*pow(x,0)] : // Solve for x
              
              [(-b-pow(pow(b,2)-4*a*(c-y),0.5))/(2*a),
              (-b+pow(pow(b,2)-4*a*(c-y),0.5))/(2*a)]):  // Solve for y      
        let(  x0=x,
              y0=Parabola(a,b,c,x,o=0)[1], 
              an=ParabolaNormal(a,b,c,x,angle=true)[0])
            ( x>=-b/2/a ?     
            [ x0+o*cos(an), y0+o*sin(an) ]:   // Add offset         
            [ x0-o*cos(an), y0-o*sin(an) ])): // Add offset behind center
        let( xn=ParabolaTravel(a,b,c,x,l=f,n=n))
              Parabola(a,b,c,xn,o=o,f=0));  //Add length
            
// Returns the gradient of the parabola at different sections.
function ParabolaGradient(a,b=0,c=0,x=nan,y=nan,angle=false) =
    (angle==false ?
    (y==nan ? [2*a*x+b] :
        2*a*Parabola(a,b,c,y=y) + [b,b] ) :
    [ for (i=ParabolaGradient(a,b,c,x=x,y=y,angle=false))atan(i) ]); 

function ParabolaNormal(a,b=0,c=0,x=nan,y=nan,angle=false) =
    (angle==false ?
        [ for (i=ParabolaGradient(a,b,c,x=x,y=y,angle=angle)) -1/i ] :
        [ for (th=ParabolaNormal(a,b,c,x=x,y=y,angle=false)) atan(th) ]);  

// Use eulers method to travel a length around the parabola.
function ParabolaTravel(a,b=0,c=0,x,l=1,step=0,n=1) = 
    (step >= n ? x : 
                  ParabolaTravel(a,b,c,
                  x + l/n*cos(
                    ParabolaGradient(a,b,c,x=x,angle=true)[0]),
                  l, step+1, n));

module ParabolaPolygon(a,b=0,c=0,x=[],y=[],o=[0,1],f=[0,0],n=$fn+20){        
    // Adjust x based on offset length
    x=[ for(i=[0,1]) ParabolaTravel(a,b,c,x[i],l=f[i],n=n)];
        echo("x=",x);
    
    // Resolution of Polygon
    xres = (x[1]-x[0])/n;
    
    // Polygon generator
    polygon(
        cat(
        [for (xi=[x[0]:xres:x[1]]) 
            Parabola(a,b,c,x=xi,o=o[0])
        ],
        [for (xi=[x[1]:-xres:x[0]]) 
            Parabola(a,b,c,x=xi,o=o[1])
        ]));
    }
  

// Testing
echo(cat([1,2],[3,4,5]));
echo("sumv([1,2,3,4,5])", sumv([1,2,3,4,5]));  
echo("Parabola(1,2,3,x=5)", Parabola(1,2,3,x=5));
echo("Parabola(1,2,3,y=38)", Parabola(1,2,3,y=38));
echo("Gradient(1,2,3,x=5)", ParabolaGradient(1,2,3,x=5));
echo("Gradient(1,2,3,y=38)", ParabolaGradient(1,2,3,y=38));
echo("Gradient(1,2,3,x=5)", ParabolaGradient(1,2,3,x=5,angle=true));
echo("Gradient(1,2,3,x=38)", ParabolaGradient(1,2,3,y=38,angle=true));
    echo("Normal(1,2,3,x=5)", ParabolaNormal(1,2,3,x=5,angle=true));
echo("Normal(1,2,3,x=38)", ParabolaNormal(1,2,3,y=38,angle=true));
echo("Travel(l=10,n=1)", ParabolaTravel(1,2,3,x=5,l=10,n=1));
echo("Travel(l=10,n=10)", ParabolaTravel(1,2,3,x=5,l=10,n=10));
echo("Travel(l=10,n=100)", ParabolaTravel(1,2,3,x=5,l=10,n=100));

echo("Polygon(x=[1,10])");
   
//nose cone
rotate_extrude(convexity=10,$fn=12)
ParabolaPolygon(1/2/15,0,0,x=[0,0],,o=[0,2],f=[0,30],n=100);

//parabolic extrusion
rotate([90,0,0])
linear_extrude(height=300)
ParabolaPolygon(1/2/15,0,0,x=[0,0],,o=[0,0.9],f=[-30,30]);
    
