// Variables X, Y, Z, box_x, box_y, left_foot, and fn are now passed via command line.
// Still need to first define it.
X=0; Y=0; Z=0; box_x=0; box_y=0; left_foot=0; fn=0; 

// Semi-axes
a = X/2; 
b = Y/2; 
c = Z/2;

hx = box_x/2;
hy = box_y/2;

// z_height of foot determined by square intersection
z_top = c * sqrt(1 - (hx*hx)/(a*a) - (box_y*box_y)/(b*b));

// Box runs from z = -c up to z_top
box_z0 = -c;
box_height = c - z_top;

translate([0,0,c - box_height])
// Intersection of a sphere and a cube
intersection() {
  resize([X, Y, Z]) sphere(r = 1, $fn = fn);
  translate([ 0, left_foot * hy, box_height/2 - c])
    cube([ box_x, box_y, box_height ], center = true);
}