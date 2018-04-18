function h = drawCube ( origin, size,R )

% Draw cube animation. Cube is red.
%
% Inputs: 
% origin: origin of cube (center of cube)
% size: cube size (length of side)
% R: orientation (rotation matrix)
%
% Outputs:
% h: cube plot handle
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% scale cube
x=([0 1 1 0 0 0;1 1 0 0 1 1;1 1 0 0 1 1;0 1 1 0 0 0]-0.5)*size;
y=([0 0 1 1 0 0;0 1 1 0 0 0;0 1 1 0 1 1;0 0 1 1 1 1]-0.5)*size;
z=([0 0 0 0 0 1;0 0 0 0 0 1;1 1 1 1 0 1;1 1 1 1 0 1]-0.5)*size;

% rotate cube
for i = 1:6
   for k = 1:4
      temp = (R*[x(k,i), y(k,i), z(k,i)]')'; 
      x(k,i) = temp(1);
      y(k,i) = temp(2);
      z(k,i) = temp(3);
   end
end

% offset origin
x = x + origin(1);
y = y + origin(2);
z = z + origin(3);

% draw cube using patch
for i=1:6
    h(i) =patch(x(:,i),y(:,i),z(:,i),'r');
    set(h(i),'edgecolor','k')
end