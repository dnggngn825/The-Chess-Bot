% Rotation matrix from frame {1} to {2}: 2_R_1

function R = RotationMatrix_1to2(angleInRad, axis)
switch(axis)
        case {'z', 'Z'}
            R = [cos(angleInRad) sin(angleInRad) 0;...
                    -sin(angleInRad) cos(angleInRad) 0;
                    0 0 1];
        case {'x', 'X'}
            R = [1 0 0;...
                    0 cos(angleInRad) sin(angleInRad);...
                    0 -sin(angleInRad) cos(angleInRad)];
        case {'y', 'Y'}
            R = [cos(angleInRad) 0 sin(angleInRad);...
                    0 1 0;...
                    -sin(angleInRad) 0 cos(angleInRad)];
end