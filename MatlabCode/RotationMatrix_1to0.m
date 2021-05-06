% Rotation matrix from frame {1} to {0}: 1_R_0

function R = RotationMatrix_1to0(angleInRad, axis)
if (isnumeric(angleInRad))
    angleInDeg = angleInRad*180/pi;
    switch(axis)
            case {'z', 'Z'}
                R = [cosd(angleInDeg) -sind(angleInDeg) 0;...
                        sind(angleInDeg) cosd(angleInDeg) 0;
                        0 0 1];
            case {'x', 'X'}
                R = [1 0 0;...
                        0 cosd(angleInDeg) -sind(angleInDeg);...
                        0 sind(angleInDeg) cosd(angleInDeg)];
            case {'y', 'Y'}
                R = [cosd(angleInDeg) 0 -sind(angleInDeg);...
                        0 1 0;...
                        sind(angleInDeg) 0 cosd(angleInDeg)];
    end
else
      switch(axis)
            case {'z', 'Z'}
                R = [cos(angleInRad) -sin(angleInRad) 0;...
                        sin(angleInRad) cos(angleInRad) 0;
                        0 0 1];
            case {'x', 'X'}
                R = [1 0 0;...
                        0 cos(angleInRad) -sin(angleInRad);...
                        0 sin(angleInRad) cos(angleInRad)];
            case {'y', 'Y'}
                R = [cos(angleInRad) 0 -sin(angleInRad);...
                        0 1 0;...
                        sin(angleInRad) 0 cos(angleInRad)];
    end
end
end