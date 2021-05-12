function R = RotationTransformMatrix_1to0(angleInRad,axis)
    R = [RotationMatrix_1to0(angleInRad,axis) [0;0;0];0 0 0 1];
end