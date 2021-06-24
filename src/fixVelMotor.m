function newVel = fixVelMotor(vel)
newVel = vel;
newVel(1) = -vel(1);

end