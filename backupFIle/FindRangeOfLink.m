function FindRangeOfLink(d,side)

long = sqrt((d+side)^2+(side/2)^2);

min = long*0.5/cos(pi/36);

max = d/(sqrt(2-sqrt(2)));
fprintf("Min: ");
disp(min);
fprintf("Max: ");
disp(max);
fprintf("Link length is: %f\n",(min+max)/2);

end