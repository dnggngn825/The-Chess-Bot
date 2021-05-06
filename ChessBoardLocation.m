function [XI,XF] = ChessBoardLocation(start,final)
default = [260;140;50];
corner = 'a1';

if (strcmp(start,'home'))
    XI = [240;0;300];
elseif (strcmp(start,'cp'))
    XI = [400;200;50];
else
    XI = [default(1)+(start(2)-corner(2))*40;...
        default(2)-(start(1)-corner(1))*40;...
        50];
end


if (strcmp(final, 'cp'))
    XF = [400;200;50];
elseif (strcmp(final,'home'))
    XF = [240;0;300];
else
    XF = [default(1)+(final(2)-corner(2))*40;...
            default(2)-(final(1)-corner(1))*40;...
            50];
end
end