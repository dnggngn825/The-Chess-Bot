function [XI,XF] = ChessBoardLocation(start,final)

% XI and XF in meters

default = [260;140;50];
corner = 'a1';
heightStart = 80;
heightEnd = 150;

if (strcmp(start,'home'))
    XI = [300;0;310]/1000;
elseif (strcmp(start,'cp'))
    XI = [400;200;heightStart]/1000;
else
    XI = [default(1)+(start(2)-corner(2))*40;...
        default(2)-(start(1)-corner(1))*40;...
        heightStart]/1000;
end


if (strcmp(final, 'cp'))
    XF = [400;200;heightEnd]/1000;
elseif (strcmp(final,'home'))
    XF = [300;0;310]/1000;
else
    XF = [default(1)+(final(2)-corner(2))*40;...
            default(2)-(final(1)-corner(1))*40;...
            20*(final(2)-corner(2))+80]/1000;
end
end