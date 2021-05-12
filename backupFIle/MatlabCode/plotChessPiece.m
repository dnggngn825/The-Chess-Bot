function plotChessPiece(x,y,z,zlim,radius)
    
    angle = [-pi:0.1:pi];
    og_R = radius;


    z_array = linspace(0,zlim,1000);
    for i = z_array
        if (i<10)
            radius = -(0.5*i-2)^2+20;
        elseif (i<30)
            radius = 1/(0.01*i)+1;
        elseif (i<35)
            radius = -(0.8*i-26)^2+8;
        else
            radius =  1/(0.01*i)+1;
        end
        x_array = x*ones(size(angle)) + radius.*cos(angle);
        y_array = y*ones(size(angle)) + radius.*sin(angle);
%         plot3(x_array',y_array',(i+z)*ones(size(x_array')),'.');
        fill3(x_array',y_array',(i+z)*ones(size(x_array')),[224/255 224/255 224/255]);
        alpha(.5);
        hold on;

        
    
    end
        plot3(x,y,z+25,'-o','color','b','LineWidth',5);hold on

end