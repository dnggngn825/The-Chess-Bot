function plotChessBoard()
    [X_chess, Y_chess, width_chess, singleSquare]= ChessboardSpec();
    
    colorChess = [ 	54, 54, 54]/260;
    for t = 1:9
    plot3([X_chess+width_chess*(t-1)/8 X_chess+width_chess*(t-1)/8],[-width_chess/2 -width_chess/2+width_chess],[Y_chess Y_chess],'color',colorChess,'LineWidth',2);
    hold on;
    plot3([X_chess X_chess+width_chess],[-width_chess/2+width_chess*(t-1)/8 -width_chess/2+width_chess*(t-1)/8],[Y_chess Y_chess],'color',colorChess,'LineWidth',2);
    hold on;
    end
    plot3([340;340;460;460]/1000,[160;240;240;160]/1000,[0;0;0;0],'LineWidth',2,'Color',colorChess);hold on
end