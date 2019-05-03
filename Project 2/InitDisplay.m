function [] = InitDisplay(StartNode, GoalNode, res)

    close all
    figure(1);
    set(gcf, 'Position', [1536*0.1 864*0.1 1536*0.6 864*0.6]);
    axis([0 250.5 0 150.5]);
    grid on;
    grid minor;
    hold on;

    % establish block
    bX = [-0.5,-0.5,0.5,0.5,-0.5]*res;
    bY = [-0.5,0.5,0.5,-0.5,-0.5]*res;
    % add in start and goal nodes
    patch(StartNode(1)+bX,StartNode(2)+bY,'g');
    patch(GoalNode(1)+bX,GoalNode(2)+bY,'r');

    % add in obstacles
    if res >= 1
        for x = 0:250
            for y = 0:150
            c = EvalCrash([res*floor(x/res),res*floor(y/res)], res);
                if c
                    patch(res*floor(x/res)+bX,res*floor(y/res)+bY,'k');
                end
            end
        end
    else
%         x = 0; 
%         while x <= 250
%             y = 0;
%             while y <= 150
%                 c = EvalCrash([x,y], res);
%                 if c
%                     patch(x+bX,y+bY,'k');
%                 end
%                 y = y + res;
%             end
%             x = x + res;
%         end
        xs = [55,55,105,105,55];
        ys = [67.5,112.5,112.5,67.5,67.5];
        fill(xs,ys,'k');
        t = 0:360;
        xc = 15*cosd(t)+180;
        yc = 15*sind(t)+120;
        fill(xc,yc,'k');
        xa = [120,158,165,188,168,145,120];
        ya = [55,51,89,51,14,14,55];
        fill(xa,ya,'k');
    end

    % add boarders to obstacles
    % Square
    x1 = 55:105;
    y1 = 67.5*ones(size(x1));
    y2 = 112.5*ones(size(x1));
    y3 = 67.5:112.5;
    x3 = 55*ones(size(y3));
    x4 = 105*ones(size(y3));
    plot(x1,y1,'Color',[0.5 0.5 0.5],'LineWidth',2)
    plot(x1,y2,'Color',[0.5 0.5 0.5],'LineWidth',2)
    plot(x3,y3,'Color',[0.5 0.5 0.5],'LineWidth',2)
    plot(x4,y3,'Color',[0.5 0.5 0.5],'LineWidth',2)

    % Circle
    t = 0:360;
    x5 = 15*cosd(t)+180;
    y5 = 15*sind(t)+120;
    plot(x5,y5,'Color',[0.5 0.5 0.5],'LineWidth',2)

    % Arbitrary Obstacle Divided into 4 Triangles
    x1 = 120:145;
    y1 = ((14-55)/(145-120))*x1 + (55-(120)*(14-55)/(145-120));
    x2 = 120:158;
    y2 = (51-55)/(158-120)*x2 + (55-(120)*(51-55)/(158-120));
    x3 = 145:158;
    y3 = ((51-14)/(158-145))*x3 + (14-145*(51-14)/(158-145));
    x4 = 145:168;
    y4 = 14*ones(size(x4));
    x5 = 158:168;
    y5 = ((14-51)/(168-158))*x5 + (51-158*(14-51)/(168-158));
    x6 = 158:188;
    y6 = 51*ones(size(x6));
    x7 = 168:188;
    y7 = ((51-14)/(188-168))*x7 + (14-168*(51-14)/(188-168));
    x8 = 158:165;
    y8 = ((89-51)/(165-158))*x8 + (51-158*(89-51)/(165-158));
    x9 = 165:188;
    y9 = ((51-89)/(188-165))*x9 + (89-165*(51-89)/(188-165));

    plot(x1,y1,'Color',[0.5 0.5 0.5],'LineWidth',2)
    plot(x2,y2,'Color',[0.5 0.5 0.5],'LineWidth',2)
    plot(x3,y3,'Color',[0.5 0.5 0.5],'LineWidth',2)
    plot(x4,y4,'Color',[0.5 0.5 0.5],'LineWidth',2)
    plot(x5,y5,'Color',[0.5 0.5 0.5],'LineWidth',2)
    plot(x6,y6,'Color',[0.5 0.5 0.5],'LineWidth',2)
    plot(x7,y7,'Color',[0.5 0.5 0.5],'LineWidth',2)
    plot(x8,y8,'Color',[0.5 0.5 0.5],'LineWidth',2)
    plot(x9,y9,'Color',[0.5 0.5 0.5],'LineWidth',2)
    % hold off

    pause(0.001); % to allow user to see the display

end

