function [] = InitDisplay(StartNode, GoalNode,res)

    close all
    figure(1);
    set(gcf, 'Position', [1536*0.1 864*0.1 1536*0.6 864*0.6]);
    axis equal
    grid on;
    grid minor;
    hold on;

    % add in obstacles

    % table 1
    ct1 = [5.525, -3.3750]; % m, center
    st1 = [0.8, 2]; % m, size
    xt1 = [ct1(1)+st1(1)/2,ct1(1)+st1(1)/2,ct1(1)-st1(1)/2,ct1(1)-st1(1)/2,ct1(1)+st1(1)/2];
    yt1 = [ct1(2)+st1(2)/2,ct1(2)-st1(2)/2,ct1(2)-st1(2)/2,ct1(2)+st1(2)/2,ct1(2)+st1(2)/2];
    fill(xt1,yt1,'k');
    
    % table 2
    ct2 = [4.325, -3.3750]; % m, center
    st2 = [0.8, 2]; % m, size
    xt2 = [ct2(1)+st2(1)/2,ct2(1)+st2(1)/2,ct2(1)-st2(1)/2,ct2(1)-st2(1)/2,ct2(1)+st2(1)/2];
    yt2 = [ct2(2)+st2(2)/2,ct2(2)-st2(2)/2,ct2(2)-st2(2)/2,ct2(2)+st2(2)/2,ct2(2)+st2(2)/2];
    fill(xt2,yt2,'k');
    
    % table 3
    ct3 = [1.1475, 0]; % m, center
    st3 = [2, 3.2]; % m, size 
    xt3 = [ct3(1)+st3(1)/2,ct3(1)+st3(1)/2,ct3(1)-st3(1)/2,ct3(1)-st3(1)/2,ct3(1)+st3(1)/2];
    yt3 = [ct3(2)+st3(2)/2,ct3(2)-st3(2)/2,ct3(2)-st3(2)/2,ct3(2)+st3(2)/2,ct3(2)+st3(2)/2];
    fill(xt3,yt3,'k');
    
    % table 4
    ct4 = [-2.6025, 0]; % m, center
    st4 = [2, 3.2]; % m, size 
    xt4 = [ct4(1)+st4(1)/2,ct4(1)+st4(1)/2,ct4(1)-st4(1)/2,ct4(1)-st4(1)/2,ct4(1)+st4(1)/2];
    yt4 = [ct4(2)+st4(2)/2,ct4(2)-st4(2)/2,ct4(2)-st4(2)/2,ct4(2)+st4(2)/2,ct4(2)+st4(2)/2];
    fill(xt4,yt4,'k');
    
    % table 5
    ct5 = [-5.35, -4.1]; % m, center
    st5 = [1.6, 1]; % m, size
    xt5 = [ct5(1)+st5(1)/2,ct5(1)+st5(1)/2,ct5(1)-st5(1)/2,ct5(1)-st5(1)/2,ct5(1)+st5(1)/2];
    yt5 = [ct5(2)+st5(2)/2,ct5(2)-st5(2)/2,ct5(2)-st5(2)/2,ct5(2)+st5(2)/2,ct5(2)+st5(2)/2];
    fill(xt5,yt5,'k');
    
    % table 6
    ct6 = [-6.95, -0.275]; % m, center
    st6 = [0.8, 2]; % m, size
    xt6 = [ct6(1)+st6(1)/2,ct6(1)+st6(1)/2,ct6(1)-st6(1)/2,ct6(1)-st6(1)/2,ct6(1)+st6(1)/2];
    yt6 = [ct6(2)+st6(2)/2,ct6(2)-st6(2)/2,ct6(2)-st6(2)/2,ct6(2)+st6(2)/2,ct6(2)+st6(2)/2];
    fill(xt6,yt6,'k');
    
    % table 7
    ct7 = [-6.95, 1.725]; % m, center
    st7 = [0.8, 2]; % m, size
    xt7 = [ct7(1)+st7(1)/2,ct7(1)+st7(1)/2,ct7(1)-st7(1)/2,ct7(1)-st7(1)/2,ct7(1)+st7(1)/2];
    yt7 = [ct7(2)+st7(2)/2,ct7(2)-st7(2)/2,ct7(2)-st7(2)/2,ct7(2)+st7(2)/2,ct7(2)+st7(2)/2];
    fill(xt7,yt7,'k');
    
%     t = 0:360;
%     r;
%     xc;
%     yc;
%     xcr = r*cosd(t)+xc;
%     ycr = r*sind(t)+yc;
%     fill(xcr,ycr,'k');
    

    % establish block
    th = 0:15:360;
    bX = (res/4)*cosd(th);
    bY = (res/4)*sind(th);
    % add in start and goal nodes
    global n
    n = [];
    n(2) = patch(StartNode(1)+bX,StartNode(2)+bY,'g');
    n(1) = patch(GoalNode(1)+bX,GoalNode(2)+bY,'r');
    global np
    np = [];
    
    pause(0.001); % to allow user to see the display

end

