
clc; clear; close all;

env = imread('map.png');
map = env(:,:,1)/255;

map = rot90(map);

% Defining initial position of the robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robotposition1 = [25,72]; %You can change the robot position, just make 
% sure it does not collide with an obstacle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% To show where we started later on, I store the first initial position:
initial(1,1) = robotposition1(1,1);
initial(1,2) = robotposition1(1,2);

% Initial belief:
for i=1:200
    for j=1:200
        belief(i,j) = 1/200/200;
    end
end

Newmap = map;
% In the following for loop, I change the Newmap matrix according the
% statements(comments) in the loop.
for i=2:100
    for j=2:100
        if Newmap(i,j) == 0 % if there is a wall
            wallProb = probability; % gives a probability
            if wallProb <= 0.40 % if prob = 0.4 
                Newmap(i,j) = 0; % Lidar sees the wall as it is supposed to see
            elseif (0.40 <= wallProb) && wallProb <= 0.475 % if prob = 0.075
                Newmap(i,j) = 1; % Lidar sees the wall a free space
                Newmap(i-1,j-1) = 0; % Lidar sees the wall adjacent to real wall
            elseif (0.475 <= wallProb) && wallProb <= 0.550 % if prob = 0.075
                Newmap(i,j) = 1;% Lidar sees the wall a free space% Lidar sees the wall a free space
                Newmap(i,j-1) = 0; % Lidar sees the wall adjacent to real wall
            elseif (0.550 <= wallProb) && wallProb <= 0.625 % if prob = 0.075
                Newmap(i,j) = 1; % Lidar sees the wall a free space
                Newmap(i+1,j-1) = 0; % Lidar sees the wall adjacent to real wall
            elseif (0.625 <= wallProb) && wallProb <= 0.700 % if prob = 0.075
                Newmap(i,j) = 1; % Lidar sees the wall a free space
                Newmap(i+1,j) = 0; % Lidar sees the wall adjacent to real wall
            elseif (0.700 <= wallProb) && wallProb <= 0.775 % if prob = 0.075
                Newmap(i,j) = 1; % Lidar sees the wall a free space
                Newmap(i+1,j+1) = 0; % Lidar sees the wall adjacent to real wall
            elseif (0.775 <= wallProb) && wallProb <= 0.850 % if prob = 0.075
                Newmap(i,j) = 1; % Lidar sees the wall a free space
                Newmap(i,j+1) = 0; % Lidar sees the wall adjacent to real wall
            elseif (0.850 <= wallProb) && wallProb <= 0.925 % if prob = 0.075
                Newmap(i,j) = 1; % Lidar sees the wall a free space
                Newmap(i-1,j+1) = 0; % Lidar sees the wall adjacent to real wall
            else % if prob = 0.075
                Newmap(i,j) = 1; % Lidar sees the wall a free space
                Newmap(i-1,j) = 0; % Lidar sees the wall adjacent to real wall
            end
        end
    end
end

for i=1:200
    for j=1:200
        % matrixForLidar is a new matrix that combines the real map and new
        % map that lidar sees the walls with probability 0.40.
        % This way, I create a new algorithm that the robot's belief and
        % real map combined so that the robot cannot go thorough the real
        % walls and it does not contradict with its belief.
        
        % If one of the matrices has 0 in one cell, then matrixForLidar has
        % 0 in the same cell:
       matrixForLidar(i,j) = Newmap(i,j)*map(i,j); 
    end
end

matrixForLidar_ = matrixForLidar'; 

fh = figure();
fh.WindowState = 'maximized';

% Plot on the right: This will give the positions of the robot for start of
% cycles
subplot(1,2,2)
imagesc(matrixForLidar) % Showing combined matrix
hold on

%Grids:
for j = 1:200
    line([0.5,200.5], [0 + j+0.5,0 + j+0.5],"Color","k","Linewidth",0.5)
    hold on
    line([0 + j+0.5,0 + j+0.5],[0.5,200.5],"Color","k","Linewidth",0.5)
    hold on
end
scatter(robotposition1(1,1),robotposition1(1,2),30,"r","filled","s")
hold on

cyclesize = 4; % The cycle size that robot will move. You can change it.
total = 0;
for i = 1:cyclesize
    
    %Within each cycle, I update the probabilities of the cells that the
    %robot might be inside:
    down1 = [robotposition1(1,1),robotposition1(1,2)+1];
    down2 = [robotposition1(1,1),robotposition1(1,2)+2];
    down3 = [robotposition1(1,1),robotposition1(1,2)+3];
    down4 = [robotposition1(1,1),robotposition1(1,2)+4];
    down5 = [robotposition1(1,1),robotposition1(1,2)+5];
    
    up1 = [robotposition1(1,1),robotposition1(1,2)-1];
    up2 = [robotposition1(1,1),robotposition1(1,2)-2];
    up3 = [robotposition1(1,1),robotposition1(1,2)-3];
    up4 = [robotposition1(1,1),robotposition1(1,2)-4];
    up5 = [robotposition1(1,1),robotposition1(1,2)-5];
    
    right1 = [robotposition1(1,1)+1,robotposition1(1,2)];
    right2 = [robotposition1(1,1)+2,robotposition1(1,2)];
    right3 = [robotposition1(1,1)+3,robotposition1(1,2)];
    right4 = [robotposition1(1,1)+4,robotposition1(1,2)];
    right5 = [robotposition1(1,1)+5,robotposition1(1,2)];
    
    left1 = [robotposition1(1,1)-1,robotposition1(1,2)];
    left2 = [robotposition1(1,1)-2,robotposition1(1,2)];
    left3 = [robotposition1(1,1)-3,robotposition1(1,2)];
    left4 = [robotposition1(1,1)-4,robotposition1(1,2)];
    left5 = [robotposition1(1,1)-5,robotposition1(1,2)];

    if up1(:,2)<=0
       up1(:,2)=1;
    elseif up2(:,2)<=0
       up2(:,2)=1;
    elseif up3(:,2)<=0
       up3(:,2)=1;
    elseif up4(:,2)<=0
       up4(:,2)=1;
    elseif up5(:,2)<=0
       up5(:,2)=1;
    elseif left1(:,1)<=0
       left1(:,1)=1;
    elseif left2(:,1)<=0
       left2(:,1)=1;
    elseif left3(:,1)<=0
       left3(:,1)=1;
    elseif left4(:,1)<=0
       left4(:,1)=1;
    elseif left5(:,1)<=0
       left5(:,1)=1;
    end


    if down1(:,2)>=200
       down1(:,2)=199;
    elseif down2(:,2)>=200
       down2(:,2)=199;
    elseif down3(:,2)>=200
       down3(:,2)=199;
    elseif down4(:,2)>=200
       down4(:,2)=199;
    elseif down5(:,2)>=200
       down5(:,2)=199;

    elseif right1(:,1)>=200
       right1(:,1)=199;
    elseif right2(:,1)>=200
       right2(:,1)=199;
    elseif right3(:,1)>=200
       right3(:,1)=199;
    elseif right4(:,1)>=200
       right4(:,1)=199;
    elseif right5(:,1)>=200
       right5(:,1)=199;
    end

    totalchange = belief(down1(1,1),down1(1,2))*0.05+belief(down2(1,1),down2(1,2))*0.15...
        +belief(down3(1,1),down3(1,2))*0.6+belief(down4(1,1),down4(1,2))*0.15...
        +belief(down5(1,1),down5(1,2))*0.05...
    +belief(up1(1,1),up1(1,2))*0.05+belief(up2(1,1),up2(1,2))*0.15...
        +belief(up3(1,1),up3(1,2))*0.6+belief(up4(1,1),up4(1,2))*0.15...
        +belief(up5(1,1),up5(1,2))*0.05...
    +belief(left1(1,1),left1(1,2))*0.05+belief(left2(1,1),left2(1,2))*0.15...
        +belief(left3(1,1),left3(1,2))*0.6+belief(left4(1,1),left4(1,2))*0.15...
        +belief(left5(1,1),left5(1,2))*0.05...
    +belief(right1(1,1),right1(1,2))*0.05+belief(right2(1,1),right2(1,2))*0.15...
        +belief(right3(1,1),right3(1,2))*0.6+belief(right4(1,1),right4(1,2))*0.15...
        +belief(right5(1,1),right5(1,2))*0.05;
    
    
    belief(down1(1,1),down1(1,2)) = belief(down1(1,1),down1(1,2)) + belief(down1(1,1),down1(1,2))*0.05;
    belief(down2(1,1),down2(1,2)) = belief(down2(1,1),down2(1,2)) + belief(down2(1,1),down2(1,2))*0.15;
    belief(down3(1,1),down3(1,2)) = belief(down3(1,1),down3(1,2)) + belief(down3(1,1),down3(1,2))*0.60;
    belief(down4(1,1),down4(1,2)) = belief(down4(1,1),down4(1,2)) + belief(down4(1,1),down4(1,2))*0.15;
    belief(down5(1,1),down5(1,2)) = belief(down5(1,1),down5(1,2)) + belief(down5(1,1),down5(1,2))*0.05;
    
    belief(up1(1,1),up1(1,2)) = belief(up1(1,1),up1(1,2)) + belief(up1(1,1),up1(1,2))*0.05;
    belief(up2(1,1),up2(1,2)) = belief(up2(1,1),up2(1,2)) + belief(up2(1,1),up2(1,2))*0.15;
    belief(up3(1,1),up3(1,2)) = belief(up3(1,1),up3(1,2)) + belief(up3(1,1),up3(1,2))*0.60;
    belief(up4(1,1),up4(1,2)) = belief(up4(1,1),up4(1,2)) + belief(up4(1,1),up4(1,2))*0.15;
    belief(up5(1,1),up5(1,2)) = belief(up5(1,1),up5(1,2)) + belief(up5(1,1),up5(1,2))*0.05;
    
    belief(right1(1,1),right1(1,2)) = belief(right1(1,1),right1(1,2)) + belief(right1(1,1),right1(1,2))*0.05;
    belief(right2(1,1),right2(1,2)) = belief(right2(1,1),right2(1,2)) + belief(right2(1,1),right2(1,2))*0.15;
    belief(right3(1,1),right3(1,2)) = belief(right3(1,1),right3(1,2)) + belief(right3(1,1),right3(1,2))*0.60;
    belief(right4(1,1),right4(1,2)) = belief(right4(1,1),right4(1,2)) + belief(right4(1,1),right4(1,2))*0.15;
    belief(right5(1,1),right5(1,2)) = belief(right5(1,1),right5(1,2)) + belief(right5(1,1),right5(1,2))*0.05;
    
    belief(left1(1,1),left1(1,2)) = belief(left1(1,1),left1(1,2)) + belief(left1(1,1),left1(1,2))*0.05;
    belief(left2(1,1),left2(1,2)) = belief(left2(1,1),left2(1,2)) + belief(left2(1,1),left2(1,2))*0.15;
    belief(left3(1,1),left3(1,2)) = belief(left3(1,1),left3(1,2)) + belief(left3(1,1),left3(1,2))*0.60;
    belief(left4(1,1),left4(1,2)) = belief(left4(1,1),left4(1,2)) + belief(left4(1,1),left4(1,2))*0.15;
    belief(left5(1,1),left5(1,2)) = belief(left5(1,1),left5(1,2)) + belief(left5(1,1),left5(1,2))*0.05;

    belief = belief - totalchange/200/200;

    % Robot might know its position very close to 100% probability if we
    % apply a lot of cycles. If it is know its position, then there is no
    % need for further movement. The following loop does that.
    flag = 0;
    for k = 1:200
        for l = 1:200
            if belief(k,l) >= 1
                belief(k,l) = 1;
                flag = 1;                
                break
            end
        end
        if flag==1
            break
        end
    end
    
    % When robot knows where it is, a message is given about the situation:
    if flag==1
        cyclesize = i - 1;
        errordlg('Robot found its position with 100% probability at the cycle = '+string(cyclesize)+' There is no need for further movement','ROBOT FOUND WHERE IT IS');
        break
    end
                
    
    a = probability;
    if a <= 0.05 % Robot moves 1 cell with probability 0.05
        move = 1;
        cellprob = 0.05;
    elseif (0.05 < a)&& (a <= 0.20) % Robot moves 3 cell with probability 0.15
        move = 2;
        cellprob = 0.15;
    elseif (0.20 < a)&& (a <= 0.80) % Robot moves 3 cell with probability 0.60
        move = 3;
        cellprob = 0.60;
    elseif (0.80 < a)&& (a <= 0.95) % Robot moves 4 cell with probability 0.15
        move = 4;
        cellprob = 0.15;
    else % Robot moves 5 cell with probability 0.05
        move = 5;
        cellprob = 0.05;
    end

    % new probability defined to select the direction
    %(up,down,left or right with probabilities 0.25 for each):
    % up -->      b <= 0.25
    % down -->    0.25 < b <= 0.50
    % left -->    0.50 < b <= 0.75
    % right -->   0.75 < b <= 1.00
    
    % In the following if statements, I define a new algorithm which
    %prevents the robot to go that direction with some logic as well. 
    %Let's say robot will go up(b = 0.25) with 4 cells(0.80 < a <= 0.95)   
    %but there is a wall in the 3rd cell on the way, then the robot choose 
    %not to go that way because the robot actually was aware the wall
    %combination of up, down, left and right directions initially. It is
    %the best to not go to directions with walls in shorter distance than
    %the "move" command.
    b = probability; 
    dirprob = 0.25;
    if move == 1
        if matrixForLidar_(robotposition1(1,1),robotposition1(1,2)+1) == 1 && b <= 0.25
            robotposition1(1,2) = robotposition1(1,2) + move;
        elseif matrixForLidar_(robotposition1(1,1),robotposition1(1,2)-1)== 1 &&(0.25 < b)&& (b <= 0.50)
            robotposition1(1,2) = robotposition1(1,2) - move;
        elseif matrixForLidar_(robotposition1(1,1) + 1,robotposition1(1,2))== 1 &&(0.50 < b)&& (b <= 0.75)
            robotposition1(1,1) = robotposition1(1,1) + move;
        elseif matrixForLidar_(robotposition1(1,1)-1,robotposition1(1,2))== 1&&(0.75 < b)&& (b <= 1)
            robotposition1(1,1) = robotposition1(1,1) - move;
        end
    end
    
    
    if move == 2
        if matrixForLidar_(robotposition1(1,1),robotposition1(1,2)+1)== 1 && ...
                matrixForLidar_(robotposition1(1,1),robotposition1(1,2)+2)== 1 && b <= 0.25
            robotposition1(1,2) = robotposition1(1,2) + move;
        elseif matrixForLidar_(robotposition1(1,1),robotposition1(1,2)-1)== 1 && ...
                matrixForLidar_(robotposition1(1,1),robotposition1(1,2)-2)== 1 &&(0.25 < b)&& (b <= 0.50)
            robotposition1(1,2) = robotposition1(1,2) - move;
        elseif matrixForLidar_(robotposition1(1,1) + 1,robotposition1(1,2))== 1 && ...
                matrixForLidar_(robotposition1(1,1) + 2,robotposition1(1,2))== 1 &&(0.50 < b)&& (b <= 0.75)
            robotposition1(1,1) = robotposition1(1,1) + move;
        elseif matrixForLidar_(robotposition1(1,1)-1,robotposition1(1,2))== 1&& ...
                matrixForLidar_(robotposition1(1,1)-2,robotposition1(1,2))== 1&&(0.75 < b)&& (b < 1)
            robotposition1(1,1) = robotposition1(1,1) - move;
        end
    end
    
    if move == 3
        if matrixForLidar_(robotposition1(1,1),robotposition1(1,2)+1)== 1 &&...
                matrixForLidar_(robotposition1(1,1),robotposition1(1,2)+2)== 1 && ...
                matrixForLidar_(robotposition1(1,1),robotposition1(1,2)+3)== 1 && b <= 0.25
            robotposition1(1,2) = robotposition1(1,2) + move;
        elseif matrixForLidar_(robotposition1(1,1),robotposition1(1,2)-1)== 1 &&...
                matrixForLidar_(robotposition1(1,1),robotposition1(1,2)-2)== 1 && ...
                matrixForLidar_(robotposition1(1,1),robotposition1(1,2)-3)== 1 &&(0.25 < b)&& (b <= 0.50)
            robotposition1(1,2) = robotposition1(1,2) - move;
        elseif matrixForLidar_(robotposition1(1,1) + 1,robotposition1(1,2))== 1 &&...
                matrixForLidar_(robotposition1(1,1) + 2,robotposition1(1,2))== 1 && ...
                matrixForLidar_(robotposition1(1,1) + 3,robotposition1(1,2))== 1 &&(0.50 < b)&& (b <= 0.75)
            robotposition1(1,1) = robotposition1(1,1) + move;
        elseif matrixForLidar_(robotposition1(1,1)-1,robotposition1(1,2))== 1&&...
                matrixForLidar_(robotposition1(1,1)-2,robotposition1(1,2))== 1&& ...
                matrixForLidar_(robotposition1(1,1)-3,robotposition1(1,2))== 1&& (0.75 < b)&& (b < 1)
            robotposition1(1,1) = robotposition1(1,1) - move;
        end
    end
    
    
    
    if move == 4
        if matrixForLidar_(robotposition1(1,1),robotposition1(1,2)+1)== 1 &&...
                matrixForLidar_(robotposition1(1,1),robotposition1(1,2)+2)== 1 && ...
                matrixForLidar_(robotposition1(1,1),robotposition1(1,2)+3)== 1 && ...
                matrixForLidar_(robotposition1(1,1),robotposition1(1,2)+4)== 1 && b <= 0.25
            robotposition1(1,2) = robotposition1(1,2) + move;
        elseif matrixForLidar_(robotposition1(1,1),robotposition1(1,2)-1)== 1 &&...
                matrixForLidar_(robotposition1(1,1),robotposition1(1,2)-2)== 1 && ...
                matrixForLidar_(robotposition1(1,1),robotposition1(1,2)-3)== 1 && ...
                matrixForLidar_(robotposition1(1,1),robotposition1(1,2)-4)== 1 &&(0.25 < b)&& (b <= 0.50)
            robotposition1(1,2) = robotposition1(1,2) - move;
        elseif matrixForLidar_(robotposition1(1,1) + 1,robotposition1(1,2))== 1 &&...
                matrixForLidar_(robotposition1(1,1) + 2,robotposition1(1,2))== 1 && ...
                matrixForLidar_(robotposition1(1,1) + 3,robotposition1(1,2))== 1 && ...
                matrixForLidar_(robotposition1(1,1) + 4,robotposition1(1,2))== 1 &&(0.50 < b)&& (b <= 0.75)
            robotposition1(1,1) = robotposition1(1,1) + move;
        elseif matrixForLidar_(robotposition1(1,1)-1,robotposition1(1,2))== 1&&...
                matrixForLidar_(robotposition1(1,1)-2,robotposition1(1,2))== 1&& ...
                matrixForLidar_(robotposition1(1,1)-3,robotposition1(1,2))== 1&& ...
                matrixForLidar_(robotposition1(1,1)-4,robotposition1(1,2))== 1 &&(0.75 < b)&& (b < 1)
            robotposition1(1,1) = robotposition1(1,1) - move;
        end
    end
    
    
    
    
    if move == 5
        if matrixForLidar_(robotposition1(1,1),robotposition1(1,2)+1)== 1 &&...
                matrixForLidar_(robotposition1(1,1),robotposition1(1,2)+2)== 1 && ...
                matrixForLidar_(robotposition1(1,1),robotposition1(1,2)+3)== 1 && ...
                matrixForLidar_(robotposition1(1,1),robotposition1(1,2)+4)== 1 && ...
                matrixForLidar_(robotposition1(1,1),robotposition1(1,2)+5)== 1 &&b <= 0.25
            robotposition1(1,2) = robotposition1(1,2) + move;
        elseif matrixForLidar_(robotposition1(1,1),robotposition1(1,2)-1)== 1 &&...
                matrixForLidar_(robotposition1(1,1),robotposition1(1,2)-2)== 1 && ...
                matrixForLidar_(robotposition1(1,1),robotposition1(1,2)-3)== 1 && ...
                matrixForLidar_(robotposition1(1,1),robotposition1(1,2)-4)== 1 && ...
                matrixForLidar_(robotposition1(1,1),robotposition1(1,2)-5)== 1 &&(0.25 < b)&& (b <= 0.50)
            robotposition1(1,2) = robotposition1(1,2) - move;
        elseif matrixForLidar_(robotposition1(1,1) + 1,robotposition1(1,2))== 1 &&...
                matrixForLidar_(robotposition1(1,1) + 2,robotposition1(1,2))== 1 && ...
                matrixForLidar_(robotposition1(1,1) + 3,robotposition1(1,2))== 1 && ...
                matrixForLidar_(robotposition1(1,1) + 4,robotposition1(1,2))== 1 && ...
                matrixForLidar_(robotposition1(1,1) + 5,robotposition1(1,2))== 1 &&(0.50 < b)&& (b <= 0.75)
            robotposition1(1,1) = robotposition1(1,1) + move;
        elseif matrixForLidar_(robotposition1(1,1)-1,robotposition1(1,2))== 1&&...
                matrixForLidar_(robotposition1(1,1)-2,robotposition1(1,2))== 1&& ...
                matrixForLidar_(robotposition1(1,1)-3,robotposition1(1,2))== 1&& ...
                matrixForLidar_(robotposition1(1,1)-4,robotposition1(1,2))== 1 && ...
                matrixForLidar_(robotposition1(1,1)-5,robotposition1(1,2))== 1 &&(0.75 < b)&& (b < 1)
            robotposition1(1,1) = robotposition1(1,1) - move;
        end
    end

    if robotposition1(1,1) >= 200
        robotposition1(1,1) = 194;
    elseif robotposition1(1,1) <= 0
        robotposition1(1,1) = 6;
    end

    if robotposition1(1,2) >= 200
        robotposition1(1,2) = 198;
    elseif robotposition1(1,2) <= 0
        robotposition1(1,2) = 2;
    end 
    % I create positionarray object to let you run lidardata for any cycle.
    positionarray(i,1) = robotposition1(1,1); 
    positionarray(i,2) = robotposition1(1,2);
        
    
    % Plot on the left: This will show you how the probability distribution
    % changes within the map. 
    subplot(1,2,1)
    imagesc(belief')
    axis image
    colorbar
    title("Probability distrubition when the cycle is "+string(i))
    hold off
  
    
    %Updating the plot on the right for each movement.
    %subplot(1,2,2) 
    %axis equal

    total = total + 1;
    %scatter(robotposition1(1,1),robotposition1(1,2),30,"r","filled","s")

    %title("Positions robot has been at until the cycle "+string(i))
            
    pause(0.01)
end
subplot(1,2,2)
scatter(robotposition1(1,1),robotposition1(1,2),30,"b","filled","s")
title("Final Position of the robot at the end of "+string(cyclesize)+" cycles (Red is the initial position / Blue is the final position)")
pause(20)

fh = figure();
fh.WindowState = 'maximized';
imagesc(matrixForLidar)
axis equal
hold on

%This figure will show the final position of the robot and the lidar data
%of it(what it can see) according to the conditions and wall probabilities
%in the homework. (matrixForLidar already does the probability job.)
for j = 1:200
    line([0.5,200.5], [0 + j+0.5,0 + j+0.5],"Color","k","Linewidth",0.5)
    hold on
    line([0 + j+0.5,0 + j+0.5],[0.5,200.5],"Color","k","Linewidth",0.5)
    hold on
end
for i=1:total
    scatter(positionarray(i,1),positionarray(i,2),30,"r","filled","s")
    title("Positions Robot has been at during Localization, Cycle Number = "+string(i))
    hold on
    pause(0.000001)
end

% Robot now knows its location. It will use its location for path planning
% and control
FINAL_POSITION = robotposition1;
goal = [150,180]; % you can change the goal

env = imread('map.png');
map = env(:,:,1)/255;

map = map();
% heatmap(map)
% colormap(gray)


x_grid = linspace(0,10,200);
y_grid = linspace(0,10,200);

x_in = FINAL_POSITION(1);
y_in = FINAL_POSITION(2);


robot_gridx = find(x_in >= x_grid,1, 'last');
robot_gridy = find(y_in >= y_grid,1, 'last');

map = int8(~map);

x_goal = 180;
y_goal = 198;
goal_gridx = find(x_goal >= x_grid,1, 'last');
goal_gridy = find(y_goal >= y_grid,1, 'last');

[path, path_and_map] = path_planning(rot90(map), [y_in x_in], [y_goal x_goal]);

fh = figure();
fh.WindowState = 'maximized';
imagesc((path_and_map))
colormap(jet)
axis equal

title("Path Planning and the Map")
for i = 1:200 
    line([0.5,200.5], [0 + i+0.5,0 + i+0.5],"Color","k","Linewidth",0.5)
    hold on
    line([0 + i+0.5,0 + i+0.5],[0.5,200.5],"Color","k","Linewidth",0.5)
    hold on
end

% Below is the controller that follows the waypoints given by the path
% planner

theta = 0;

l = 0.16;
r = 0.033;
lb = l/2;
wheelR=0.033;
zeta(1,1) = path(1,2);
zeta(2,1) = path(1,1);
zeta(3,1) = 0;


for i = 1:length(path)-1 %animation starts here
    rho = ((x_goal-x_in)^2 + (y_goal-y_in)^2)^(1/2);
    alpha = -theta + atan2((y_goal-y_in),(x_goal-x_in));
    beta = -theta - alpha;
    RotationMatrix = [cos(theta), sin(theta), 0;
                 -sin(theta), cos(theta), 0;
                 0, 0, 1];

    k_rho = 0.0001;
    k_alpha = 0.0001;
    k_beta = 0.0001;

    rho_dot = -k_rho*rho*cos(alpha);
    alpha_dot = k_rho * sin(alpha) - k_alpha *alpha - k_beta * beta;
    beta_dot = -k_rho* sin(alpha);

    kine_mat = [-cos(alpha) 0; sin(alpha)/rho -1; -sin(alpha)/rho 0];

    vel_vec = inv(kine_mat' * kine_mat) * kine_mat' *  [rho_dot; alpha_dot; beta_dot];

    lin_vel = vel_vec(1);
    rot_vel = vel_vec(2);
    
    phi_dot_r = (lin_vel+rot_vel*l)/r;
    phi_dot_l = (lin_vel-rot_vel*l)/r;

    zeta(1,i) = transpose(path(i,2));
    zeta(2,i) = transpose(path(i,1));
    zeta(3,i) = theta;

    psi_dot = [phi_dot_r; phi_dot_l];
    A = [-sin(alpha+beta), cos(alpha+beta), lb*cos(beta);
               -sin(alpha+beta), cos(alpha+beta), -lb*cos(beta);
               cos(alpha+beta), sin(alpha+beta), lb*sin(beta);
               cos(alpha+beta), sin(alpha+beta), -lb*sin(beta);];
     
    B = [wheelR, 0;
               0, wheelR;
               0, 0;
               0, 0;];

    dt = rho/((phi_dot_r*r)^2+(phi_dot_l*r)^2);
    [phi_dot_r,phi_dot_l];
           
    zeta_dot(:,i) = inv(RotationMatrix)*inv(transpose(A)*A)...
              *transpose(A)*B*psi_dot; % zeta_dot derivation
    zeta(:,i+1) = zeta(:,i) + dt * zeta_dot(:,i);

    scatter(zeta(1,i),zeta(2,i),30,"g",'filled','s') 

    hold on
    
    zeta(1,i+1) = x_goal;
    zeta(2,i+1) = y_goal;
    theta = zeta(3,i);

    pause(0.001)
end



function [x_new, y_new, theta_new] = control(x_goal, y_goal, x_in, y_in, theta_in)
    
    rho = ((x_goal-x_in)^2 + (y_goal-y_in)^2)^(1/2);
    alpha = -theta_in + atan2((y_goal-y_in)/(x_goal-x_in));
    beta = -theta - alpha;

    k_rho = 1;
    k_alpha = 1;
    k_beta = 1;

    rho_dot = -k_rho*rho*cos(alpha);
    alpha_dot = k_rho * sin(alpha) - k_alpha *alpha - k_beta * beta;
    beta_dot = -k_rho* sin(alpha);

    kine_mat = [-cos(alpha) 0; sin(alpha)/rho -1; -sin(alpha)/rho 0];

    vel_vec = inv(kine_mat^T * kine_mat) * kine_mat^T *  [rho_dot; alpha_dot; beta_dot];

    lin_vel = vel_vec(1);
    rot_vel = vel_vec(2);
    
    phi_dot_r = (lin_vel+rot_vel*l)/r;
    phi_dot_l = (lin_vel-rot_vel*l)/r;

end

function [path, path_and_map] = path_planning(grid, init, goal)
    cost = 1;
    heuristic = zeros(size(grid,1), size(grid,2));
    for i = 1:size(grid,1)
        for j = 1:size(grid,2)
            heuristic(i,j) = abs(i - goal(1)) + abs(j - goal(2));
            if grid(i,j) == 1
                heuristic(i,j) = 99;
            end
        end
    end

    [path,action] = a_star(grid, init, goal, cost, heuristic);
    
    path_and_map = grid;
    for i = 1:size(path,1)
       path_and_map(path(i,1), path(i,2)) = 10 ;
    end
    
end

function [path, action] = a_star(grid, init, goal, cost, heuristic)
    directions = [-1 0; 0 -1; 1 0; 0 1];
    
    closed = zeros(size(grid,1), size(grid,2));
    closed(init(1), init(2)) = 1;
    action = zeros(size(grid,1), size(grid,2));
    
    x = init(1);
    y = init(2);
    g = 0;
    f = g + heuristic(x,y);
    cell = [f g x y];
   
    
    found = false;
    resign = false;
    
    while found == false && resign == false
        cell = sortrows(cell, 'descend');
        
        next = cell(size(cell,1),:);
        cell(size(cell,1),:) = [];
        x = next(3);
        y = next(4);
        g = next(2);
        
        
        if x == goal(1) && y == goal(2)
            found = true;
            
        else
            for i = 1:size(directions,1)
                x2 = x + directions(i,1);
                y2 = y + directions(i,2);
                if x2 >= 1 && x2 <= size(grid,1) && y2 >= 1 && y2 <= size(grid,2)
                   if closed(x2,y2) == 0 && grid(x2,y2) == 0                       
                       g2 = g + cost;
                       f2 = g2 + heuristic(x2,y2);
                       cell(size(cell,1)+1,:) = [f2 g2 x2 y2];
                       closed(x2,y2) = 1;
                       action(x2,y2) = i;
                   end
                end                        
            end  
        end
        
    end
    
    invpath = [];
    x = goal(1);
    y = goal(2);
    invpath(size(invpath,1)+1,:) = [x y];
    while x ~= init(1) || y ~= init(2)
        act = directions(action(x,y),:);
        x2 = x - act(1);
        y2 = y - act(2);
        x = x2;
        y = y2;
        invpath(size(invpath,1)+1,:) = [x y];
    end
    
    path = [];
    for i = 1:size(invpath,1)
        path(size(path,1)+1, :) = invpath(size(invpath,1)-i+1,:);
    end
    
end

% lidardata function discussed at the very beginning of the code:
% For the discrete map for the data of HW3, it takes in the robot position 
%and the map, and outputs (and plots) the lidar data expected as if there 
%is a lidar on the robot. The maximum range that the lidar can see is 10 
%cells. The lidar is not able to see anything beyond an obstacle, if there 
%is an obstacle at a distance less than this maximum range.
function lidardata(robotposition,map) 

trans = map';

if trans(robotposition(1),robotposition(2)) == 1
for i=1:10
    if trans(robotposition(1)+i,robotposition(2)) == 1
        scatter(robotposition(1)+i,robotposition(2),30,"b",'filled','s','MarkerFaceAlpha',.5)
    else
        break
    end
    
end
for i=1:10
    if trans(robotposition(1)-i,robotposition(2)) == 1
        scatter(robotposition(1)-i,robotposition(2),30,"b",'filled','s','MarkerFaceAlpha',.5)
    else
        break
    end
end
end

if trans(robotposition(1),robotposition(2)) == 1
for i=1:9
    if trans(robotposition(1),robotposition(2)+1) == 1
        scatter(robotposition(1),robotposition(2)+1,30,"b",'filled','s','MarkerFaceAlpha',.5)
        if trans(robotposition(1)+i,robotposition(2)+1) == 1
            scatter(robotposition(1)+i,robotposition(2)+1,30,"b",'filled','s','MarkerFaceAlpha',.5)
        else
            break
        end
    end
end

for i=1:9
    if trans(robotposition(1)-i,robotposition(2)+1) == 1
        scatter(robotposition(1)-i,robotposition(2)+1,30,"b",'filled','s','MarkerFaceAlpha',.5)
    else
        break
    end
end
end


if trans(robotposition(1),robotposition(2)) == 1
for i=1:9
    if trans(robotposition(1),robotposition(2)-1) == 1
        scatter(robotposition(1),robotposition(2)-1,30,"b",'filled','s','MarkerFaceAlpha',.5)
        if trans(robotposition(1)+i,robotposition(2)-1) == 1
            scatter(robotposition(1)+i,robotposition(2)-1,30,"b",'filled','s','MarkerFaceAlpha',.5)
        else
            break
        end
    end
end

for i=1:9
    if trans(robotposition(1)-i,robotposition(2)-1) == 1
        scatter(robotposition(1)-i,robotposition(2)-1,30,"b",'filled','s','MarkerFaceAlpha',.5)
    else
        break
    end
end
end


if trans(robotposition(1),robotposition(2))==1 && trans(robotposition(1),robotposition(2)+1)==1
    for i=1:9
        if trans(robotposition(1),robotposition(2)+2) == 1
            scatter(robotposition(1),robotposition(2)+2,30,"b",'filled','s','MarkerFaceAlpha',.5)
            if trans(robotposition(1)+i,robotposition(2)+2) == 1
                scatter(robotposition(1)+i,robotposition(2)+2,30,"b",'filled','s','MarkerFaceAlpha',.5)
            else
                break
            end
        end
    end

    for i=1:9
        if trans(robotposition(1)-i,robotposition(2)+2) == 1
            scatter(robotposition(1)-i,robotposition(2)+2,30,"b",'filled','s','MarkerFaceAlpha',.5)
        else
            break
        end
    end
end


if trans(robotposition(1),robotposition(2))==1 && trans(robotposition(1),robotposition(2)-1)==1
    for i=1:9
        if trans(robotposition(1),robotposition(2)-2) == 1
            scatter(robotposition(1),robotposition(2)-2,30,"b",'filled','s','MarkerFaceAlpha',.5)
            if trans(robotposition(1)+i,robotposition(2)-2) == 1
                scatter(robotposition(1)+i,robotposition(2)-2,30,"b",'filled','s','MarkerFaceAlpha',.5)
            else
                break
            end
        end
    end

    for i=1:9
        if trans(robotposition(1)-i,robotposition(2)-2) == 1
            scatter(robotposition(1)-i,robotposition(2)-2,30,"b",'filled','s','MarkerFaceAlpha',.5)
        else
            break
        end
    end
end


if trans(robotposition(1),robotposition(2))==1 && trans(robotposition(1),robotposition(2)+1)==1 && ...
        trans(robotposition(1),robotposition(2)+2)==1
    for i=1:9
        if trans(robotposition(1),robotposition(2)+3) == 1
            scatter(robotposition(1),robotposition(2)+3,30,"b",'filled','s','MarkerFaceAlpha',.5)
            if trans(robotposition(1)+i,robotposition(2)+3) == 1
                scatter(robotposition(1)+i,robotposition(2)+3,30,"b",'filled','s','MarkerFaceAlpha',.5)
            else
                break
            end
        end
    end

    for i=1:9
        if trans(robotposition(1)-i,robotposition(2)+3) == 1
            scatter(robotposition(1)-i,robotposition(2)+3,30,"b",'filled','s','MarkerFaceAlpha',.5)
        else
            break
        end
    end
end


if trans(robotposition(1),robotposition(2))==1 && trans(robotposition(1),robotposition(2)-1)==1 && ...
        trans(robotposition(1),robotposition(2)-2)==1
    for i=1:9
        if trans(robotposition(1),robotposition(2)-3) == 1
            scatter(robotposition(1),robotposition(2)-3,30,"b",'filled','s','MarkerFaceAlpha',.5)
            if trans(robotposition(1)+i,robotposition(2)-3) == 1
                scatter(robotposition(1)+i,robotposition(2)-3,30,"b",'filled','s','MarkerFaceAlpha',.5)
            else
                break
            end
        end
    end

    for i=1:9
        if trans(robotposition(1)-i,robotposition(2)-3) == 1
            scatter(robotposition(1)-i,robotposition(2)-3,30,"b",'filled','s','MarkerFaceAlpha',.5)
        else
            break
        end
    end
end


if trans(robotposition(1),robotposition(2))==1 && trans(robotposition(1),robotposition(2)+1)==1 && ...
        trans(robotposition(1),robotposition(2)+2)==1 && trans(robotposition(1),robotposition(2)+3)==1
    for i=1:9
        if trans(robotposition(1),robotposition(2)+4) == 1
            scatter(robotposition(1),robotposition(2)+4,30,"b",'filled','s','MarkerFaceAlpha',.5)
            if trans(robotposition(1)+i,robotposition(2)+4) == 1
                scatter(robotposition(1)+i,robotposition(2)+4,30,"b",'filled','s','MarkerFaceAlpha',.5)
            else
                break
            end
        end
    end

    for i=1:9
        if trans(robotposition(1)-i,robotposition(2)+4) == 1
            scatter(robotposition(1)-i,robotposition(2)+4,30,"b",'filled','s','MarkerFaceAlpha',.5)
        else
            break
        end
    end
end

if trans(robotposition(1),robotposition(2))==1 && trans(robotposition(1),robotposition(2)-1)==1 && ...
        trans(robotposition(1),robotposition(2)-2)==1 && trans(robotposition(1),robotposition(2)-3)==1
    for i=1:9
        if trans(robotposition(1),robotposition(2)-4) == 1
            scatter(robotposition(1),robotposition(2)-4,30,"b",'filled','s','MarkerFaceAlpha',.5)
            if trans(robotposition(1)+i,robotposition(2)-4) == 1
                scatter(robotposition(1)+i,robotposition(2)-4,30,"b",'filled','s','MarkerFaceAlpha',.5)
            else
                break
            end
        end
    end

    for i=1:9
        if trans(robotposition(1)-i,robotposition(2)-4) == 1
            scatter(robotposition(1)-i,robotposition(2)-4,30,"b",'filled','s','MarkerFaceAlpha',.5)
        else
            break
        end
    end
end


if trans(robotposition(1),robotposition(2))==1 && trans(robotposition(1),robotposition(2)+1)==1 && ...
        trans(robotposition(1),robotposition(2)+2)==1 && trans(robotposition(1),robotposition(2)+3)==1 && ...
        trans(robotposition(1),robotposition(2)+4)==1
    for i=1:8
        if trans(robotposition(1),robotposition(2)+5) == 1
            scatter(robotposition(1),robotposition(2)+5,30,"b",'filled','s','MarkerFaceAlpha',.5)
            if trans(robotposition(1)+i,robotposition(2)+5) == 1
                scatter(robotposition(1)+i,robotposition(2)+5,30,"b",'filled','s','MarkerFaceAlpha',.5)
            else
                break
            end
        end
    end

    for i=1:8
        if trans(robotposition(1)-i,robotposition(2)+5) == 1
            scatter(robotposition(1)-i,robotposition(2)+5,30,"b",'filled','s','MarkerFaceAlpha',.5)
        else
            break
        end
    end
end

if trans(robotposition(1),robotposition(2))==1 && trans(robotposition(1),robotposition(2)-1)==1 && ...
        trans(robotposition(1),robotposition(2)-2)==1 && trans(robotposition(1),robotposition(2)-3)==1 && ...
        trans(robotposition(1),robotposition(2)-4)==1
    for i=1:8
        if trans(robotposition(1),robotposition(2)-5) == 1
            scatter(robotposition(1),robotposition(2)-5,30,"b",'filled','s','MarkerFaceAlpha',.5)
            if trans(robotposition(1)+i,robotposition(2)-5) == 1
                scatter(robotposition(1)+i,robotposition(2)-5,30,"b",'filled','s','MarkerFaceAlpha',.5)
            else
                break
            end
        end
    end

    for i=1:8
        if trans(robotposition(1)-i,robotposition(2)-5) == 1
            scatter(robotposition(1)-i,robotposition(2)-5,30,"b",'filled','s','MarkerFaceAlpha',.5)
        else
            break
        end
    end
end


if trans(robotposition(1),robotposition(2))==1 && trans(robotposition(1),robotposition(2)+1)==1 && ...
        trans(robotposition(1),robotposition(2)+2)==1 && trans(robotposition(1),robotposition(2)+3)==1 && ...
        trans(robotposition(1),robotposition(2)+4)==1 && trans(robotposition(1),robotposition(2)+5)==1 
    for i=1:8
        if trans(robotposition(1),robotposition(2)+6) == 1
            scatter(robotposition(1),robotposition(2)+6,30,"b",'filled','s','MarkerFaceAlpha',.5)
            if trans(robotposition(1)+i,robotposition(2)+6) == 1
                scatter(robotposition(1)+i,robotposition(2)+6,30,"b",'filled','s','MarkerFaceAlpha',.5)
            else
                break
            end
        end
    end

    for i=1:8
        if trans(robotposition(1)-i,robotposition(2)+6) == 1
            scatter(robotposition(1)-i,robotposition(2)+6,30,"b",'filled','s','MarkerFaceAlpha',.5)
        else
            break
        end
    end
end

if trans(robotposition(1),robotposition(2))==1 && trans(robotposition(1),robotposition(2)-1)==1 && ...
        trans(robotposition(1),robotposition(2)-2)==1 && trans(robotposition(1),robotposition(2)-3)==1 && ...
        trans(robotposition(1),robotposition(2)-4)==1 && trans(robotposition(1),robotposition(2)-5)==1
    for i=1:8
        if trans(robotposition(1),robotposition(2)-6) == 1
            scatter(robotposition(1),robotposition(2)-6,30,"b",'filled','s','MarkerFaceAlpha',.5)
            if trans(robotposition(1)+i,robotposition(2)-6) == 1
                scatter(robotposition(1)+i,robotposition(2)-6,30,"b",'filled','s','MarkerFaceAlpha',.5)
            else
                break
            end
        end
    end

    for i=1:8
        if trans(robotposition(1)-i,robotposition(2)-6) == 1
            scatter(robotposition(1)-i,robotposition(2)-6,30,"b",'filled','s','MarkerFaceAlpha',.5)
        else
            break
        end
    end
end


if trans(robotposition(1),robotposition(2))==1 && trans(robotposition(1),robotposition(2)+1)==1 && ...
        trans(robotposition(1),robotposition(2)+2)==1 && trans(robotposition(1),robotposition(2)+3)==1 && ...
        trans(robotposition(1),robotposition(2)+4)==1 && trans(robotposition(1),robotposition(2)+5)==1 && ...
        trans(robotposition(1),robotposition(2)+6)==1 
    for i=1:7
        if trans(robotposition(1),robotposition(2)+7) == 1
            scatter(robotposition(1),robotposition(2)+7,30,"b",'filled','s','MarkerFaceAlpha',.5)
            if trans(robotposition(1)+i,robotposition(2)+7) == 1
                scatter(robotposition(1)+i,robotposition(2)+7,30,"b",'filled','s','MarkerFaceAlpha',.5)
            else
                break
            end
        end
    end

    for i=1:7
        if trans(robotposition(1)-i,robotposition(2)+7) == 1
            scatter(robotposition(1)-i,robotposition(2)+7,30,"b",'filled','s','MarkerFaceAlpha',.5)
        else
            break
        end
    end
end

if trans(robotposition(1),robotposition(2))==1 && trans(robotposition(1),robotposition(2)-1)==1 && ...
        trans(robotposition(1),robotposition(2)-2)==1 && trans(robotposition(1),robotposition(2)-3)==1 && ...
        trans(robotposition(1),robotposition(2)-4)==1 && trans(robotposition(1),robotposition(2)-5)==1 && ...
        trans(robotposition(1),robotposition(2)-6)==1
    for i=1:7
        if trans(robotposition(1),robotposition(2)-7) == 1
            scatter(robotposition(1),robotposition(2)-7,30,"b",'filled','s','MarkerFaceAlpha',.5)
            if trans(robotposition(1)+i,robotposition(2)-7) == 1
                scatter(robotposition(1)+i,robotposition(2)-7,30,"b",'filled','s','MarkerFaceAlpha',.5)
            else
                break
            end
        end
    end

    for i=1:7
        if trans(robotposition(1)-i,robotposition(2)-7) == 1
            scatter(robotposition(1)-i,robotposition(2)-7,30,"b",'filled','s','MarkerFaceAlpha',.5)
        else
            break
        end
    end
end


if trans(robotposition(1),robotposition(2))==1 && trans(robotposition(1),robotposition(2)+1)==1 && ...
        trans(robotposition(1),robotposition(2)+2)==1 && trans(robotposition(1),robotposition(2)+3)==1 && ...
        trans(robotposition(1),robotposition(2)+4)==1 && trans(robotposition(1),robotposition(2)+5)==1 && ...
        trans(robotposition(1),robotposition(2)+6)==1 && trans(robotposition(1),robotposition(2)+7)==1 
    for i=1:6
        if trans(robotposition(1),robotposition(2)+8) == 1
            scatter(robotposition(1),robotposition(2)+8,30,"b",'filled','s','MarkerFaceAlpha',.5)
            if trans(robotposition(1)+i,robotposition(2)+8) == 1
                scatter(robotposition(1)+i,robotposition(2)+8,30,"b",'filled','s','MarkerFaceAlpha',.5)
            else
                break
            end
        end
    end

    for i=1:6
        if trans(robotposition(1)-i,robotposition(2)+8) == 1
            scatter(robotposition(1)-i,robotposition(2)+8,30,"b",'filled','s','MarkerFaceAlpha',.5)
        else
            break
        end
    end
end

if trans(robotposition(1),robotposition(2))==1 && trans(robotposition(1),robotposition(2)-1)==1 && ...
        trans(robotposition(1),robotposition(2)-2)==1 && trans(robotposition(1),robotposition(2)-3)==1 && ...
        trans(robotposition(1),robotposition(2)-4)==1 && trans(robotposition(1),robotposition(2)-5)==1 && ...
        trans(robotposition(1),robotposition(2)-6)==1 && trans(robotposition(1),robotposition(2)-7)==1 
    for i=1:6
        if trans(robotposition(1),robotposition(2)-8) == 1
            scatter(robotposition(1),robotposition(2)-8,30,"b",'filled','s','MarkerFaceAlpha',.5)
            if trans(robotposition(1)+i,robotposition(2)-8) == 1
                scatter(robotposition(1)+i,robotposition(2)-8,30,"b",'filled','s','MarkerFaceAlpha',.5)
            else
                break
            end
        end
    end

    for i=1:6
        if trans(robotposition(1)-i,robotposition(2)-8) == 1
            scatter(robotposition(1)-i,robotposition(2)-8,30,"b",'filled','s','MarkerFaceAlpha',.5)
        else
            break
        end
    end
end

if trans(robotposition(1),robotposition(2))==1 && trans(robotposition(1),robotposition(2)+1)==1 && ...
        trans(robotposition(1),robotposition(2)+2)==1 && trans(robotposition(1),robotposition(2)+3)==1 && ...
        trans(robotposition(1),robotposition(2)+4)==1 && trans(robotposition(1),robotposition(2)+5)==1 && ...
        trans(robotposition(1),robotposition(2)+6)==1 && trans(robotposition(1),robotposition(2)+7)==1 && ...
        trans(robotposition(1),robotposition(2)+8)==1  
    for i=1:4
        if trans(robotposition(1),robotposition(2)+9) == 1
            scatter(robotposition(1),robotposition(2)+9,30,"b",'filled','s','MarkerFaceAlpha',.5)
            if trans(robotposition(1)+i,robotposition(2)+9) == 1
                scatter(robotposition(1)+i,robotposition(2)+9,30,"b",'filled','s','MarkerFaceAlpha',.5)
            else
                break
            end
        end
    end

    for i=1:4
        if trans(robotposition(1)-i,robotposition(2)+9) == 1
            scatter(robotposition(1)-i,robotposition(2)+9,30,"b",'filled','s','MarkerFaceAlpha',.5)
        else
            break
        end
    end
end

if trans(robotposition(1),robotposition(2))==1 && trans(robotposition(1),robotposition(2)-1)==1 && ...
        trans(robotposition(1),robotposition(2)-2)==1 && trans(robotposition(1),robotposition(2)-3)==1 && ...
        trans(robotposition(1),robotposition(2)-4)==1 && trans(robotposition(1),robotposition(2)-5)==1 && ...
        trans(robotposition(1),robotposition(2)-6)==1 && trans(robotposition(1),robotposition(2)-7)==1 && ...
        trans(robotposition(1),robotposition(2)-8)==1 
    for i=1:4
        if trans(robotposition(1),robotposition(2)-9) == 1
            scatter(robotposition(1),robotposition(2)-9,30,"b",'filled','s','MarkerFaceAlpha',.5)
            if trans(robotposition(1)+i,robotposition(2)-9) == 1
                scatter(robotposition(1)+i,robotposition(2)-9,30,"b",'filled','s','MarkerFaceAlpha',.5)
            else
                break
            end
        end
    end

    for i=1:4
        if trans(robotposition(1)-i,robotposition(2)-9) == 1
            scatter(robotposition(1)-i,robotposition(2)-9,30,"b",'filled','s','MarkerFaceAlpha',.5)
        else
            break
        end
    end
end


if trans(robotposition(1),robotposition(2))==1 && trans(robotposition(1),robotposition(2)+1)==1 && ...
        trans(robotposition(1),robotposition(2)+2)==1 && trans(robotposition(1),robotposition(2)+3)==1 && ...
        trans(robotposition(1),robotposition(2)+4)==1 && trans(robotposition(1),robotposition(2)+5)==1 && ...
        trans(robotposition(1),robotposition(2)+6)==1 && trans(robotposition(1),robotposition(2)+7)==1 && ...
        trans(robotposition(1),robotposition(2)+8)==1 &&  trans(robotposition(1),robotposition(2)+9)==1 && ...
        trans(robotposition(1),robotposition(2)+10)==1
    scatter(robotposition(1),robotposition(2)+10,30,"b",'filled','s','MarkerFaceAlpha',.5)
end

if trans(robotposition(1),robotposition(2))==1 && trans(robotposition(1),robotposition(2)-1)==1 && ...
        trans(robotposition(1),robotposition(2)-2)==1 && trans(robotposition(1),robotposition(2)-3)==1 && ...
        trans(robotposition(1),robotposition(2)-4)==1 && trans(robotposition(1),robotposition(2)-5)==1 && ...
        trans(robotposition(1),robotposition(2)-6)==1 && trans(robotposition(1),robotposition(2)-7)==1 && ...
        trans(robotposition(1),robotposition(2)-8)==1 &&  trans(robotposition(1),robotposition(2)-9)==1 && ...
        trans(robotposition(1),robotposition(2)-10)==1
    scatter(robotposition(1),robotposition(2)-10,30,"b",'filled','s','MarkerFaceAlpha',.5)
end

scatter(robotposition(1),robotposition(2),30,"r",'filled','s','MarkerFaceAlpha',.5)


end

% probability function discussed at the very begginning of the code
function prob = probability()
    interval = [1:100];
    idx=randperm(length(interval),1);
    prob = idx/100;
end


function env = mapp()
    env = imread('mapv3.png');
    env = env(:,:,1)/255;
    heatmap(env);
    colormap(gray)
    title('Binary image of the map')

end