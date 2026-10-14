
function [pos, J] = evalRobot2D(l, theta)
    l1 = l(1);
    l2 = l(2);
    th1 = theta(1);
    th2 = theta(2);
    
    x = l1*cos(th1) + l2*cos(th1 + th2);
    y = l1*sin(th1) + l2*sin(th1 + th2);
    pos = [x; y];

    dx_dth1 = -l1*sin(th1) - l2*sin(th1 + th2);
    dx_dth2 = -l2*sin(th1 + th2);
    dy_dth1 = l1*cos(th1) + l2*cos(th1 + th2);
    dy_dth2 = l2*cos(th1 + th2);

    J = [dx_dth1, dx_dth2; dy_dth1, dy_dth2];
end

% (a) The results for fdJacob2D are accurate enough for values of h that are
% small. The larger the value of h the larger the distance between the to
% points which would result in inaccuracies. Another issue to mention is
% for values of h that are too small and could cause floating point or
% rounding errors
%
% (b) Calculating the derivatives for the jacobian matrix could be time
% consuming especially for more complex non linear functions
function J = fdJacob2D(l, theta, h)
    dpos_dtheta1 = ( evalRobot2D(l, theta+[h;0]) - evalRobot2D(l, theta-[h;0]) ) / (2*h);
    dpos_dtheta2 = ( evalRobot2D(l, theta+[0;h]) - evalRobot2D(l, theta-[0;h]) ) / (2*h);
    J = [dpos_dtheta1, dpos_dtheta2];
end


% (4. a) Broyden's method avoids calculating the jacobian matrix directly
% and instead just approximates its values. Broyden's method however has a
% slower convergence speed compared to Newtons method. This is a cost to the
% lower computation it has compard to Newtons method of calclating the
% jacobian matrix for every iteration. It is also to note that since
% Broyden's method does not calculate the exact values of the jacobian
% matrix it is not as accurate as Newtons method.
function theta = invKin2D(l, theta0, pos, n, mode)
    tol = 0.001; % Set the tolarance of difference in change to 0.001
    if mode == 0
        % Broyden’s method
        theta = theta0;
        [current_pos, J] = evalRobot2D(l, theta); % Initial position and Jacobian approximation
        d = current_pos - pos; % f(x) = d = difference = current_pos - target_pos
    
        for k = 1:n
            if norm(d) < tol % Cut the loop short if the tolarance has been reached
                break;
            end
    
            s_k = -J \ d;
            theta = theta + s_k;
    
            [new_pos, ~] = evalRobot2D(l, theta);
            d_new = new_pos - pos; % New error
            y_k = d_new - d; % Difference in errors
    
            % Update B_k using Broyden's formula
            J = J + ((y_k - J * s_k) * s_k') / (s_k' * s_k);
    
            % Update f_theta for the next iteration
            d = d_new;
        end
    elseif mode == 1
        % Newton’s method
        theta = theta0; % Initial guess
        for k = 1:n
            [current_pos, J] = evalRobot2D(l, theta);
            d = pos - current_pos;
            if norm(d) < tol % Cut the loop short if the difference has already reached the tolerance
                break;
            end

            s_k = J \ d;
            theta = theta + s_k;
        end
    end
end

% Q1 tests
l1 = [1 1]';
th1 = [pi/3 pi/3]';
[pos1, J1] = evalRobot2D(l1, th1);
% pos = [ 0.0000 1.7321]'
% J = [ -1.7321   -0.8660;  0.0000   -0.5000]

l2 = [2 2]';
th2 = [pi/2 pi/2]';
[pos2, J2] = evalRobot2D(l2, th2);
% pos = [ -2.0000 2.0000]'
% J = [-2.0000   -0.0000;   -2.0000   -2.0000]


% Q2 tests
l3 = [1 1]';
th3 = [pi/3 pi/3]';
h3 = 0.01;
J3 = fdJacob2D(l3, th3, h3);
% J = [ -1.7320   -0.8660;  -0.0000   -0.5000] 

l4 = [2 2]';
th4 = [pi/2 pi/2]';
h4 = 0.5;
J4 = fdJacob2D(l4, th4, h4);
% J = [ -1.9177   -0.0000; -1.9177   -1.9177]




%This code is available in eval2D.m
% make sure you define n and mode, the ones
% below are for illustration
n = 10;
mode = 1;

ls=[0.5,0.5]';
t=rand(2,1); %Choose some random starting point.

clf;
plotRobot2D(ls,t);
hold off;

while(1)
  desired=ginput(1)'; %Get desired position from user

  clf;
  plot(desired(1),desired(2),'*');
  hold on;
  plotRobot2D(ls,t,':');
  
  %Solve and display the position
  t=invKin2D(ls,t,desired,n,mode); 
  plotRobot2D(ls,t);
  hold off;
end



% plotRobot2D(ls,angs)
% ls: a vector containing the link lengths
% angs: a vector containing the link rotation angles
%
function plotRobot2D(ls,angs,linespec)
    if(nargin<3)
      linespec='';
    end
    holdon=ishold;
    hold on;
    axis equal;
    
    %draw invisible workspace
    ltot=sum(ls);
    plot([-ltot,ltot,ltot,-ltot],[-ltot,-ltot,ltot,ltot],'w');
    
    %draw each segment
    f=0;
    t=0;
    for j=1:length(ls)
      t=t+angs(j);
      p=f+(cos(t)+i*sin(t))*(ls(j));
      plotSeg(f,p,linespec);
      hold on;
      f=p;
    end
    
    %Restore hold
    if holdon
      hold on
    else
      hold off
    end
end

function plotSeg(from,to,linespec)
    d=to-from;
    l=norm(d);
    r1=l/10;
    r2=l/15;
    r3=l/7;
    d=d/l;
    ks=strcat('k',linespec);
    
    mycircle(l/10,real(from),imag(from),ks);
    mycircle(l/15,real(to),imag(to),ks);
    
    pts=[from+r1*i*d,to+r2*i*d];
    plot(real(pts),imag(pts),ks);
    pts=[from-r1*i*d,to-r2*i*d];
    plot(real(pts),imag(pts),ks);
    
    %The axis
    pts=[to,to+r3*d];
    plot(real(pts),imag(pts),strcat('r',linespec));
    pts=[to,to+i*r3*d];
    plot(real(pts),imag(pts),strcat('g',linespec));
end

function mycircle(rad,px,py,opts)
    t=linspace(0,2*pi,10);
    plot(rad*cos(t)+px,rad*sin(t)+py,opts);
end