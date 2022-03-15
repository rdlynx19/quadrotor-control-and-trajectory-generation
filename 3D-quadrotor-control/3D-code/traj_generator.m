function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

persistent coffx coffy coffz traj_time d0 waypoints0
if nargin > 2
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
    waypoints0 = waypoints;
    coffx = getCoff(waypoints0(1,:)');
    coffy = getCoff(waypoints0(2,:)');
    coffz = getCoff(waypoints0(3,:)');
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    
   
else
    if(t >= traj_time(end))
        t = traj_time(end)-0.0001;
    end
    t_index = find(traj_time > t,1) - 1;
    t_index = max(t_index,1);
    scale = (t-traj_time(t_index))/d0(t_index);

%      if(t_index > 1)
%         t = t - traj_time(t_index-1);
%     end
    if(t == 0)
         desired_state.pos = waypoints0(:,1);
         desired_state.vel = zeros(3,1);
         desired_state.acc = zeros(3,1);
         desired_state.yaw = 0;
         desired_state.yawdot = 0;
    else
        t0 = polyT(8,0,scale)';
        t1 = polyT(8,1,scale)';
        t2 = polyT(8,2,scale)';
        
        index = [(t_index-1)*8+1:t_index*8];
        
        desired_state.pos = [coffx(index)'*t0;coffy(index)'*t0;coffz(index)'*t0];
        desired_state.vel = [coffx(index)'*t1*(1/d0(t_index));coffy(index)'*t1*(1/d0(t_index));coffz(index)'*t1*(1/d0(t_index))];
        desired_state.acc = [coffx(index)'*t2*(1/d0(t_index)^2);coffy(index)'*t2*(1/d0(t_index)^2);coffz(index)'*t2*(1/d0(t_index)^2)];
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    end
 
end
%


%% Fill in your code here
function [T] = polyT(n,k,t)
%n is the polynom number of coefficients, k is the requested derivative and
%t is the actual value of t (this can be anything, not just 0 or 1).
T = zeros(n,1);
D = zeros(n,1);
%Init:
for i=1:n
D(i) = i-1;
T(i) = 1;
end
%Derivative:
for j=1:k
    for i=1:n
        T(i) = T(i) * D(i);

        if D(i) > 0
            D(i) = D(i) - 1;
        end
    end
end
%put t value
for i=1:n
T(i) = T(i) * t^D(i);
end
T = T';
end

function [coff, A, b] = getCoff(waypoints)
n = size(waypoints,1)-1; % number of segments P1..n
A = zeros(8*n, 8*n);
b = zeros(1,8*n);

% --------------------------------------------

% YOUR CODE HERE 
% Fill A and b matices with values using loops

% Example for first 4 equations:

for i=1:n
    b(1,i) = waypoints(i);
    b(1,i+n) = waypoints(i+1);
end

row = 1;
%Pi(0) = Wi
for i=1:n 
    A(row,(8*(i-1))+1:8*i) = polyT(8,0,0); 
    row = row + 1;
end
% Pi(1) = W(i+1)
for i=n+1:2*n
    A(row,(8*(i-5))+1:8*(i-4)) = polyT(8,0,1);
    row = row + 1;
end
%P1(k)(0)=0 for all 1<=k<=3
    A(9,1:8) = polyT(8,1,0);
    A(10,1:8) = polyT(8,2,0);
    A(11,1:8) = polyT(8,3,0);
%Pn(k)(1)=0 for all 1<=k<=3
    A(12,25:32) = polyT(8,1,1);%check 
    A(13,25:32) = polyT(8,2,1);%check
    A(14,25:32) = polyT(8,3,1);%check
    
%Pi-1(k)(1) = Pi(k)(0) for all i=2..n and k=1..6
    A(15,1:16) = [polyT(8,1,1) -polyT(8,1,0)];
    A(16,9:24) = [polyT(8,1,1) -polyT(8,1,0)];
    A(17,17:32) = [polyT(8,1,1) -polyT(8,1,0)];
    
    A(18,1:16) = [polyT(8,2,1) -polyT(8,2,0)];
    A(19,9:24) = [polyT(8,2,1) -polyT(8,2,0)];
    A(20,17:32) = [polyT(8,2,1) -polyT(8,2,0)];
    
    A(21,1:16) = [polyT(8,3,1) -polyT(8,3,0)];
    A(22,9:24) = [polyT(8,3,1) -polyT(8,3,0)];
    A(23,17:32) = [polyT(8,3,1) -polyT(8,3,0)];
    
    A(24,1:16) = [polyT(8,4,1) -polyT(8,4,0)];
    A(25,9:24) = [polyT(8,4,1) -polyT(8,4,0)];
    A(26,17:32) = [polyT(8,4,1) -polyT(8,4,0)];
    
    A(27,1:16) = [polyT(8,5,1) -polyT(8,5,0)];
    A(28,9:24) = [polyT(8,5,1) -polyT(8,5,0)];
    A(29,17:32) = [polyT(8,5,1) -polyT(8,5,0)];
    
    A(30,1:16) = [polyT(8,6,1) -polyT(8,6,0)];
    A(31,9:24) = [polyT(8,6,1) -polyT(8,6,0)];
    A(32,17:32) = [polyT(8,6,1) -polyT(8,6,0)];
% --------------------------------------------

coff = A\b';
end
% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;
end

