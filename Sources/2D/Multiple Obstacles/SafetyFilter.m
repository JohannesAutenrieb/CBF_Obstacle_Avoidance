function [u,h]= SafetyFilter(x,u_des, params)
%ADAPTIVECRUISECONTROLLER CLF-CBF QP-based adaptive cruise conroller
%implementation

h(1)=(x(1)-params.x_obs(1))^2 +(x(2)-params.x_obs(2))^2 - params.r_obs^2;
h(2)=(x(1)-params.x_obs2(1))^2 +(x(2)-params.x_obs2(2))^2 - params.r_obs^2;


h_dot= (2*(x(1)-params.x_obs(1))*x(3)+2*(x(2)-params.x_obs(2))*x(4) + params.gamma*h(1));
h_dot2= (2*(x(1)-params.x_obs2(1))*x(3)+2*(x(2)-params.x_obs2(2))*x(4) + params.gamma*h(2));

% lie derivitavies of control barrier function
Lfh_x     = 2*x(3)^2 + 2*x(4)^2 +params.gamma2*h_dot;
Lgh_x     = -2*[((x(1)-params.x_obs(1))) ((x(2)-params.x_obs(2)))];

Lfh_x2     = 2*x(3)^2 + 2*x(4)^2 +params.gamma2*h_dot2;
Lgh_x2     = -2*[((x(1)-params.x_obs2(1))) ((x(2)-params.x_obs2(2)))];


A       = [Lgh_x;Lgh_x2];      
b       = [Lfh_x;Lfh_x2];

% Fmincon
u_d = [u_des];
fun = @(u) (u - u_d)'*[1, 0; 0, 1]*(u - u_d);

options = optimoptions('fmincon','Display','off','Algorithm','sqp');   %"active-set"
u_opt = fmincon(fun,u_d,A,b,[],[],[],[],[],options);

% Return variables
u = u_opt;
    
end