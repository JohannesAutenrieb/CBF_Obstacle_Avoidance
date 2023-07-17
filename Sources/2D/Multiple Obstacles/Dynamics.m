function dxdt= Dynamics(x,u,params)

xp_dot      = params.A*x + params.B*u;
dxdt = [xp_dot];

end

