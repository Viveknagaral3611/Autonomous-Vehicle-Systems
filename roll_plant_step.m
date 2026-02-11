function x = roll_plant_step(x, u, plant, dt)
% Simple roll dynamics:
% Iphi * phi_ddot + cphi * phi_dot + kphi * phi = u

phi_ddot = (u - plant.cphi * x.phi_dot - plant.kphi * x.phi) / plant.Iphi;

% Integrate (semi-implicit Euler is stable enough here)
x.phi_dot = x.phi_dot + dt * phi_ddot;
x.phi     = x.phi     + dt * x.phi_dot;
end

