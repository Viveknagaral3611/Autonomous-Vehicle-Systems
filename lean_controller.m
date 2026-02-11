function [u_cmd, u_out] = lean_controller(phi_des, phi_des_dot_filt, phi, phi_dot, u_prev, ctrl, act, plant, dt)
%LEAN_CONTROLLER Feedforward + PD feedback lean controller with actuator limits.
%
% Inputs:
%   phi_des           desired lean angle (rad)
%   phi_des_dot_filt  desired lean rate (rad/s) -- filtered in main
%   phi               current roll angle (rad)
%   phi_dot           current roll rate (rad/s)
%   u_prev            previous applied torque (N*m)
%   ctrl              struct with Kp, Kd, use_rate_limit
%   act               struct with u_max, du_max_step (optional)
%   plant             struct with kphi (for feedforward)
%   dt                timestep (s) [kept for clarity; not required if du_max_step is used]
%
% Outputs:
%   u_cmd raw requested torque (N*m)
%   u_out applied torque after saturation and (optional) rate limiting (N*m)

% Tracking errors
e  = phi_des - phi;
ed = phi_des_dot_filt - phi_dot;

% Feedforward torque to counter roll stiffness demand
% (Helps reduce steady error and avoids "fighting" kphi purely via feedback)
u_ff = plant.kphi * phi_des;

% PD feedback torque
u_fb = ctrl.Kp * e + ctrl.Kd * ed;

% Total commanded torque (before limits)
u_cmd = u_ff + u_fb;

% Saturation
u_sat = min(max(u_cmd, -act.u_max), act.u_max);

% Optional rate limit (per step)
if isfield(ctrl,'use_rate_limit') && ctrl.use_rate_limit && isfield(act,'du_max_step')
    du = u_sat - u_prev;
    du = min(max(du, -act.du_max_step), act.du_max_step);
    u_out = u_prev + du;
else
    u_out = u_sat;
end
end
