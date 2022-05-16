function [q_exp] = quaternion_exponential(Quaternion)
%   Extract the vector part of the quaternion
v = Quaternion(2:4);
%   Extract the scalar part of the quaternion
s = Quaternion(1);

nv = norm(v);
if nv < 1e-9
    q_exp = [exp(1), 0,0,0];
else
    q_exp = [exp(s)*cos(nv),  v.*sin(nv)/nv];
end
end

