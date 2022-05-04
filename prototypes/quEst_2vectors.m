function [q_opt, unoptimal] = quEst_2vectors(...
    r1,...
    r2,...
    b1,...
    b2,...
    varargin)

if nargin > 4
    recursion = varargin{1};
else
    recursion = false;
end
unoptimal = false;
r = [r1,r2];
b = [b1,b2];
% Weights
a = [1,1]; a = a/norm(a);
% B and z
z = zeros(3,1);
B = zeros(3,3);
for i = 1:2
    B = B + a(i)*b(:,i)*r(:,i)';
    z = z + a(i)*cross(b(:,i),r(:,i));
end
% Lambda max
aux     = dot(r1,r2)*dot(b1,b2) + norm(cross(r1,r2))*norm(cross(b1,b2));
lambda  = sqrt(a(1)^2 + 2*a(1)*a(2)*aux + a(2)^2);
% gamma and X
P       = (lambda + trace(B))*eye(3) - B - B';
gamma   = det(P);
if gamma < 1e-9
    unoptimal = true;
end
if ~recursion && unoptimal
    disp('rotation angle is 180, perfoming sequential rotations method')
    conv = rotx(180);
    r1_r = conv*r1;
    r2_r = conv*r2;
    [q,unoptimal] = quEst_2vectors(r1_r,r2_r,b1,b2,true);
    if ~unoptimal
        q_opt = [q(2);-q(1);-q(3);q(4)];
        return
    else
        conv = roty(180);
        r1_r = conv*r1;
        r2_r = conv*r2;
        [q,unoptimal] = quEst_2vectors(r1_r,r2_r,b1,b2,true);
        if ~unoptimal
            q_opt = [q(3);-q(2);-q(1);q(4)];
            return
        else
            conv = rotz(180);
            r1_r = conv*r1;
            r2_r = conv*r2;
            [q,unoptimal] = quEst_2vectors(r1_r,r2_r,b1,b2,true);
            q_opt = [q(4);-q(3);-q(2);-q(1)];
            return
        end
    end
end
x       = (adjoint(P))*z;
% Quaternion build
q_opt = (1/sqrt(gamma^2 + norm(x)^2))*[gamma;x];
q_opt = q_opt/norm(q_opt);
end

