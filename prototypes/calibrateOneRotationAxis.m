% Iterative method to calibrate one rotation axis
% RETURNS:
%   - rot: rotation vector computed for the current iteration
%   - se: error in the estimation
%   - J: current cost calculated
function [rot,se,J] = calibrateOneRotationAxis(...
    config,...  % Configuration for the calibration
    aA,...      % Old rotation vector to be corrected
    omegaR)     % Relative angular velocity
persistent dJk_phi_array J_array
    alternative = 0;
    conversion  = roty(90);
% Extraer inputs
    M      = config.gradientWindow;
    lambda = config.gradientStepSize;
    vRot   = aA;
    if isempty(dJk_phi_array) || isempty(J_array) 
        dJk_phi_array   = zeros(2,M);
        J_array         = zeros(1,M);
    end
% Seleccionar el tipo de coordenadas esfericas a utilizar
    thetaA = atan2(sqrt(1-vRot(3)^2),vRot(3));
    if thetaA < pi/4 || thetaA > 3*pi/4
         alternative = 1;
         vRot   = conversion*vRot;
         omegaR = conversion*omegaR;
         thetaA = atan2(sqrt(1-vRot(3)^2),vRot(3));
    end
    rhoA = atan2(vRot(2),vRot(1));
% Calcular alpha
    alpha = (vRot'*vRot)*vRot'*omegaR;
% Calcular error
    omegaRnorm = norm(omegaR);
    omegaR2    = omegaR'*omegaR;
    if (omegaRnorm) < 3e-1
        disp('no movement')
        e = zeros(3,1);
    else
        e = (alpha*vRot - omegaR);
    end
% Calcular parciales de cada angulo
    aA_thetaA = [ cos(thetaA)*cos(rhoA); cos(thetaA)*sin(rhoA); -sin(thetaA)];
    aA_rhoA   = [-sin(thetaA)*sin(rhoA); sin(thetaA)*cos(rhoA); 0];
    
    phi = [thetaA;rhoA];
% Calcular coste
    Jk      = (e'*e)/omegaR2;
    J_array = [Jk, J_array(1:end,2:end-1)];
    J       = 1/M*sum(J_array,2);
% Calcular parciales del indice de coste
    
    dJk_thetaA = (alpha*e'*aA_thetaA)/(omegaR2);
    dJk_rhoA   = (alpha*e'*aA_rhoA  )/(omegaR2);
    
    dJk_phi_k     = [dJk_thetaA; dJk_rhoA];
    dJk_phi_array = [dJk_phi_k, dJk_phi_array(1:end,2:end-1)];
    dJk_phi       = 1/M*sum(dJk_phi_array,2);
% Descenso del gradiente
    phi = phi - lambda*dJk_phi;
% Extraer vectores de rotación
    thetaA = phi(1);
    rhoA   = phi(2);
    vRot = [sin(thetaA)*cos(rhoA); sin(thetaA)*sin(rhoA); cos(thetaA)];
    vRot = vRot/norm(vRot);
    if alternative >= 1
        vRot = conversion\vRot;
    end
    
% Salidas
    se = sqrt(e'*e);
    rot = vRot;
end