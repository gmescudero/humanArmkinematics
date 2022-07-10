% Iterative method to calibrate one rotation axis
% RETURNS:
%   - rotX: rotation vector X computed for the current iteration
%   - se: error in the estimation
%   - J: current cost calculated
%   - alpha, beta: coeficients to reconstruct omegaR from with the computed
%   rotation vectors
function [rotA,rotB,se,J,alpha,beta] = calibrateTwoRotationAxes(...
    config,...  % Configuration for the calibration
    aA,...      % Old rotation vector to be corrected
    bB,...      % Old rotation vector to be corrected
    rotBA,...   % Conversion from B to A
    omegaR)     % Relative angular velocity
persistent dJk_phi_array J_array
    alternative = 0;
    conversion  = rotx(90);
% Extraer inputs
    M        = config.gradientWindow;
    lambda   = config.gradientStepSize;
    vRotA    = aA;
    vRotB    = bB;
    vRotB_A  = rotate_vector_by_quaternion(bB',rotBA)';
    rotMatBA = quat2rotm(rotBA);
    if isempty(dJk_phi_array) || isempty(J_array)
        dJk_phi_array = zeros(4,M);        
        J_array       = zeros(1,M);
    end

% Seleccionar el tipo de coordenadas esfericas a utilizar
    thetaA = atan2(sqrt(1-vRotA(3)^2),vRotA(3));
    thetaB = atan2(sqrt(1-vRotB(3)^2),vRotB(3));
    if thetaA < pi/4 || thetaA > 3*pi/4 || thetaB < pi/4 || thetaB > 3*pi/4
         alternative = 1;
         vRotA    = conversion*vRotA;
         vRotB    = conversion*vRotB;
         omegaR   = conversion*omegaR;
         thetaA   = atan2(sqrt(1-vRotA(3)^2),vRotA(3));
         thetaB   = atan2(sqrt(1-vRotB(3)^2),vRotB(3));
         if thetaA < pi/4 || thetaA > 3*pi/4 || thetaB < pi/4 || thetaB > 3*pi/4
            fprintf('WARNING: ')
            fprintf("theta: [%f, %f]\t",thetaA,thetaB);
            fprintf("aA: [%f, %f, %f]\t",aA(1),aA(2),aA(3));
            fprintf("bB: [%f, %f, %f]\n",bB(1),bB(2),bB(3));
            lambda = lambda/10;
         end
    end
    rhoA = atan2(vRotA(2),vRotA(1));
    rhoB = atan2(vRotB(2),vRotB(1));
    
    phi = [thetaA;rhoA;thetaB;rhoB];
% Calcular alpha y beta
    Mk = [vRotA, vRotB_A];
    aux = (Mk'*Mk)*Mk'*omegaR;
    alpha = aux(1);
    beta  = aux(2);
% Calcular error
    omegaRnorm = norm(omegaR);
    omegaR2    = omegaR'*omegaR;
    if (omegaRnorm) < 3e-1
        % No movement
        disp('no movement')
        e = zeros(3,1);
    else
        e = omegaR - (alpha*vRotA + beta*vRotB_A);
    end
% Calcular coste
    Jk      = (e'*e)/omegaR2;
    J_array = [Jk, J_array(1:end,2:end-1)];
    J       = 1/M*sum(J_array,2);
% Calcular parciales de cada angulo
    aA_thetaA = [ cos(thetaA)*cos(rhoA); cos(thetaA)*sin(rhoA); -sin(thetaA)];
    aA_rhoA   = [-sin(thetaA)*sin(rhoA); sin(thetaA)*cos(rhoA); 0];
    bB_thetaB = [ cos(thetaB)*cos(rhoB); cos(thetaB)*sin(rhoB); -sin(thetaB)];
    bB_rhoB   = [-sin(thetaB)*sin(rhoB); sin(thetaB)*cos(rhoB); 0];
% Calcular parciales del indice de coste
    dJk_thetaA = (alpha*e'*aA_thetaA)/(omegaR2);
    dJk_rhoA   = (alpha*e'*aA_rhoA  )/(omegaR2);
    dJk_thetaB = (beta*e'*rotMatBA*bB_thetaB)/(omegaR2);
    dJk_rhoB   = (beta*e'*rotMatBA*bB_rhoB  )/(omegaR2);
    
    dJk_phi_k     = [dJk_thetaA; dJk_rhoA; dJk_thetaB; dJk_rhoB];
    dJk_phi_array = [dJk_phi_k, dJk_phi_array(:,2:end-1)];
    dJk_phi       = -(2/M)*sum(dJk_phi_array,2);
% Descenso del gradiente
    phi = phi - lambda*dJk_phi;
% Extraer vectores de rotación
    thetaA = phi(1);
    rhoA   = phi(2);
    thetaB = phi(3);
    rhoB   = phi(4);
    vRotA = [sin(thetaA)*cos(rhoA); sin(thetaA)*sin(rhoA); cos(thetaA)];
    vRotA = vRotA/norm(vRotA);
    vRotB = [sin(thetaB)*cos(rhoB); sin(thetaB)*sin(rhoB); cos(thetaB)];
    vRotB = vRotB/norm(vRotB);

    if alternative >= 1
        vRotA = conversion\vRotA;
        vRotB = conversion\vRotB;
    end
    
% Salidas
    se = sqrt(e'*e);
    rotA = vRotA;
    rotB = vRotB;

%     rotA = [0;0;1];
%     rotB = [1;0;0];

end

