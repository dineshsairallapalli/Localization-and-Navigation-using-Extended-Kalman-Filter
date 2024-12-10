function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%z_t Its the measurement
%covarEst and uEst are the predItcted covarItance and mean respectItvely
%uCurr and covar_curr are the updated mean and covarItance respectItvely

% It = zeros(6,15);
% It(1:6,1:6)= eye(6,6);

It = [eye(3),zeros(3),zeros(3),zeros(3),zeros(3);zeros(3),eye(3),zeros(3),zeros(3),zeros(3)];
z = It * uEst + zeros(6,1);

Ct = It ;


% Wt = eye(6,6);                                                                      

R = eye(6,6) * 0.1;

Kt = covarEst * Ct' /(Ct * covarEst* Ct' + R );
uCurr = uEst + Kt * (z_t - z );
covar_curr = covarEst- Kt* Ct * covarEst;

end