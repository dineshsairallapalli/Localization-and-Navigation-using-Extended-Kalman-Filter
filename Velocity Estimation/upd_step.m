function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%z_t Its the measurement
%covarEst and uEst are the predItcted covarItance and mean respectItvely
%uCurr and covar_curr are the updated mean and covarItance respectItvely
It = [zeros(3,3),zeros(3,3),eye(3,3),zeros(3,3),zeros(3,3)];
z = It * uEst + zeros(3,1);

Ct = It ;

R = eye(3,3)*0.1;

Kt = covarEst * Ct' /(Ct * covarEst* Ct' + R );
uCurr = uEst + Kt * (z_t - z );
covar_curr = covarEst- Kt* Ct * covarEst;

end