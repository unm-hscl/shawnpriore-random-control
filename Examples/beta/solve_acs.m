pow_func = @(x) 4./(9*(x.^2+1));
[pow_func_m, pow_func_c] = function_affine(1e-3, 2000, sqrt(5/3), pow_func, 5e-4, sqrt(5/3));
%%
lam = sqrt(4./(9.*alpha_i)-1) * ones(n_lin_state,1);


U_p = zeros(time_horizon*3,1);
max_iter = 100;
traj_holder = zeros((time_horizon+1)*6, max_iter);
tic;
for iter = 1:max_iter

    cvx_begin quiet
        variable Ex_x(time_horizon*6,1)
        variable U(time_horizon*3,1)

        minimize (U'*U)
        subject to
            Ex_x == x_no_imp + ex_beta * Bd_concat * U;

            input_space_A * U <= input_space_b;
            

            for i = 1:n_lin_state
                G(i,:) * Ex_x + lam(i) * norm(chol_var_Bd * diag(U) * Bd_concat' * G(i,:)') <= h(i);    
            end
    cvx_end
    
    if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
        fprintf('Failed to control: %d \n', iter);
        return
    end
    traj_holder(:,iter) = [x_0; Ex_x];
    opt_u = cvx_optval;
    
    cvx_begin quiet
        variable lam(n_lin_state,1)
        variable lam_temp(n_lin_state,1)
        variable slack(n_lin_state, 1) nonnegative

        minimize (slack'*slack)
        subject to

            for i = 1:n_lin_state
                G(i,:)*Ex_x + (lam(i)- slack(i))* norm(chol_var_Bd * diag(U) * Bd_concat' * G(i,:)')  <= h(i);    
                lam_temp(i) >= pow_func_m .* lam(i) + pow_func_c;
                lam_temp(i) >=  0;
            end
            sum(lam_temp) <= safety_target;
            lam >= sqrt(5/3);
    cvx_end

    if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
        fprintf('Failed to safety: %d \n', iter);
        return
    end
    opt_l = cvx_optval;
    
    comp = norm(U_p - U);
    U_p = U;
    fprintf('iteration: %d ', iter);
    fprintf('\t %f', opt_u);
    fprintf('\t %f \n', opt_l);
    
    if  comp <= 1e-6
        break
    end
end
toc

[p_proposed, traj_prop] = verify(1e5, x_0, U, Ad, Bd, G, h, time_horizon, a, b);