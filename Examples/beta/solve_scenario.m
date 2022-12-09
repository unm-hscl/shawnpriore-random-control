conf = 1e-8;
N_samples = ceil(2/safety_target * (log(1/conf)+size(Bd, 2) * time_horizon));

Bs = zeros(size(Bd,1) * time_horizon, size(Bd, 2) * time_horizon, N_samples);
rng(3);
for samp = 1:N_samples
    for i = 0:(time_horizon-1)
        Bs(size(Bd, 1)*i + [1:size(Bd, 1)], size(Bd, 2)*i + [1:size(Bd, 2)], samp) = Bd * diag(betarnd(a, b, 1, 3));
    end
end
tic;
cvx_begin quiet
    variable Ex_x_scen(time_horizon*6, N_samples)
    variable U(time_horizon*3,1)

    minimize (U'*U)
    subject to
        for samp = 1:N_samples
            Ex_x_scen(:,samp) == x_no_imp + Ad_control_concat * Bs(:,:,samp) * U;
            G * Ex_x_scen(:,samp) <= h; 
        end
        input_space_A * U <= input_space_b;

cvx_end
toc
Ex_x_scen = [x_0; x_no_imp + ex_beta * Bd_concat * U];
[p_scen, traj_scen] = verify(1e5, x_0, U, Ad, Bd, G, h, time_horizon, a, b);