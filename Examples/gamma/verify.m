function p = verify(N, x_0, U, A, B, G, h, time_horizon, a, b)
    tester = zeros(N,1);
    A_concat = zeros(size(A, 1)*time_horizon, size(A, 2));
    B_concat = zeros(size(A, 1)*time_horizon, size(A, 2)*time_horizon);
    C_concat = zeros(size(B, 1)*time_horizon, size(B, 2)*time_horizon);
    for i = 0:(time_horizon-1)
        A_concat(size(A, 1)*i + [1:size(A, 1)], :) = A^(i+1);
    end
    for i = 0:(time_horizon-1)
        for j = 0:i
            B_concat(size(A, 1)*i + [1:size(A, 1)], size(A, 2)*j + [1:size(A, 2)]) = A^(i-j);
        end
    end
    rng(1);
    for  sample = 1:N
        for i = 0:(time_horizon-1)
            C_concat(size(B, 1)*i + [1:size(B, 1)], size(B, 2)*i + [1:size(B, 2)]) = [B(1:3,:) * diag(gamrnd(a, b, 1, 3)); B(4:6,:)];
        end
        x = A_concat * x_0 + B_concat * C_concat * U;
        tester(sample) = all(G*x <= h);
    end
    p = sum(tester)/N;
end