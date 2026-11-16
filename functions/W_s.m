function w_s = W_s(alpha,beta_0,omega_b,beta_inf)
    s = tf('s');
    w_s = alpha*(beta_inf*s + beta_0*omega_b*sqrt((beta_inf^2 - 1)/(1 - beta_0^2)))...
        /(s + omega_b*sqrt((beta_inf^2 - 1)/(1 - beta_0^2)));
end

