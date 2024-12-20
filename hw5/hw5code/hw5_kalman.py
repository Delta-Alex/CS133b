sig_x = 1 
sig_m = 1.5

sigma_prev_sq = sig_x**2 #previous best variance at t=0 ms


# t = 10 ms * i
for i in range(1, 100000):
    if i % 16 == 0:
        #measurement update included now
        sigma_pt_sq = sigma_prev_sq + sig_x**2
        sigma_t_sq = (sigma_pt_sq * sig_m**2)/(sigma_pt_sq + sig_m**2)
        sigma_prev_sq = sigma_t_sq
        print("-" * 30)
        print("Measurement Update:")
        print("sigma_pt ^2 = {}".format(sigma_pt_sq))
        print("sigma_t ^2 = {}".format(sigma_t_sq))
    else:
        sigma_pt_sq = sigma_prev_sq + sig_x**2
        sigma_prev_sq = sigma_pt_sq
        print("sigma_pt ^2 = {}".format(sigma_pt_sq))




