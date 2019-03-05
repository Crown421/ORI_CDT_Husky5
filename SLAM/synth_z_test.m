
p1 = [0; 1]; p2 = [1; 0]; p3 = [-1; 0]; p4 = [0; -1];

ground_truth = [p1, p2, p3, p4];

x_rob = [0; 0; 0];
z_raw = sysnth_z_raw(ground_truth, x_rob);

[x_recvd, y_recvd] = pol2cart(z_raw(2, :), z_raw(1, :));

ab_rec = [y_recvd; x_recvd];
