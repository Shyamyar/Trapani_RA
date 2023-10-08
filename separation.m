function sep = separation(p_A,p_B)
sep.del_xy = p_B-p_A;
sep.D_AB = sqrt(dot(sep.del_xy,sep.del_xy));