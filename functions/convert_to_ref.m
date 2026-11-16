function sys_ref = convert_to_ref(sys,phi_bar,phi_til)
    for i=1:3
        for j=1:3
            T(:,:,i,j) = phi_bar(:,:,i,j)*phi_til(:,:,i,j);
            S(:,:,i,j) = inv(T(:,:,i,j));
            T_r(:,:,i,j) = T([1:7],:,i,j);
            S_r(:,:,i,j) = S(:,[1:7],i,j);
        end
    end
    
    
    S_r_ref = S_r(:,:,2,2);
    
    for i=1:3
        for j=1:3
            T_i_ref(:,:,i,j) = T_r(:,:,i,j)*S_r_ref;
            T_ref_i(:,:,i,j) = inv(T_i_ref(:,:,i,j));
            A_bar_r(:,:,i,j) = sys(:,:,i,j).A;
            B_bar_r(:,:,i,j) = sys(:,:,i,j).B;
            C_bar_r(:,:,i,j) = sys(:,:,i,j).C;
            D_bar(:,:,i,j) = sys(:,:,i,j).D;
    
            A_ref(:,:,i,j) = T_ref_i(:,:,i,j)*A_bar_r(:,:,i,j)*T_i_ref(:,:,i,j);
            B_ref(:,:,i,j) = T_ref_i(:,:,i,j)*B_bar_r(:,:,i,j);
            C_ref(:,:,i,j) = C_bar_r(:,:,i,j)*T_i_ref(:,:,i,j);
    
            sys_ref(:,:,i,j) = ss(A_ref(:,:,i,j),B_ref(:,:,i,j),C_ref(:,:,i,j),D_bar(:,:,i,j));
        end
    end


end