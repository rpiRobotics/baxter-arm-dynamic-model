function PHI = phi_gen(q,H,P)

% Generates Phi matrix for N-E recursive dynamics. 
% Inputs:
% q: joint angles
% H: joint rotation vectors
% P: joint offsets

% Outputs:
% PHI: Phi matrix of the form 
% Phi = [Phi10,Phi 11, 0 ... 0; Phi20, Phi 21, Phi 22, 0 ... 0;...], 
% where Phi_{ij} = [R_{ij} zeros_{3x3}; -R_{ij}*hat(P_{ji}) R_{ij}]
% NOTE: THIS MATRIX IS NOT SQUARE

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    n = size(H,2);
    PHI = zeros((6*n),(6*(n+1)));

    
    for k = 1:n
        PHI((k*6 - 5):(k*6),(k*6+1):(k*6 + 6)) = eye(6);
        R = eye(3);
        R_frame = eye(3);
        p = zeros(3,1);
        for ii = 1:k
           R = rot(H(:,k+1-ii),q(k+1-ii))*R;
           R_tran = R';
           p = R_frame*p + P(:,k+1 - ii);
           if ii < k
               R_frame = rot(H(:,k-ii),q(k-ii));
           else
               R_frame = eye(3);
           end
           PHI((6*k - 5):(6*k),(6*k - 6*ii + 1):(6*k - 6*ii + 6)) = ...
               [R_tran, zeros(3,3); -R_tran*hat(p), R_tran];
        end
        
    end

end