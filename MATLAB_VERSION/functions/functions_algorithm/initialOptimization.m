function [xi_01, T_01] = initialOptimization(R_01, data)
n_valid_data = data.n_valid_data;
n_0 = data.l0.n_0;
n_1 = data.l1.n_1;

R_10 = R_01^-1;
xi_10 = se3Log([R_10 [0 0 0]';0 0 0 1]);

n_1_warp = cell(n_valid_data,1);
J = zeros(4*n_valid_data,6);
r = zeros(4*n_valid_data,1);

iter=1;
cost_save=[];
huber_thr = 0.006;

while 1
    % rotational matrix
    xi_save(:,iter) = xi_10;
    
    T_10 = se3Exp(xi_10);
    
    % rotated normal vector
    for i = 1:n_valid_data
        n_1_warp{i,1} = T_10'*n_1{i};
    end
    
    % Calculate Jacobian (don't forget minus(-) )
    for i = 1:n_valid_data
        J(4*(i-1)+1,:) = [0, 0, 0, n_1{i}(3)*T_10(2,1)-n_1{i}(2)*T_10(3,1), ...
            n_1{i}(1)*T_10(3,1)-n_1{i}(3)*T_10(1,1), n_1{i}(2)*T_10(1,1)-n_1{i}(1)*T_10(2,1)];
        J(4*(i-1)+2,:) = [0, 0, 0, n_1{i}(3)*T_10(2,2)-n_1{i}(2)*T_10(3,2), ...
            n_1{i}(1)*T_10(3,2)-n_1{i}(3)*T_10(1,2), n_1{i}(2)*T_10(1,2)-n_1{i}(1)*T_10(2,2)];
        J(4*(i-1)+3,:) = [0, 0, 0, n_1{i}(3)*T_10(2,3)-n_1{i}(2)*T_10(3,3), ...
            n_1{i}(1)*T_10(3,3)-n_1{i}(3)*T_10(1,3), n_1{i}(2)*T_10(1,3)-n_1{i}(1)*T_10(2,3)];
        J(4*(i-1)+4,:) = [n_1{i}(1), n_1{i}(2), n_1{i}(3), n_1{i}(3)*T_10(2,4)-n_1{i}(2)*T_10(3,4), ...
            n_1{i}(1)*T_10(3,4)-n_1{i}(3)*T_10(1,4), n_1{i}(2)*T_10(1,4)-n_1{i}(1)*T_10(2,4)];
    end
    
    % Calculate residua l
    for i = 1:n_valid_data
        r(4*(i-1)+1) = n_1_warp{i}(1) - n_0{i}(1);
        r(4*(i-1)+2) = n_1_warp{i}(2) - n_0{i}(2);
        r(4*(i-1)+3) = n_1_warp{i}(3) - n_0{i}(3);
        r(4*(i-1)+4) = n_1_warp{i}(4) - n_0{i}(4);
    end

    if(iter>1)
        W = ones(size(r));
        over_ind = abs(r) > huber_thr;
        W(over_ind) = huber_thr ./ abs(r(over_ind));
        JW = bsxfun(@times,J,W);
        H  = JW.'*J;
        delta_xi = -(H+(1e-5)*eye(size(H)))^-1*JW'*r;
    else
        W = ones(size(r));
        H  = J.'*J;
        delta_xi = -(H+(1e-5)*eye(size(H)))^-1*J'*r;
    end
    expm_xi = se3Exp(delta_xi)*se3Exp(xi_10);
    xi_10      = se3Log(expm_xi);

    residual_curr = norm(W.*r);
    cost_save = [cost_save, residual_curr];
    
    if(norm(delta_xi) <= 1e-7)
        figure();
        plot(xi_save(1,:)); hold on;
        plot(xi_save(2,:));
        plot(xi_save(3,:));
        plot(xi_save(4,:));
        plot(xi_save(5,:));
        plot(xi_save(6,:));
        xlabel('iteration');
        xlim([1,size(xi_save,2)]);
        title('se(3) optimization (initialization)');
        legend('v1','v2','v3','w1','w2','w3');
        break;
    end
    
    iter=iter+1;
end
T_01 = T_10^-1;
xi_01 = se3Log(T_01);
end