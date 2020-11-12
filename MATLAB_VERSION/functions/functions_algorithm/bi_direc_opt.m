function [xi_01, T_01, data] = bi_direc_opt(xi_01, T_01, n_ring, data)

n_v_data = data.n_v_data;
X_0_     = data.l0.X_0_;
X_1_     = data.l1.X_1_;
X_0_ch_  = data.l0.X_0_ch_;
X_1_ch_  = data.l1.X_1_ch_;
plane_0_ = data.l0.plane_0_;
plane_1_ = data.l1.plane_1_;

del_rho0 = repmat([0], n_ring,1);
del_rho1 = repmat([0], n_ring,1);

X_0_p_warp = cell(n_v_data,1);
X_0_p = cell(n_v_data,1);
X_1_p_warp = cell(n_v_data,1);
X_1_p = cell(n_v_data,1);

n_pts0 = 0;
for i=1:length(X_0_)
    n_pts0 = n_pts0 + length(X_0_{i}); % # of valid 3D points
end
n_pts1 = 0;
for i=1:length(X_1_)
    n_pts1 = n_pts1 + length(X_1_{i}); % # of valid 3D points
end


J = zeros(n_pts0 + n_pts1 , 6+n_ring+n_ring);
r = zeros(n_pts0 + n_pts1 , 1);
r_total = [];

iter=1;
cost_save=[];
huber_thr = 0.005;

s_save=[];
while 1
    s_save(:,iter) = [xi_01; del_rho0; del_rho1];
    
    T_10 = T_01^-1;
    
    for i = 1:n_v_data
        [theta, psi] = theta_psi_generator(X_0_{i});
        for j=1:length(X_0_{i})
            X_0_p{i,1}(:,j) = X_0_{i}(:,j) + ...
                del_rho0(X_0_ch_{i, 1}(j,1), 1) * [cos(theta(j))*cos(psi(j)); cos(theta(j))*sin(psi(j)); sin(theta(j))];
        end
        X_0_p_warp{i,1} = T_10*[X_0_p{i}; ones(1, length(X_0_p{i}) )];
    end
    
    for ii = 1:n_v_data
        [theta, psi] = theta_psi_generator(X_1_{ii});
        for j=1:length(X_1_{ii})
            X_1_p{ii,1}(:,j) = X_1_{ii}(:,j) + ...
                del_rho1(X_1_ch_{ii, 1}(j,1), 1) * [cos(theta(j))*cos(psi(j)); cos(theta(j))*sin(psi(j)); sin(theta(j))];
        end
        X_1_p_warp{ii,1} = T_01*[X_1_p{ii}; ones(1, length(X_1_p{ii}) )];
    end
    
    % Calculate Jacobian
    start_row = 0;
    for i = 1:n_v_data
        [theta, psi] = theta_psi_generator(X_1_{i});
        for j=1:length(X_1_{i})
            M1 = [1 0 0 0 X_1_p_warp{i}(3,j) -X_1_p_warp{i}(2,j); ...
                0 1 0 -X_1_p_warp{i}(3,j) 0 X_1_p_warp{i}(1,j); ...
                0 0 1 X_1_p_warp{i}(2,j) -X_1_p_warp{i}(1,j) 0; ...
                0 0 0 0 0 0];
            M2 = zeros(4, n_ring + n_ring);
            M2(1:3, n_ring + X_1_ch_{i, 1}(j,1)) =...
                T_01(1:3,1:3) * [cos(theta(j))*cos(psi(j)), cos(theta(j))*sin(psi(j)), sin(theta(j))]';
            M = [M1, M2];
            J(start_row + j,:) = plane_0_{i} * M;
        end
        start_row = start_row + length(X_1_{i});
    end
    
    R_10 = T_10(1:3,1:3);
    
    for i = 1:n_v_data
        [theta, psi] = theta_psi_generator(X_0_{i});
        for j=1:length(X_0_{i})
            M1 = [ -R_10, [-R_10(7)*X_0_p{i}(2,j)+R_10(4)*X_0_p{i}(3,j) , R_10(7)*X_0_p{i}(1,j)-R_10(1)*X_0_p{i}(3,j) , -R_10(4)*X_0_p{i}(1,j)+R_10(1)*X_0_p{i}(2,j);...
                -R_10(8)*X_0_p{i}(2,j)+R_10(5)*X_0_p{i}(3,j) , R_10(8)*X_0_p{i}(1,j)-R_10(2)*X_0_p{i}(3,j) , -R_10(5)*X_0_p{i}(1,j)+R_10(2)*X_0_p{i}(2,j);...
                -R_10(9)*X_0_p{i}(2,j)+R_10(6)*X_0_p{i}(3,j) , R_10(9)*X_0_p{i}(1,j)-R_10(3)*X_0_p{i}(3,j) , -R_10(6)*X_0_p{i}(1,j)+R_10(3)*X_0_p{i}(2,j)];...
                0, 0, 0, 0 , 0, 0];
            M2 = zeros(4, n_ring+ n_ring);
            M2(1:3, X_0_ch_{i, 1}(j,1)) =...
                T_10(1:3,1:3) * [cos(theta(j))*cos(psi(j)), cos(theta(j))*sin(psi(j)), sin(theta(j))]';
            M = [M1, M2];
            J(start_row + j,:) = plane_1_{i} * M;
        end
        start_row = start_row + length(X_0_{i});
    end
    
    % Calculate residual
    start_row = 0;
    for i = 1:n_v_data
        for j=1:length(X_1_{i})
            r(start_row + j) = plane_0_{i} *  X_1_p_warp{i,1}(:,j);
        end
        start_row = start_row + length(X_1_{i});
    end
    
    for i = 1:n_v_data
        for j=1:length(X_0_{i})
            r(start_row + j) = plane_1_{i} *  X_0_p_warp{i,1}(:,j);
        end
        start_row = start_row + length(X_0_{i});
    end
    
    r_total(:,iter)=r;
    r_norm_total(:,iter)= norm(r);
    
    if(iter>1)
        W = ones(size(r));
        over_ind = abs(r) > huber_thr;
        W(over_ind) = huber_thr ./ abs(r(over_ind));
        JW = bsxfun(@times,J,W);
        H  = JW.'*J;
        delta_s = -(H+(1e-5)*eye(size(H)))^-1*JW'*r;
    else
        W = ones(size(r));
        H  = J.'*J;
        delta_s = -(H+(1e-5)*eye(size(H)))^-1*J'*r;
    end
    expm_xi  = se3Exp(delta_s(1:6,1))*se3Exp(xi_01);
    xi_01    = se3Log(expm_xi);
    del_rho0 = del_rho0 + delta_s(7:6+n_ring,1);
    del_rho1 = del_rho1 + delta_s(6+n_ring+1: end,1);
    
    residual_curr = norm(W.*r);
    cost_save = [cost_save, residual_curr];
    
    T_01 = se3Exp(xi_01);
    
    plane_0_p=cell(n_v_data,1);
    plane_1_p=cell(n_v_data,1);
    
    for i = 1: n_v_data
        [plane,~,~,~] = plane_reweight(X_1_p{i},[],0);
        plane_1_p{i} = plane;
    end
    for i = 1: n_v_data
        [plane,~,~,~] = plane_reweight(X_0_p{i},[],0);
        plane_0_p{i} = plane;
    end
    
    plane_0_ = plane_0_p;
    plane_1_ = plane_1_p;
    
    fprintf('norm(delta_s) %d-th iter: %f\n',iter,norm(delta_s))
    if(norm(delta_s) <= 1e-3)
        figure();
        plot(s_save(1,:)); hold on;
        plot(s_save(2,:));
        plot(s_save(3,:));
        plot(s_save(4,:));
        plot(s_save(5,:));
        plot(s_save(6,:));
        xlabel('iteration');
        xlim([1,size(s_save,2)]);
        title('se(3) optimization');
        legend('xi1','xi2','xi3','xi4','xi5','xi6');
        break;
    end
    
    iter=iter+1;
end

data.l0.plane_0_p = plane_0_p;
data.l1.plane_1_p = plane_1_p;
data.l0.X_0_p = X_0_p;
data.l1.X_1_p = X_1_p;
data.l0.X_0_p_warp = X_0_p_warp;
data.l1.X_1_p_warp = X_1_p_warp;
data.l0.del_rho0 = del_rho0;
data.l1.del_rho1 = del_rho1;

for i=1:n_v_data
    data.l0.n_0_p{i} = data.l0.plane_0_p{i}'; %row -> col
    data.l1.n_1_p{i} = data.l1.plane_1_p{i}'; %row -> col
end

T_01 = se3Exp(xi_01);
end