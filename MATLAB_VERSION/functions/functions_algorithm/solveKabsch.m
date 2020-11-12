function R_01 = solveKabsch(data)
N_0 = data.l0.n_0;
N_1 = data.l1.n_1;

N = size(N_0,1);
Sigma_ncinli = zeros(3,3);
for n = 1:N
   Sigma_ncinli = Sigma_ncinli +N_1{n}(1:3,1)*N_0{n}(1:3,1)';
end
Sigma_ncinli = Sigma_ncinli*1/N;

[U,~,V] = svd(Sigma_ncinli);
R_01 = V*U';
end