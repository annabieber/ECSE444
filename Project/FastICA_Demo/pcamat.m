function [E, D] = pcamat(vectors)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate PCA

% Calculate the covariance matrix.
covarianceMatrix = cov(vectors', 1);

% Calculate the eigenvalues and eigenvectors of covariance
% matrix.
[E, D] = eig (covarianceMatrix);
