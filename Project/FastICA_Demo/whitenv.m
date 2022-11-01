function [newVectors, whiteningMatrix, dewhiteningMatrix] = whitenv ...
    (vectors, E, D)

% ========================================================
% Calculate the whitening and dewhitening matrices (these handle
% dimensionality simultaneously).
whiteningMatrix = inv(sqrt (D)) * E';
dewhiteningMatrix = E * sqrt (D);

% Project to the eigenvectors of the covariance matrix.
% Whiten the samples and reduce dimension simultaneously.
newVectors =  whiteningMatrix * vectors;