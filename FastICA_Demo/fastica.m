function [Out1, Out2, Out3] = fastica(mixedsig)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Remove the mean and check the data

[mixedsig, mixedmean] = remmean(mixedsig);

[Dim, NumOfSampl] = size(mixedsig);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Default values for 'fpica' parameters
epsilon           = 0.0001;
maxNumIterations  = 1000;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculating PCA
[E, D]=pcamat(mixedsig);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Whitening the data
[whitesig, whiteningMatrix, dewhiteningMatrix] = whitenv(mixedsig, E, D);
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculating the ICA
  
% Check some parameters
% The dimension of the data may have been reduced during PCA calculations.
% The original dimension is calculated from the data by default, and the
% number of IC is by default set to equal that dimension.
  
Dim = size(whitesig, 1);
  
% Calculate the ICA with fixed point algorithm.
[A, W] = fpica (whitesig,  whiteningMatrix, dewhiteningMatrix, ...
                epsilon, maxNumIterations);
  
% Check for valid return
if ~isempty(W)
  % Add the mean back in.
  icasig = W * mixedsig + (W * mixedmean) * ones(1, NumOfSampl);
else
  icasig = [];
end


Out1 = icasig;
Out2 = A;
Out3 = W;

