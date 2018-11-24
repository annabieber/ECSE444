function [A, W] = fpica(X, whiteningMatrix, dewhiteningMatrix, ...
                        epsilon, maxNumIterations)

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Default values

  if nargin < 3, error('Not enough arguments!'); end
  if nargin < 5, maxNumIterations = 1000; end
  if nargin < 4, epsilon = 0.0001; end
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  [numOfIC, numSamples] = size(X);
  B = zeros(numOfIC);  
  % The search for a basis vector is repeated numOfIC times.
  round = 1;

  while round <= numOfIC,

    % Take a random initial vector of lenght 1 and orthogonalize it
    % with respect to the other vectors.
    w = randn (numOfIC, 1);
    w = w - B * B' * w;
    w = w / norm(w);

    wOld = zeros(size(w));

    % This is the actual fixed-point iteration loop.
    %    for i = 1 : maxNumIterations + 1
    i = 1;
    while i <= maxNumIterations

      % Project the vector into the space orthogonal to the space
      % spanned by the earlier found basis vectors. Note that we can do
      % the projection with matrix B, since the zero entries do not
      % contribute to the projection.
      w = w - B * B' * w;
      w = w / norm(w);

      % Test for termination condition. Note that the algorithm has
      % converged if the direction of w and wOld is the same, this
      % is why we test the two cases.
      if norm(w - wOld) < epsilon || norm(w + wOld) < epsilon
        % Save the vector
        B(:, round) = w;

        % Calculate the de-whitened vector.
        A(:,round) = dewhiteningMatrix * w;
        % Calculate ICA filter.
        W(round,:) = w' * whiteningMatrix;

        break; % IC ready - next...
      end

      wOld = w;

      % pow3
      w = (X * ((X' * w) .^ 3)) / numSamples - 3 * w;

      % Normalize the new w.
      w = w / norm(w);
      i = i + 1;
    end
    round = round + 1;
  end
  
end