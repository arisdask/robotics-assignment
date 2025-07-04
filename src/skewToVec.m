function v = skewToVec(S)
    % Extract vector from skew-symmetric matrix
    % For 3x3 skew-symmetric matrix S, returns 3x1 vector v
    % such that S = hat(v), where hat() is the skew-symmetric operator
    v = [S(3,2); S(1,3); S(2,1)];
end
