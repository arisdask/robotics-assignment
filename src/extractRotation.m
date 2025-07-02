% Extract rotation matrix from transformation matrix
function R = extractRotation(T)
    R = T(1:3, 1:3);
end
