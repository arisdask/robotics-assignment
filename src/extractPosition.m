% Extract position vector from transformation matrix
function p = extractPosition(T)
    p = T(1:3, 4);
end
