function y = bezier6(aMat, x)

baseDof = 6;
y = zeros(size(aMat,1), 1);

for i = baseDof+1 : size(aMat, 1)

    a0 = aMat(i, 1);
    a1 = aMat(i, 2);
    a2 = aMat(i, 3);
    a3 = aMat(i, 4);
    a4 = aMat(i, 5);
    a5 = aMat(i, 6);
    a6 = aMat(i, 7);

    y(i-baseDof) = ...
                     a0 *(1-x)^6      + ...
                  6 *a1 *(1-x)^5 *x   + ...
                 15 *a2 *(1-x)^4 *x^2 + ...
                 20 *a3 *(1-x)^3 *x^3 + ...
                 15 *a4 *(1-x)^2 *x^4 + ...
                  6 *a5 *(1-x)   *x^5 +  ...
                     a6          *x^6;

end





end