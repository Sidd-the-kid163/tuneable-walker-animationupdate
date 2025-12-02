function Dy = bezier6_partial(aMat, x)
%%% partial bezier / partial x

baseDof = 6;
Dy = zeros(size(aMat,1), 1); %this is a matrix if state based.

for i = baseDof+1 : size(aMat, 1)

    a0 = aMat(i, 1);
    a1 = aMat(i, 2);
    a2 = aMat(i, 3);
    a3 = aMat(i, 4);
    a4 = aMat(i, 5);
    a5 = aMat(i, 6);
    a6 = aMat(i, 7);

    Dy(i-baseDof) = ...
    6*a6*x^5 - 6*a5*x^5 + 6*a0*(x - 1)^5 - 6*a1*(x - 1)^5 ...
    - 30*a1*x*(x - 1)^4 + 30*a2*x*(x - 1)^4 - 30*a5*x^4*(x - 1) + 15*a4*x^4*(2*x - 2) ...
    + 60*a2*x^2*(x - 1)^3 - 60*a3*x^2*(x - 1)^3 ...
    - 60*a3*x^3*(x - 1)^2 + 60*a4*x^3*(x - 1)^2;

end

end
 
