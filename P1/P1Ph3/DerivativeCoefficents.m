function [constantCoeffs] = DerivativeCoefficents(order,derivat,scale)
%DerivativeCoefficents generates the coefficents for the A matrix. These
%are the coefficents of the polynomial obtained by differentiation it
%derivat number of times and multiplying with time.
derCoeffs = ones(1,order);
powers = 0:(order-1);
indexArray = 1:order;
for k =1:derivat
   derCoeffs = derCoeffs.*(indexArray -  ones(1,order));
   indexArray = indexArray -  ones(1,order);
end
derCoeffs(derCoeffs < 0) = 0;
powers = powers - derivat * ones(1,order);
powers(powers < 0) = 0;
time = ones(1,order);
scales = scale * ones(1,order);
scales(1,1) = 1;
index = find(derCoeffs > 0,1);
time(1,index:order) = scales(1,index:order) .^ powers(1,index:order);
constantCoeffs = derCoeffs .* time;
end