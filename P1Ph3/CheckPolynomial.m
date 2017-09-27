function CheckPolynomial(coeffsX, coeffsY, coeffsZ, numcoeffs)
pos = [];
for scale = 0:0.01:1
    newpos = [sum(coeffsX(1:numcoeffs,1) .* (DerivativeCoefficents(numcoeffs,0,scale))');...
                            sum(coeffsY(1:numcoeffs,1) .* (DerivativeCoefficents(numcoeffs,0,scale))');...
                            sum(coeffsZ(1:numcoeffs,1) .* (DerivativeCoefficents(numcoeffs,0,scale))');...
                            ];
    pos = [pos;newpos'];
end
for scale = 0:0.01:1
    newpos = [sum(coeffsX((numcoeffs+1):2*numcoeffs,1) .* (DerivativeCoefficents(numcoeffs,0,scale))');...
                            sum(coeffsY((numcoeffs+1):2*numcoeffs,1) .* (DerivativeCoefficents(numcoeffs,0,scale))');...
                            sum(coeffsZ((numcoeffs+1):2*numcoeffs,1) .* (DerivativeCoefficents(numcoeffs,0,scale))');...
                            ];
    pos = [pos;newpos'];
end
figure(1)
hold on
plot3(pos(:,1),pos(:,2),pos(:,3),'g', 'LineWidth', 2)
end