function CheckPolynomial(coeffsX, coeffsY, coeffsZ, numcoeffs, numberpoly)
pos = [];

for i = 1 :numberpoly
    highpos = i * numcoeffs;
    lowpos = highpos - numcoeffs + 1;
    cx = coeffsX(lowpos:highpos,1);
    cy = coeffsY(lowpos:highpos,1);
    cz = coeffsZ(lowpos:highpos,1);
    for scale = 0:0.01:1
        dcoeffs = DerivativeCoefficents(numcoeffs,0,scale);      
        newpos = [cx' *dcoeffs'; cy' * dcoeffs'; cz' * dcoeffs']; 
        pos = [pos;newpos'];
    end
end

figure(1)
hold on
plot3(pos(:,1),pos(:,2),pos(:,3),'g', 'LineWidth', 2)
end