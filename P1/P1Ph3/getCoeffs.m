function [coeffsX,coeffsY,coeffsZ] = getCoeffs(orderPoly,orderSystem,numberpoly,waypoints, eachPolyTime)
%getCoeffs generates the unknown co-efficients for all the piece-wise
%polynomials.
%orderPoly - Order of the polynomial we are trying to fit
%orderSystem - Order of the system of which dynamics is to be
%studied.(Quadrotors are modelled as fourth order system)
%numberpoly - number of polynomials we are trying to fit through the given
%waypoints.
%waypoints - a 3 * n matrix of the co-ordinates of X,Y,Z waypoints, where n is
%the number of waypoints. numberpoly should be n - 1.
%FurtherMore all the trajectories are applicable with the assumption that
%time taken to travel each polynomial has been scaled between 0 and 1.

numcoeffEachPoly = orderPoly + 1;
constraints = numberpoly * numcoeffEachPoly;
A = zeros(constraints,constraints);
currentRow = 1;
currentCol = 1;
%Each poly should pass through the starting waypoint
coeffstzero = DerivativeCoefficents(numcoeffEachPoly , 0 , 0);
for i = 1 : numberpoly
    A(currentRow,currentCol:currentCol + orderPoly) = coeffstzero;
    currentRow = currentRow + 1;
    currentCol = currentCol + numcoeffEachPoly;
end
currentCol = 1;

%Each poly should pass through the ending waypoint
coeffstone = DerivativeCoefficents(numcoeffEachPoly, 0 , 1);
for i = 1 : numberpoly
A(currentRow,currentCol:currentCol + orderPoly) = coeffstone;
currentRow = currentRow + 1;
currentCol = currentCol + numcoeffEachPoly;
end
%currentCol = 1;

%It should be continuous till (orderPoly - 1) derivative
for i = 1:(numberpoly - 1)
	for j = 1 : (orderPoly - 1)
        currentCol = 1 + numcoeffEachPoly * (i - 1);
		A(currentRow,currentCol:currentCol + orderPoly) = (DerivativeCoefficents(numcoeffEachPoly , j , 1)) ./(eachPolyTime(i)^j) ;
		currentCol = currentCol + numcoeffEachPoly;
		A(currentRow,currentCol:currentCol + orderPoly) = -(DerivativeCoefficents(numcoeffEachPoly , j , 0)) ./(eachPolyTime(i + 1)^j);
		currentRow = currentRow + 1;
	end
end

%Start and stop at rest
currentCol = 1;
for i=1 : (orderSystem - 1)
	A(currentRow,currentCol:currentCol + orderPoly) = DerivativeCoefficents(numcoeffEachPoly,i,0) ./(eachPolyTime(1)^i);
	currentRow = currentRow + 1;
end

currentCol = numberpoly * numcoeffEachPoly - orderPoly;
for i= 1 : (orderSystem - 1)
	A(currentRow,currentCol:currentCol + orderPoly) = DerivativeCoefficents(numcoeffEachPoly,i,1) ./(eachPolyTime(end)^i);
	currentRow = currentRow + 1;
end
%display(A);
b = zeros(constraints,1);

b(1:numberpoly,1) = waypoints(1:numberpoly,1);
b((numberpoly + 1):numberpoly*2,1) = waypoints(2:(numberpoly + 1),1);
coeffsX = A\b;

b(1:numberpoly,1) = waypoints(1:numberpoly,2);
b((numberpoly + 1):numberpoly*2,1) = waypoints(2:(numberpoly + 1),2);
coeffsY = A\b;

b(1:numberpoly,1) = waypoints(1:numberpoly,3);
b((numberpoly + 1):numberpoly*2,1) = waypoints(2:(numberpoly + 1),3);
coeffsZ = A\b;
end