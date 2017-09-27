function [coeffsX,coeffsY,coeffsZ] = getCoeffs(orderPoly,orderSystem,numberpoly,waypoints)
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

coeffEachPoly = orderPoly + 1;
constraints = numberpoly * coeffEachPoly;
A = zeros(constraints,constraints);
currentRow = 1;
currentCol = 1;
%Each poly should pass through the starting waypoint
for i = 1 : numberpoly
    A(currentRow,currentCol:currentCol + orderPoly) = DerivativeCoefficents(coeffEachPoly , 0 , 0);
    currentRow = currentRow + 1;
    currentCol = currentCol + coeffEachPoly;
end
currentCol = 1;

%Each poly should pass through the ending waypoint
for i = 1 : numberpoly
A(currentRow,currentCol:currentCol + orderPoly) = DerivativeCoefficents(coeffEachPoly, 0 , 1);
currentRow = currentRow + 1;
currentCol = currentCol + coeffEachPoly;
end
%currentCol = 1;

%It should be continuous till (orderPoly - 1) derivative
for i = 1:(numberpoly - 1)
	for j = 1 : (orderPoly - 1)
        currentCol = 1 + coeffEachPoly * (i - 1);
		A(currentRow,currentCol:currentCol + orderPoly) = DerivativeCoefficents(coeffEachPoly , j , 1);
		currentCol = currentCol + coeffEachPoly;
		A(currentRow,currentCol:currentCol + orderPoly) = -DerivativeCoefficents(coeffEachPoly , j , 0);
		currentRow = currentRow + 1;
	end
end

%Start and stop at rest
currentCol = 1;
for i=1 : (orderSystem - 1)
	A(currentRow,currentCol:currentCol + orderPoly) = DerivativeCoefficents(coeffEachPoly,i,0);
	currentRow = currentRow + 1;
end

currentCol = numberpoly * coeffEachPoly - orderPoly;
for i= 1 : (orderSystem - 1)
	A(currentRow,currentCol:currentCol + orderPoly) = DerivativeCoefficents(coeffEachPoly,i,1);
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