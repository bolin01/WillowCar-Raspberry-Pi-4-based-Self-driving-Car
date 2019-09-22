function [IsOntheLeft,A_,b_]=LeftorRightofLine(CurrentLoc,LineStartLoc,LineEndLoc)
%% Input
% CurrentLoc: 2x1
% LineStartLoc: 2x1
% LineEndLoc: 2x1
%% Output
% IsOntheLeft: 1x1 Boolean
% A_: 1x2 for inequality constraint
% b_: 1x1 for inequality constraint
% If CurrentLoc is the right, A_*x - b_ < 0
% If CurrentLoc is the left, - A_*x + b_ < 0

%% Symbolic cross function
% syms v11 v12 v21 v22
% v3 = cross([v11;v12;0],[v21;v22;0]);
% v33 = v3(3); % v33 = v11*v22 - v12*v21

    %% Implementation
    v1=LineStartLoc-CurrentLoc;
    v2=LineEndLoc-CurrentLoc;
    if v1(1,1)*v2(2,1) - v1(2,1)*v2(1,1)>0
        IsOntheLeft=true;
    else
        IsOntheLeft=false;
    end
    
%% Symbolic cross function
% syms x y x1 y1 x2 y2
% v1=[x1-x;y1-y];
% v2=[x2-x;y2-y];
% v3=cross([v1;0],[v2;0]);
% v3(3)
% A_=[y1-y2 x2-x1];
% b_=-x1*y2+x2*y1;
    %% Inequality factors (for bl)
    x1=LineStartLoc(1,1);
    y1=LineStartLoc(2,1);
    x2=LineEndLoc(1,1);
    y2=LineEndLoc(2,1);
    A_=[y1-y2 x2-x1];
    b_=-x1*y2+x2*y1;
end