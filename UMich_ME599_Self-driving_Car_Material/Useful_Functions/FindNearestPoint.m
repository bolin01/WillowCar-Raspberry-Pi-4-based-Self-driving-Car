function [NearestPointIndex,SecondNearestPointIndex]=FindNearestPoint(CurrentLoc,WayPoints)
% Input:
% CurrentLoc: 2x1
% WayPoints: 2xN
% Output:
% NearestPointIndex: 1x1
% SecondNearestPointIndex: 1x1

    distance=(WayPoints(1,:)-CurrentLoc(1,1)).^2+(WayPoints(2,:)-CurrentLoc(2,1)).^2;
    [~,distance_index]=sort(distance,'ascend'); % Sort the distance in ascending order
    NearestPointIndex=distance_index(1); % Claim the Nearest point index
    SecondNearestPointIndex=distance_index(2); % Claim the Second Nearest point index
    
end 