function totalDist = Dist2Goal(agent, paths, goalI)
%% This function returns the driving distance from an AMR's current location to 
%% the machine. Part of the AURORA time entropy simulation.
%% E. Halbach   VTT   2023

loc = agent.KIN.LOC;
point = agent.STATE.pathPt;
if point > 0
  Path = paths{agent.STATE.currPath};
end
goal = paths{goalI};

laneDist = 0; baseDist = 0; newLaneDist = 0; newLane = 0; dist0 = 0;

if agent.STATE.TASK == 4 % AMR is driving back
  if point > 0 % Still in lane
    if point == 2 % In previous machine branch
      dist0 = abs(loc(1) - Path(2,1));
    end 
    % Check if AMR is in new machine lane
    if Path(1,1) == goal(1,1) % AMR is in same lane as goal
      dist1 = abs(loc(2) - goal(2,2)); % Lane distance
      dist2 = abs(goal(3,1) - goal(2,1)); % Branch length
      laneDist = dist0 + dist1 + dist2;
    else % Goal is in a different lane
      distToBase = abs(loc(2) - Path(1,2));
      laneDist = dist0 + distToBase;
      baseDist = abs(Path(1,1) - goal(1,1));
      newLane = 1;
    end
  elseif point == 0
    baseDist = abs(loc(1) - goal(1,1));
    newLane = 1;
  end
elseif agent.STATE.TASK == 1 
  baseDist = abs(goal(1,1) - loc(1));
  newLane = 1;
end

if newLane % Calculate distance in new lane
  for ii = 2:rows(goal)
    newLaneDist = newLaneDist + norm( goal(ii,:) - goal(ii-1,:) );
  end
end

totalDist = laneDist + baseDist + newLaneDist;
%% Assume goal is not machine that we just serviced a few seconds ago