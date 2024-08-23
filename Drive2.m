function TRAC = Drive2(TRAC,FPS)
%% This function updates the pose of a vehicle "TRAC". First, heading is 
%% adjusted by turning in place until facing the destination. Next, the vehicle 
%% drives towards its desired location. Finally, heading is adjusted again if a 
%% specific final orientation is desired. All movements are updated with 
%% increments based on the vehicle driving and turning speed specified in the 
%% TRAC structure. FPS is the "frames per second" rate of the simulation. 
%% Part of the GIM Integrator Simulator.
%% E. Halbach   Aalto University   2012

dX = TRAC.STATE.DEST(1) - TRAC.KIN.LOC(1);
dY = TRAC.STATE.DEST(2) - TRAC.KIN.LOC(2);
distToDest = sqrt(dX^2+dY^2);
gH = atan2(-dX,dY);  % global heading to dest from reference point

heading = TRAC.KIN.HEADING;  % current vehicle heading

if distToDest > 1.1*TRAC.SPECS.SPD/FPS  % should be more than "1*(drive increment)", to avoid overshooting dest
    if TRAC.STATE.BACKWARDS == 1
        heading = heading + pi;
%         heading = atan2(sin(heading),cos(heading));
    end
    % vector to destination in local coords:
    locDest = [cos(heading) sin(heading);-sin(heading) cos(heading)]*[dX;dY];
    headingToDest = atan2(-locDest(1),locDest(2));   % in local frame

    if abs(headingToDest) > 1.1*TRAC.SPECS.TURN/FPS % should be more than "1*...", to avoid overshooting desired heading
        turn = TRAC.SPECS.TURN/FPS*sign(headingToDest);
        heading = TRAC.KIN.HEADING + turn;    % update heading
%         heading = atan2(sin(heading),cos(heading));
        TRAC.KIN.HEADING = heading;    % update heading
        TRAC.STATE.turned = TRAC.STATE.turned + abs(turn); % keep track of amount turned
        return
    end
    heading = gH;          % fine-correct heading
    driveIncr=1/FPS*TRAC.SPECS.SPD;      % forward drive increment; each loop iteration is one frame
    driveVector=[0;driveIncr;0];         % forward drive vector in local frame (along y)
    Rz=[    cos(heading)    -sin(heading)   0
            sin(heading)    cos(heading)    0
            0               0               1   ];
    GlobDrive=Rz*driveVector;      % forward drive vector in global frame, used to update position
    TRAC.KIN.LOC=TRAC.KIN.LOC+GlobDrive(1:2)';
    TRAC.STATE.driven = TRAC.STATE.driven + driveIncr; % keep track of driving distance
    if TRAC.STATE.BACKWARDS == 1
        heading = heading - pi;
        heading = atan2(sin(heading),cos(heading));
    end
    TRAC.KIN.HEADING = heading;
    TRAC.STATE.turned = TRAC.STATE.turned + abs(headingToDest); % keep track of amount turned
    return
else % Fine-correct location to exact destination (optional, can comment out)
    TRAC.KIN.LOC = TRAC.STATE.DEST;
end
% numDists=length(TRAC.dists);
% TRAC.dists(numDists+1)=distToDest;
if TRAC.STATE.endHeadingFlag
    headingToDest = atan2(sin(TRAC.STATE.endHeading-heading),cos(TRAC.STATE.endHeading-heading));
    if abs(headingToDest) > 1.1*TRAC.SPECS.TURN/FPS % should be more than "1*...", to avoid overshooting desired heading
        turn = TRAC.SPECS.TURN/FPS*sign(atan2(sin(TRAC.STATE.endHeading-heading),cos(TRAC.STATE.endHeading-heading)));
        TRAC.KIN.HEADING = TRAC.KIN.HEADING + turn;    % update heading
        TRAC.STATE.turned = TRAC.STATE.turned + abs(turn); % keep track of amount turned
        return
    else
        TRAC.KIN.HEADING = TRAC.STATE.endHeading;  % fine-correct heading
        TRAC.STATE.turned = TRAC.STATE.turned + abs(headingToDest); % keep track of amount turned
    end
end
TRAC.STATE.endPose = true;
