%% Simulator for entropy based coordination of maintenance/servicing agents.
%% E. Halbach and S. Soutukorva   VTT   2023

clear
pkg load statistics
%pkg load instrument-control
%addpath('/home/eheric@ad.vtt.fi/VirtualBox VMs/Transfer/EarthSim/Functions')
%addpath('/home/eric/Research/EarthSim/Functions')

%% simulate in coppelia
coppeliaSimulation = false;

%% send requests to mir
MIR_REQUESTS = false;

%% graphics - can be toggled to speed up the simulation
GRAPHICS = 1;

SIM = 55; % Specify which simulation (details in TimeSimSettings.m)
SIMTIME = 1000; % Simulation time (0 for no end)
HORIZON = 50; % Data point horizon for averaging system info (avg. machine idle time and system entropy)

PLOTNORM = 0; % To plot the distributions at each machine service request (1 or 0)
PLOTSTATE = 0; % Plot state of system (entropy, mean idling time)
NEWHOME = 0; % If Home position is updated to base of current path
ONLYHOME = 1; % AMRs only available when waiting at HOME location, otherwise can bid from anywhere (when driving BACK)
BACKPT1 = 0; % AMRs available when driving BACK only to path point 1 in same lane as request
TURNTIME = 0; % Include estimate of turning time
SAMEINIT = 1; % For same machine initial work times (previously randomly generated, see TimeSimSetup)
NORAND = 0; % If we want no randomness (currently from machine request time)

TimeSimSettings % Run settings script

%% Sample initial working times for up to 20 machines:
MMinit = [ 0.4458591 0.2868102 0.6064691 0.0867070 0.4051517 0.2986949 0.7900244 0.0597029 0.1357336 0.3530106 ...
                      0.7108745 0.9402001 0.5617265 0.9322325 0.3404788 0.0037890 0.6618416 0.9983493 0.0285110 0.9992883 ];

FPS = 2; % Frames per second (frame as in simulation loop)

%% AMR states
WAIT = 1;   % Waiting at home
DRIVE = 2;  % Driving to machine
SERV = 3;   % Servicing machine (or ready to)
BACK = 4;   % Driving back home

## variables for tcp comms
new_rem = "";
old_rem = "";

%% MiR comms
if MIR_REQUESTS;
  HOST = "127.0.0.1"
  PORT = 65000
  s = tcp(HOST, PORT)
endif

%% Plot world
f1 = figure('Position',[fx fy fw fh]);
line(wallsX,wallsY);
axis([wallsX(1) wallsX(2) wallsY(1) wallsY(3)]);
text(corner(1)-0.45*dx,corner(2)+(MM(2)+0.8)*dy,'Time:');
timeString = text(corner(1)-0.3*dx,corner(2)+(MM(2)+0.8)*dy, sprintf("%.1f s", 0)); % Sim time
text(corner(1)-0.45*dx,corner(2)+(MM(2)+0.5)*dy,'Idle times: ');
hold on

%% Plot timelines
TL = 0;
if TL
  tl = figure('Position',[300 300 1000 600]);
end

%% Allocation strategies:
% 1 - Random driving
% 2 - Add all requests to queue for AMRs waiting at HOME
% 3 - Randomly allocate to available (not implemented yet)
% 4 - Highest Probability of arriving early, then System Entropy for tie-breaker
% 5 - Maximize System Entropy, then Probability of Arriving Early as tie-breaker
% 6 - Choose agent with fewest capabilities, then Probability of Arriving Early as tie-breaker
% 7 - Minimum Time Difference between request and bid (abs. value)
% 8 - Closest agent

ALLOC = 5;  

nextJob = 0;

%% Machine:
mach.loc = [0 0]; mach.size = ms;
mach.state = 1; mach.currentAMR = 0; % mach.STATES = ["WORK"; "REQUEST"; "SERVICE"; "IDLE"];
% Initialize times for machine work cycle: [mean sigma], sigma = std. dev.
mach.WORKms = [150 0];  mach.WORKt = mach.WORKms(1); % [280 works well for Sim 10 and speed = 0.5]
mach.REQms = [50 2];    mach.REQt = mach.REQms(1); % usually [50 2]
mach.SERVms = [20 0];   mach.SERVt = mach.SERVms(1);
mach.workTime = 0;    mach.workN = 0;     mach.workTimes = 0;
mach.requestTime = 0; mach.requestN = 0;  mach.requestTimes = 0;
mach.idleTime = 0;    mach.idleN = 0;     mach.idleTimes = 0;
mach.serviceTime = 0; mach.serviceN = 0;  mach.serviceTimes = 0;
%% Initialize machines:
machs(1:numM) = mach;

%% Initialize machines:
ecs = ['k' 'm' 'b' 'r']; % Edge colours for different machine types
for ii = 1:length(machs)
  machs(ii).loc = mlocs(ii,:);
  ec = ecs(MMtypes(ii));
  mh(ii) = patch(mlocs(ii,1)+[-1 1 1 -1]*mach.size(1)/2, mlocs(ii,2)+[-1 -1 1 1]*mach.size(2)/2, 'c', 'EdgeColor', ec, 'LineWidth', 5, 'FaceAlpha', 0.3);
  text(mlocs(ii,1)-0.6*mach.size(1)/2,mlocs(ii,2),sprintf('Machine %d',ii));
  sh(ii) = text(mlocs(ii,1)-0.6*mach.size(1)/2,mlocs(ii,2)-0.5*mach.size(2)/2,'Working'); % Initial status
  ih(ii) = text(mlocs(ii,1)-0.2*mach.size(1)/2,mlocs(ii,2)+1.5*mach.size(2)/2,sprintf("%.1f s", 0)); % Idle time tally
  machs(ii).WORKt =  machs(ii).WORKms(1) + randn*machs(ii).WORKms(2);
  if SAMEINIT
    machs(ii).workTime = MMinit(ii) * machs(ii).WORKt; % Previously randomly generated initial work times
  else
    machs(ii).workTime = rand*machs(ii).WORKt; % Randomize initial work times
  end
end

%% Draw lanes:
%line(corner(1)+[0 (MM(1,1)-1)*dx], corner(2)*[1 1],'Color','b');
line([paths{1}(1,1) paths{end}(1,1)], [paths{1}(1,2) paths{end}(1,2)],'Color','b');
for ii = 1:length(paths)
  line(paths{ii}(:,1),paths{ii}(:,2),'Color','b');
end

%% Initialize AMRs (includes human workers):
amr.KIN.LOC = [0 0]; amr.KIN.HEADING = 0; % Zero heading is along +Y
amr.SPECS.SPDms = [1 0]; amr.SPECS.TURNms = pi/10*[1 0]; % Nominal (max.) driving (m/s) and turning (rad/s) speeds [mean std.dev.]
amr.SPECS.SPDrange = [0.85 0.95]; % Specify range of avg. speeds as fraction of max; proportional to distance to job (higher avg. for closer)
amr.SPECS.SPD = amr.SPECS.SPDms(1); amr.SPECS.TURN = amr.SPECS.TURNms(1); % Set driving and turning speed to nominal
amr.SPECS.timeS = 0;  % Std. dev. of arrival time
if SIM == 10
  amr.SPECS.length = 0.5; amr.SPECS.width = 0.25; amr.SPECS.plotColour = 'k'; % Dimensions and colour for rendering
else
  amr.SPECS.length = 1; amr.SPECS.width = 0.5; amr.SPECS.plotColour = 'k'; % Dimensions and colour for rendering
endif
amr.SPECS.useMIR = 0; % if mir is used to update the amr
amr.STATE.BACKWARDS = 0; amr.STATE.endHeadingFlag = 0; amr.STATE.endPose = 0; % Info for driving function Drive2()
amr.STATE.driven = 0; amr.STATE.turned = 0; % For keeping track of total driving and turning
amr.STATE.DEST = [0 0]; % Current destination
amr.STATE.TASK = WAIT;  % Current state
amr.STATE.currPath = 0; amr.STATE.pathPt = 0; % Current assigned path and path point destination
amr.STATE.newJob = 0; amr.STATE.newJobPt = 0; % New job info to be assigned while driving BACK
amr.STATE.free = 0; % Free time counter
amr.STATE.numJobs = 0; % Number of jobs assigned
amr.STATE.MiRState = 0;

amrs(1:numR) = amr; % Initialize array of all agents

% Body frame vectors for plotting amr:
bodyY = 0.5*amr.SPECS.length*[-sin(amr.KIN.HEADING) cos(amr.KIN.HEADING)];
bodyX = 0.5*amr.SPECS.width*[cos(amr.KIN.HEADING) sin(amr.KIN.HEADING)];
robodyX = [-bodyX(1)-bodyY(1) bodyX(1)-bodyY(1) bodyX(1)+bodyY(1) -bodyX(1)+bodyY(1)];
robodyY = [-bodyX(2)-bodyY(2) bodyX(2)-bodyY(2) bodyX(2)+bodyY(2) -bodyX(2)+bodyY(2)];
% Body frame vectors for plotting human:
bodyYh = 0.5*amr.SPECS.width*[-sin(amr.KIN.HEADING+pi/4) cos(amr.KIN.HEADING+pi/4)];
bodyXh = 0.5*amr.SPECS.width*[cos(amr.KIN.HEADING+pi/4) sin(amr.KIN.HEADING+pi/4)];
hubodyX = [-bodyXh(1)-bodyYh(1) bodyXh(1)-bodyYh(1) bodyXh(1)+bodyYh(1) -bodyXh(1)+bodyYh(1)];
hubodyY = [-bodyXh(2)-bodyYh(2) bodyXh(2)-bodyYh(2) bodyXh(2)+bodyYh(2) -bodyXh(2)+bodyYh(2)];

% Initialize and plot AMRs:
numT = rows(rcaps); % Number of types of machines
amrMS = 10; % marker size, usually 7
for ii = 1:length(amrs)
  amrs(ii).KIN.LOC = rlocs(ii,:);  amrs(ii).STATE.HOME = rlocs(ii,:);   amrs(ii).STATE.DEST = rlocs(ii,:);  amrs(ii).SPECS.SPDms(1) = speeds(ii);
  if rtypes(ii) == 9  % Human, can tend to all Machine types
    r1xdata = amrs(ii).KIN.LOC(1)+hubodyX; r1ydata = amrs(ii).KIN.LOC(2)+hubodyY;
    rh(ii) = patch(r1xdata,r1ydata,'b');
    amr.SPECS.SPDrange = [0.75 0.95];
  else
    r1xdata = amrs(ii).KIN.LOC(1)+robodyX; r1ydata = amrs(ii).KIN.LOC(2)+robodyY;
    %% Initialize type markers (all green initially)
    if MIR_REQUESTS && ii == 1
      rh(ii) = patch(r1xdata,r1ydata,'y'); % Robot handle
    else
      rh(ii) = patch(r1xdata,r1ydata,'g'); % Robot handle
    endif
    startPt = amrs(ii).KIN.LOC+bodyY-bodyY/numT; % First point for plotting agent capability marker
    for jj = 1:numT % Initialize capability markers on AMRs
      th(ii,jj) = plot(startPt(1)-(jj-1)*2*bodyY(1)/numT, startPt(2)-(jj-1)*2*bodyY(2)/numT, 'sg', 'MarkerSize', amrMS, 'MarkerFaceColor', 'g'); % Type handle
      if rcaps(jj,ii) == 1 % Capability match of AMR to machine type
        set(th(ii,jj),'MarkerFaceColor',ecs(jj)); % Type handle
      end
    end
  end
end

%% GUI:
stopButton = uicontrol('Style','Togglebutton','String','Stop','Position',[10 10 60 20]);%,'Callback',@stopSim);
pauseButton = uicontrol('Style','Togglebutton','String','Pause','Position',[10 35 60 20]);
fastButton = uicontrol('Style','Togglebutton','String','Fast','Position',[80 10 60 20]);
fasterButton = uicontrol('Style','Togglebutton','String','Faster','Position',[80 35 60 20]);
if ~GRAPHICS
  set(fastButton,'Value',1);
end

bg = uibuttongroup(f1,'Units','Pixels','Position',[10 60 110 110]);
bb(1) = uicontrol(bg,'Style','Radiobutton','String','Probability','Position',[5 87 130 20]); % ALLOC = 4
bb(2) = uicontrol(bg,'Style','Radiobutton','String','System Ent.','Position',[5 66 130 20]); % ALLOC = 5
bb(3) = uicontrol(bg,'Style','Radiobutton','String','Fewest Cap.','Position',[5 45 130 20]); % ALLOC = 6
bb(4) = uicontrol(bg,'Style','Radiobutton','String','Time Diff.','Position',[5 24 130 20]); % ALLOC = 7
bb(5) = uicontrol(bg,'Style','Radiobutton','String','Closest','Position',[5 3 130 20]); % ALLOC = 8

set(bb(ALLOC-3),'Value',1);

wt = uicontrol(f1,'Style','Edit','String',num2str(mach.WORKms(1)),'Position',[50 175 40 20]);
wtx = uicontrol(f1,'Style','Text','String','Work','Position',[10 175 40 20]);
rt = uicontrol(f1,'Style','Edit','String',num2str(mach.REQms(1)),'Position',[50 200 40 20]);
rtx = uicontrol(f1,'Style','Text','String','Req.','Position',[10 200 40 20]);
st = uicontrol(f1,'Style','Edit','String',num2str(mach.SERVms(1)),'Position',[50 225 40 20]);
stx = uicontrol(f1,'Style','Text','String','Serv.','Position',[10 225 40 20]);
hor = uicontrol(f1,'Style','Edit','String',num2str(HORIZON),'Position',[50 255 40 20]);
hortx = uicontrol(f1,'Style','Text','String','Pt.Horiz.','Position',[10 255 40 20]);

%% Find maximum path length from a home position to a machine. Used later for variable std. dev. in lateness
initLength = DriveDist(paths{1}(1,:), paths{1});
maxLength = initLength;  minLength = initLength;
for ii = 1:MM(1)
  for jj = 1:length(paths)
    pathLength = DriveDist(paths{ii}(1,:), paths{jj});
    if pathLength > maxLength
      maxLength = pathLength;
    end
    if pathLength < minLength
      minLength = pathLength;
    end
  end
end

%% Main loop:
ttime = 0;
multiAllocJobs = 0;  % Number of allocations with more than one capable agent available
singleAllocJobs = 0; % Number of allocations with only one capable agent available
allocJobs = 0;  % Number of allocations with at least one agent available
queueJobs = 0; % Number of allocations from the queue
totalJobs = 0; % Total number of jobs

allIdles = []; allSysEnts = []; sepIdles{1} = []; sepIdles{2} = []; sepIdles{3} = [];
if PLOTSTATE
  f3 = figure('Position',[800 400 600 400]);
  title('System State');
  xlabel('Time (minutes)');
  ylabel('System entropy / Avg. Idle Time');
  sysent = SystemEntropy(rcaps,ones(1,numR),0);
  [sysAx, hse, hai] = plotyy(ttime/60, sysent, ttime/60, mean(allIdles));
  ylabel(sysAx(1),'System Entropy');
  ylabel(sysAx(2),'Mean Idle Time (s)');
  set(hse,'Marker','s');
  set(hai,'Marker','o');
end


while ~get(stopButton,'Value')
  if ~get(pauseButton,'Value')
  ttime += 1/FPS;
  if ~get(fasterButton,'Value') % GRAPHICS
%  if GRAPHICS
    set(timeString,'String',sprintf("%.1f s", ttime));
  endif
  %% Update machines
  for ii = 1:length(machs)
    if machs(ii).state == 1
      machs(ii).workTime += 1/FPS;
      if machs(ii).workTime > machs(ii).WORKt %% Make request
          machs(ii).state = 2;
          if ~get(fasterButton,'Value') % GRAPHICS
%          if GRAPHICS
            set(mh(ii),"FaceColor","y");
            set(sh(ii),'String','Request');
          endif

          %% - Coppelia machine update
          if coppeliaSimulation
            machineStateUpdate = sprintf('mach,0,%i,%i,0',ii,2);
            tcp_write(s, machineStateUpdate);
          endif

          machs(ii).workN += 1;
          machs(ii).workTimes(machs(ii).workN) = machs(ii).workTime;
          machs(ii).workTime = 0;
          %% Set new request and service time if needed
          machs(ii).REQms(1) = str2num(get(rt,'String'));
          if ~NORAND % If we want no randomness
            machs(ii).REQt = machs(ii).REQms(1) + randn*machs(ii).REQms(2);
          end
          machs(ii).SERVms(1) = str2num(get(st,'String'));
          machs(ii).SERVt = machs(ii).SERVms(1) + randn*machs(ii).SERVms(2);
          %% Allocate AMR
          if ALLOC == 2 % Add to queue
            if nextJob(1) == 0
              nextJob = ii;
            else
              nextJob(end + 1) = ii;
            end
          elseif ALLOC > 2
            if PLOTNORM
              f2 = figure('Position',[2000 800 600 200]);
              title(sprintf('Machine %d',ii))
              xlabel('Time (seconds)')
              ylabel('Probability Density')
              nh = PlotNorm(machs(ii).REQms(1), machs(ii).REQms(2), 'r');
              hold on
            end
            bestMatch = 0; minH = 0; highProb = 0; bestNominalTime = 0; bestDist2job = 0; matchS = 0; fewestCap = 0; maxH = 0; % Variables for chosen AMR
            shortestDist = 0; totalTurn = 0; extraTime = 0; minTimeDiff = 0; lateness = 0; minLate = 0;
            numCapAv = 0; % Number of capable agents available
            for jj = 1:length(amrs) %% Estimate arrival times
              if rcaps(MMtypes(ii),jj) % First check if AMR capability matches machine type
              % Check if AMR is waiting at home or is driving back and considered available
              % If driving back, either available from anywhere (~BACKPT1) or only when driving to Point 1 in same lane as request (BACKPT1)
              if amrs(jj).STATE.TASK == WAIT || ~ONLYHOME && amrs(jj).STATE.TASK == BACK && ~amrs(jj).STATE.newJob && (~BACKPT1 || BACKPT1 && paths{amrs(jj).STATE.currPath}(1,1) == paths{ii}(1,1) && amrs(jj).STATE.pathPt == 1)
                numCapAv += 1;
                % Estimate driving time:
                %dist2job = DriveDist(amrs(jj).STATE.HOME, paths{ii}); % Driving distance to machine
                dist2job = Dist2Goal(amrs(jj),paths,ii);
                % Determine extra time waiting for servicing to finish if applicable:
      %          if amrs(jj).STATE.TASK == SERV
     %             if machs(amrs(jj).STATE.currPath).state == 2 % If machine still in Request state
    %                extraTime =
   %               elseif machs(amrs(jj).STATE.currPath).state == 4 % If machine (briefly) Idle?
  %                  extraTime =
   %               elseif machs(amrs(jj).STATE.currPath).state == 3 % If machine in Service state
    %                extraTime =
     %             end
      %          end
                nominalDriveTime = dist2job / amrs(jj).SPECS.SPDms(1); % At nominal (maximum) driving speed
                fraction = (dist2job-minLength) / (maxLength-minLength); % Fraction of max driving distance (subtract minimum)
                highMeanSpeed = amrs(jj).SPECS.SPDrange(2);
                meanSpeedRange = amrs(jj).SPECS.SPDrange(2) - amrs(jj).SPECS.SPDrange(1);
                currMeanSpeed = amrs(jj).SPECS.SPDms(1) * (highMeanSpeed - fraction*meanSpeedRange);
                meanDriveTime = dist2job / currMeanSpeed; % Mean driving time to machine (without turning)
                timeS = abs(meanDriveTime - nominalDriveTime) / 3; % 3 sigma - doesn't consider turning time
                %if timeS == machs(ii).REQms(2)
                %  "equal std. dev."
                %  timeS = machs(ii).REQms(2)*1.01; % Add 1% to std. dev. so not equal
                %end
                if TURNTIME
                  if amrs(jj).STATE.HOME(1) == paths{ii}(1) % Include turning time
                    totalTurn = pi/2;
                  else
                    totalTurn = 3*pi/2;
                  end
                end
                nominalTurnTime = totalTurn / amrs(jj).SPECS.TURNms(1);
                meanTime = meanDriveTime + nominalTurnTime + extraTime;
                if PLOTNORM
                  nh = PlotNorm(meanTime, timeS, amrs(jj).SPECS.plotColour);
                  pause(0.2)
                end
                taskP = ProbCalc2(machs(ii).REQms(1), machs(ii).REQms(2), meanTime, timeS);
                lateness = meanTime - machs(ii).REQms(1);
                %sysent = SystemEntropy(rcaps,Available(amrs),jj)
                sysent = SystemEntropy(rcaps,ones(1,numR),jj);
                
                %% Allocation:
                replaceBest = 0;
                if bestMatch == 0 % First match found
                  replaceBest = 1;
                elseif ALLOC == 4 % get(bb(3),'Value') % If using probability of being early
                  ALLOCATION_METHOD = "HIGHPROB";
                  if taskP > highProb
                    replaceBest = 1;
                  elseif taskP == highProb
                    if sysent > maxH
                    %if lateness < minLate
                      replaceBest = 1;
                    end
                  end  
                elseif ALLOC == 5 % get(bb(1),'Value') % Max. system entropy
                  ALLOCATION_METHOD = "SYSENT";
%                  sysent = SystemEntropy(rcaps,Available(amrs),jj);
                  if sysent > maxH
                    replaceBest = 1; % Use agent resulting in higher remaining system entropy
                  elseif sysent == maxH % We need to compare task entropy
                    if taskP > highProb
                      replaceBest = 1;
                    %elseif taskP == highProb
                    %  if lateness < minLate
                    %    replaceBest = 1;
                    %  end
                    end
                  end
                elseif ALLOC == 6 % get(bb(2),'Value') % Select minimum capabilities
                  ALLOCATION_METHOD = "MINCAP";
                  if sum(rcaps(:,jj)) < fewestCap
                    replaceBest = 1; % Use agent resulting in higher remaining system entropy
                  elseif sum(rcaps(:,jj)) == fewestCap % We need to compare task entropy
                    if taskP > highProb
                      replaceBest = 1;
                    %elseif taskP == highProb
                    %  if lateness < minLate
                    %    replaceBest = 1;
                    %  end
                    end
                  end
                elseif ALLOC == 7 % get(bb(4),'Value') % Time Diff. - if using min. abs. time diff.
                  ALLOCATION_METHOD = "TIME DIFF.";
                  if abs(lateness) < abs(minTimeDiff)
                    replaceBest = 1; % Use agent with smaller abs. time diff.
                  elseif abs(lateness) == abs(minTimeDiff) % See if new one is actually sooner
                    if lateness < minTimeDiff
                      replaceBest = 1;
                    end
                  end
                elseif ALLOC == 8 % get(bb(5),'Value') % Closest
                  ALLOCATION_METHOD = "CLOSEST";
                  %if DriveDist(amrs(jj).STATE.HOME, paths{ii}) < shortestDist
                  if Dist2Goal(amrs(jj),paths,ii) < shortestDist
                    replaceBest = 1; % Use closer agent
                  elseif Dist2Goal(amrs(jj),paths,ii) == shortestDist
                    if sysent > maxH
                      replaceBest = 1;
                    end
                  end
                end
                if replaceBest
                  bestMatch = jj;

                  highProb = taskP;
                  minLate = lateness;
                  minTimeDiff = lateness;
                  fewestCap = sum(rcaps(:,jj)); % Fewest capabilities (for higher system entropy)
                  %maxH = SystemEntropy(rcaps,Available(amrs),jj); % Remaining system entropy if current AMR is selected
                  maxH = sysent; % Remaining system entropy if current AMR is selected
                  %shortestDist = DriveDist(amrs(jj).STATE.HOME, paths{ii}); % If using SHORTEST
                  shortestDist = Dist2Goal(amrs(jj),paths,ii); % If using SHORTEST

                  %% Save driving info for later randomization:
                  bestMeanDriveTime = meanDriveTime;
                  bestMeanTime = meanTime;
                  bestDist2job = dist2job;
                  matchS = timeS;
                end
              end
              end
            end
            if bestMatch > 0 % If job is allocated to an available AMR
              %bestMatch
              if numCapAv > 1
                multiAllocJobs += 1;
                totalJobs += 1;
              elseif numCapAv == 1
                singleAllocJobs += 1;
                totalJobs += 1;
              end
              % sysent = SystemEntropy(rcaps,Available(amrs),0); % Results in errors?
              sysent = SystemEntropy(rcaps,ones(1,numR),bestMatch);
              allSysEnts(end+1) = sysent;
              if isnan(sysent)
                1
              end

              if PLOTSTATE
                axes(sysAx(1));
                hold on
                plot(ttime/60,sysent,'bs'); % Plot current system entropy
                %% Plot mean system entropy (over specified data point horizon):
                HORIZON = str2num(get(hor,'String'));
                if length(allSysEnts) > HORIZON
                  meanSysEnt = mean(allSysEnts(end-HORIZON:end));
                else
                  meanSysEnt = mean(allSysEnts);
                end
                plot(ttime/60,meanSysEnt,'bo','MarkerFaceColor','c');
                %meanSysEnt
                xlim([0 ttime/60]); axes(sysAx(2)); xlim([0 ttime/60]); % axes(sysAx(2));
              end

              %% - REQUEST AMR
              if MIR_REQUESTS
                fprintf('Sending mir request\n')
                AMRrequest = sprintf('tend,%i,%i,%i',bestMatch,ii,amrs(1).STATE.TASK)
                tcp_write(s, AMRrequest);
              endif

              %% This part is only if AMRs are considered available when driving "BACK", and accounts for their current location by reassigning here or delaying:
              if amrs(bestMatch).STATE.TASK == BACK && ~ONLYHOME % AMR currently driving back home
                if paths{amrs(bestMatch).STATE.currPath}(1,1) == paths{ii}(1,1) && amrs(bestMatch).STATE.pathPt > 0 % AMR is in same lane as machine
                  if amrs(bestMatch).STATE.pathPt == 2 % Still in branch of previous machine
                    amrs(bestMatch).STATE.newJob = ii; % Continue going back, reassign later. Assume new machine is not same as just serviced
                    amrs(bestMatch).STATE.newJobPt = 2;
                  elseif amrs(bestMatch).STATE.pathPt == 1 % In same lane
                    if amrs(bestMatch).KIN.LOC(2) > paths{ii}(end,2) % AMR is north of machine in same lane
                      amrs(bestMatch).STATE.newJob = ii; % Continue going back, reassign later
                      amrs(bestMatch).STATE.newJobPt = 3;
                    elseif amrs(bestMatch).KIN.LOC(2) <= paths{ii}(end,2) % AMR is south of machine in same lane or at new branch
                      amrs(bestMatch).STATE.BACKWARDS = 0;
                    end
                    amrs(bestMatch).STATE.pathPt = 2;
                    amrs(bestMatch).STATE.DEST = paths{ii}(2,:); % Assign new dest
                  end
                elseif amrs(bestMatch).STATE.pathPt == 0 % Driving home in base lane
                  if amrs(bestMatch).KIN.LOC(1) > paths{ii}(1,1) % AMR is east of machine lane
                    amrs(bestMatch).STATE.newJob = ii; % Continue going back, reassign later
                    amrs(bestMatch).STATE.newJobPt = 2;
                  elseif amrs(bestMatch).KIN.LOC(1) <= paths{ii}(1,1) % AMR is west of machine lane or at new lane
                    amrs(bestMatch).STATE.BACKWARDS = 0;
                  end
                  amrs(bestMatch).STATE.pathPt = 1;
                  amrs(bestMatch).STATE.DEST = paths{ii}(1,:); % Assign new dest
                  amrs(bestMatch).STATE.endHeadingFlag = 0;
                else % In different lane
                  amrs(bestMatch).STATE.newJob = ii; % Continue going back, reassign later
                  amrs(bestMatch).STATE.newJobPt = 1;
                end
              end

              %% Now activate DRIVE to new job if needed:
              if amrs(bestMatch).STATE.newJob == 0 % Can assign new job now, because it's not being stored for later
                amrs(bestMatch).STATE.currPath = ii;
                amrs(bestMatch).STATE.TASK = DRIVE;
              end
              amrs(bestMatch).STATE.numJobs += 1;
              %% Add variability in actual driving time:
              driveTime = bestMeanDriveTime + randn*matchS; % Normal distribution in drive time
              actualSpeed = bestDist2job / driveTime;
              % save system entropy
              sysentData = sprintf(";%f;%f;%i",ttime,sysent,ALLOC);
              save("-append", "aurora_sysents.txt", "sysentData");
              if ~NORAND
                amrs(bestMatch).SPECS.SPD = actualSpeed;
              end
              if actualSpeed < 0
                amrs(bestMatch).SPECS.SPD = amrs(bestMatch).SPECS.SPDms(1); % Set to nominal speed in case we got a negative value
                "Negative speed"
                break
              end
            elseif nextJob(1) == 0 % Queue job
              nextJob = ii;
            else % Queue job
              nextJob(end + 1) = ii;
            end
            if PLOTNORM
              if bestMatch > 0
                nh = PlotNorm(bestMeanTime, matchS, 'm');
              end
              pause(0.2)
              delete(f2)
            end
          end
      end
    elseif machs(ii).state == 2
      machs(ii).requestTime += 1/FPS;
      if machs(ii).requestTime > machs(ii).REQt
        machs(ii).state = 4;
        if ~get(fasterButton,'Value') % GRAPHICS
%        if GRAPHICS
          set(mh(ii),"FaceColor","r");
          set(sh(ii),'String','Idle');
        endif
        machs(ii).requestN += 1;
        machs(ii).requestTimes(machs(ii).requestN) = machs(ii).requestTime;
        machs(ii).requestTime = 0;

        %% - Coppelia machine update
        if coppeliaSimulation
          machineStateUpdate = sprintf('mach,0,%i,%i,0',ii,4);
          tcp_write(s, machineStateUpdate);
        endif

      end
    elseif machs(ii).state == 3
      machs(ii).serviceTime += 1/FPS;
      if machs(ii).serviceTime > machs(ii).SERVt
          machs(ii).state = 1;
          if ~get(fasterButton,'Value') % GRAPHICS
%          if GRAPHICS
            set(mh(ii),"FaceColor","c");
            set(sh(ii),'String','Working');
          endif
          machs(ii).serviceN += 1;
          machs(ii).serviceTimes(machs(ii).serviceN) = machs(ii).serviceTime;
          machs(ii).serviceTime = 0;
          if ~get(fastButton,'Value') % GRAPHICS
            machs(ii).WORKms(1) = str2num(get(wt,'String'));
          end
          machs(ii).WORKt = machs(ii).WORKms(2)*randn + machs(ii).WORKms(1);

          %% - Coppelia machine update
          if coppeliaSimulation
            machineStateUpdate = sprintf('mach,0,%i,%i,0',ii,1);
            tcp_write(s, machineStateUpdate);
          endif

          %% - RELEASE MIR
          if MIR_REQUESTS && machs(ii).currentAMR == 1
            AMRrequest = sprintf('return,%i,%i,%i',1,ii,amrs(1).STATE.TASK)
            tcp_write(s, AMRrequest)
            amrs(machs(ii).currentAMR).STATE.TASK = BACK;
            machs(ii).currentAMR = 0;
          endif

         %% Release AMR:
          if MIR_REQUESTS && machs(ii).currentAMR ~= 1 && machs(ii).currentAMR ~= 0
            amrs(machs(ii).currentAMR).STATE.pathPt = amrs(machs(ii).currentAMR).STATE.pathPt - 1;
            amrs(machs(ii).currentAMR).STATE.DEST = paths{amrs(machs(ii).currentAMR).STATE.currPath}(amrs(machs(ii).currentAMR).STATE.pathPt,:);
            amrs(machs(ii).currentAMR).STATE.BACKWARDS = 1;
            amrs(machs(ii).currentAMR).STATE.TASK = BACK;
            machs(ii).currentAMR = 0;
          else
            if ~MIR_REQUESTS
              amrs(machs(ii).currentAMR).STATE.pathPt = amrs(machs(ii).currentAMR).STATE.pathPt - 1;
              amrs(machs(ii).currentAMR).STATE.DEST = paths{amrs(machs(ii).currentAMR).STATE.currPath}(amrs(machs(ii).currentAMR).STATE.pathPt,:);
              amrs(machs(ii).currentAMR).STATE.BACKWARDS = 1;
              amrs(machs(ii).currentAMR).STATE.TASK = BACK;
              machs(ii).currentAMR = 0;
            endif
          end
      end
    elseif machs(ii).state == 4
      machs(ii).idleTime += 1/FPS;
      if ~get(fasterButton,'Value') % GRAPHICS
%      if GRAPHICS
        set(ih(ii),'String',sprintf("%.1f s", sum(machs(ii).idleTimes) + machs(ii).idleTime));
      endif
    end
  end

  %% Update AMRs
  for ii = 1:length(amrs)

    ##MIR COMMS
    ##

    if coppeliaSimulation
      if amrs(ii).STATE.TASK == DRIVE || amrs(ii).STATE.TASK == BACK
        AMRposeUpdate = sprintf('poses,%i,%d,%d,%d',ii,amrs(ii).KIN.LOC(1),amrs(ii).KIN.LOC(2),rad2deg(amrs(ii).KIN.HEADING)-90)
        tcp_write(s, AMRposeUpdate);
      endif
    endif

    if MIR_REQUESTS && ii == 1;
      %fprintf('Sending mir request');
      AMRrequest = sprintf('check,%i,0,%i,0',ii,amrs(ii).STATE.TASK);
      tcp_write(s, AMRrequest);
      amrs(ii).SPECS.useMIR = 1;
      data = tcp_read(s, 22, 10);

      if data
      [data, new_rem] = strtok(char(data),"x");

        if length(old_rem) > 1
          rem = substr(old_rem, 2);
          "rem + data: "
          data = strcat(rem,data)
          old_rem = "";
        endif
        data = strsplit(data, ",");
        old_rem = new_rem;
        new_rem = "";

        ## update mir state variable
        amrs(ii).STATE.MiRState = str2num(char(data(1,1)));
        ## update xy coordinates
        amrs(ii).KIN.LOC(1) = str2double(char(data(1,2)));
        amrs(ii).KIN.LOC(2) = str2double(char(data(1,3)));
        ## update orientation
        amrs(ii).KIN.HEADING = deg2rad(str2double(char(data(1,4)))+90);

        ## update amr state if destination reached by mir
        if amrs(ii).STATE.MiRState == SERV && amrs(ii).STATE.TASK == DRIVE
          amrs(ii).STATE.TASK = amrs(ii).STATE.MiRState;
          machs(amrs(ii).STATE.currPath).currentAMR = ii
        elseif amrs(ii).STATE.MiRState == WAIT && amrs(ii).STATE.TASK == BACK
          amrs(ii).STATE.TASK = amrs(ii).STATE.MiRState;
        endif
      endif
      pause(0.01)
    endif

    ##
    ##

    if amrs(ii).STATE.TASK == DRIVE || amrs(ii).STATE.TASK == BACK
      amrs(ii) = Drive2(amrs(ii),FPS); % Drives/turns towards destination, also checks if destination reached
      % Update graphics rendering:
      if ~get(fastButton,'Value')
%      if GRAPHICS
        if rtypes(ii) ~= 9
          bodyY = 0.5*amrs(ii).SPECS.length*[-sin(amrs(ii).KIN.HEADING) cos(amrs(ii).KIN.HEADING)];
          bodyX = 0.5*amrs(ii).SPECS.width*[cos(amrs(ii).KIN.HEADING) sin(amrs(ii).KIN.HEADING)];
          startPt = amrs(ii).KIN.LOC+bodyY-bodyY/numT; % First point for plotting agent capability marker
          for jj = 1:numT
            set(th(ii,jj),'XData',startPt(1)-(jj-1)*2*bodyY(1)/numT,'YData',startPt(2)-(jj-1)*2*bodyY(2)/numT);
          end
        elseif rtypes(ii) == 9
          bodyY = 0.5*amrs(ii).SPECS.width*[-sin(amrs(ii).KIN.HEADING+pi/4) cos(amrs(ii).KIN.HEADING+pi/4)];
          bodyX = 0.5*amrs(ii).SPECS.width*[cos(amrs(ii).KIN.HEADING+pi/4) sin(amrs(ii).KIN.HEADING+pi/4)];
        end
        robodyX = [-bodyX(1)-bodyY(1) bodyX(1)-bodyY(1) bodyX(1)+bodyY(1) -bodyX(1)+bodyY(1)];
        robodyY = [-bodyX(2)-bodyY(2) bodyX(2)-bodyY(2) bodyX(2)+bodyY(2) -bodyX(2)+bodyY(2)];
        r1xdata = amrs(ii).KIN.LOC(1)+robodyX; r1ydata = amrs(ii).KIN.LOC(2)+robodyY;
        set(rh(ii),'XData',r1xdata,'YData',r1ydata);
      endif
    elseif amrs(ii).STATE.TASK == WAIT
      if ALLOC == 1 && rand > 0.95 % ALLOC 1 is random driving
        amrs(ii).STATE.TASK = DRIVE;
        amrs(ii).STATE.currPath = randi([1 length(paths)]); % Randomly assign path
      elseif ALLOC > 1 && nextJob(1) > 0 && MatchingJob(nextJob,MMtypes,rcaps,ii)
        queueJobs += 1;
        totalJobs += 1;
        jobMatch = MatchingJob(nextJob,MMtypes,rcaps,ii);

        % REQUEST MIR
        if amrs(ii).SPECS.useMIR == 1
          ##fprintf('Sending mir request\n')
          amrs(ii).STATE.TASK = DRIVE;
          amrs(ii).STATE.numJobs += 1;
          amrs(ii).STATE.currPath = jobMatch;
          AMRrequest = sprintf('tend,%i,%i,%i,0',ii,jobMatch,amrs(1).STATE.TASK)
          tcp_write(s, AMRrequest);
        elseif amrs(ii).SPECS.useMIR ~= 1
          amrs(ii).STATE.TASK = DRIVE;
          amrs(ii).STATE.numJobs += 1;
          amrs(ii).STATE.currPath = jobMatch; % nextJob(1);
        endif

        if length(nextJob) == 1
          nextJob = 0;
        else
          nextJob = nextJob(nextJob ~= jobMatch); % Remove jobMatch from nextJob
        end
      end
    elseif amrs(ii).STATE.TASK == SERV
%      if machs(amrs(ii).STATE.currPath).state == 2 && machs(amrs(ii).STATE.currPath).requestTime >= machs(amrs(ii).STATE.currPath).REQt || machs(amrs(ii).STATE.currPath).state == 4
        %% Record idle or request times:
      if machs(amrs(ii).STATE.currPath).state == 4
        machs(amrs(ii).STATE.currPath).idleN += 1;
        machs(amrs(ii).STATE.currPath).idleTimes(machs(amrs(ii).STATE.currPath).idleN) = machs(amrs(ii).STATE.currPath).idleTime;
        allIdles(end+1) = machs(amrs(ii).STATE.currPath).idleTime;
        sepIdles{MMtypes(amrs(ii).STATE.currPath)}(end+1) = machs(amrs(ii).STATE.currPath).idleTime;
        if PLOTSTATE
          axes(sysAx(2));
          hold on
          plot(ttime/60, allIdles(end), 'rs'); % Plot latest machine idle time
          %% Plot mean machine idle time over specified past data point horizon:
          HORIZON = str2num(get(hor,'String'));
          if length(allIdles) > HORIZON
            idleMean = mean(allIdles(end-HORIZON:end));
          else
            idleMean = mean(allIdles);
          end
          plot(ttime/60,idleMean,'ro','MarkerFaceColor','y');
          xlim([0 ttime/60]); axes(sysAx(1)); xlim([0 ttime/60]); axes(sysAx(2));
        end

        machs(amrs(ii).STATE.currPath).idleTime = 0;
        machs(amrs(ii).STATE.currPath).state = 3;
        if ~get(fasterButton,'Value') % GRAPHICS
%        if GRAPHICS
          set(mh(amrs(ii).STATE.currPath),"FaceColor","m");
          set(sh(amrs(ii).STATE.currPath),'String','Servicing');
        endif

        if coppeliaSimulation
          machineStateUpdate = sprintf('mach,0,%i,%i,0',ii,3);
          tcp_write(s, machineStateUpdate);
        endif

      end
    end
    pause(0.01)

    if amrs(ii).STATE.endPose && amrs(ii).SPECS.useMIR ~= 1
      if amrs(ii).STATE.TASK == DRIVE
        if amrs(ii).STATE.pathPt == 0 % When starting new job from HOME
          amrs(ii).STATE.pathPt = 1;
          amrs(ii).STATE.DEST = paths{amrs(ii).STATE.currPath}(1,:);
          if NEWHOME
            amrs(ii).STATE.HOME(1) = paths{amrs(ii).STATE.currPath}(1,1); % New home so doesn't drive all way back
          end
        elseif amrs(ii).STATE.pathPt < rows(paths{amrs(ii).STATE.currPath})
          amrs(ii).STATE.pathPt = amrs(ii).STATE.pathPt + 1;
          amrs(ii).STATE.DEST = paths{amrs(ii).STATE.currPath}(amrs(ii).STATE.pathPt,:);
        else % Must be at end of path
          %% If random driving, check if machine needs servicing
          if machs(amrs(ii).STATE.currPath).state == 1 || machs(amrs(ii).STATE.currPath).currentAMR > 0
            % Machine working normally or other AMR present (only happens for random driving, ALLOC = 1)
            amrs(ii).STATE.pathPt = amrs(ii).STATE.pathPt - 1;
            amrs(ii).STATE.DEST = paths{amrs(ii).STATE.currPath}(amrs(ii).STATE.pathPt,:);
            amrs(ii).STATE.BACKWARDS = 1;
            amrs(ii).STATE.TASK = BACK;
          else
            machs(amrs(ii).STATE.currPath).currentAMR = ii; % Assign current AMR to machine
            amrs(ii).STATE.TASK = SERV;
          end
        end
      elseif amrs(ii).STATE.TASK == BACK
        if amrs(ii).STATE.pathPt > 1
          reassign = 0;
          if ~ONLYHOME && ~amrs(ii).STATE.newJob && nextJob(1) > 0 && ALLOC > 2 % Look for jobs in same lane
            for jj = 1:length(nextJob)
              if rcaps(MMtypes(nextJob(jj)),ii) && paths{nextJob(jj)}(1,1) == paths{amrs(ii).STATE.currPath}(1,1)
                reassign = 1;
                break
              end
            end
          end
          if reassign
            queueJobs += 1;
            totalJobs += 1;
            amrs(ii).STATE.currPath = nextJob(jj);
            amrs(ii).STATE.BACKWARDS = 0;
            amrs(ii).STATE.pathPt = 2;
            amrs(ii).STATE.DEST = paths{nextJob(jj)}(2,:);
            amrs(ii).STATE.TASK = DRIVE;
            if length(nextJob) == 1
              nextJob = 0;
            else
              nextJob = [nextJob(1:jj-1) nextJob(jj+1:end)];
            end
          elseif amrs(ii).STATE.newJob && amrs(ii).STATE.newJobPt > 1 % New job was previously assigned and is in same lane
            amrs(ii).STATE.currPath = amrs(ii).STATE.newJob; amrs(ii).STATE.newJob = 0;
            amrs(ii).STATE.pathPt = amrs(ii).STATE.newJobPt; amrs(ii).STATE.newJobPt = 0;
            amrs(ii).STATE.DEST = paths{amrs(ii).STATE.currPath}(amrs(ii).STATE.pathPt,:);
            amrs(ii).STATE.TASK = DRIVE;
            amrs(ii).STATE.BACKWARDS = 0;
          else
            amrs(ii).STATE.pathPt = amrs(ii).STATE.pathPt - 1;
            amrs(ii).STATE.DEST = paths{amrs(ii).STATE.currPath}(amrs(ii).STATE.pathPt,:);
          end
        elseif amrs(ii).STATE.pathPt == 1  % At beginning of machine path
          if amrs(ii).STATE.newJob % Previously assigned job
            amrs(ii).STATE.currPath = amrs(ii).STATE.newJob; amrs(ii).STATE.newJob = 0;
            amrs(ii).STATE.pathPt = amrs(ii).STATE.newJobPt; amrs(ii).STATE.newJobPt = 0;
            amrs(ii).STATE.DEST = paths{amrs(ii).STATE.currPath}(amrs(ii).STATE.pathPt,:);
            amrs(ii).STATE.TASK = DRIVE;
            amrs(ii).STATE.BACKWARDS = 0;
          else
            amrs(ii).STATE.DEST = amrs(ii).STATE.HOME;
            amrs(ii).STATE.endHeadingFlag = 1;
            amrs(ii).STATE.endHeading = 0;
            amrs(ii).STATE.pathPt = 0;
          end
        else  % Must be at home
          amrs(ii).STATE.TASK = WAIT;
          amrs(ii).STATE.BACKWARDS = 0;
          amrs(ii).STATE.endHeadingFlag = 0;
        end
      end
      amrs(ii).STATE.endPose = 0;
    end
  end
  else
    pause(0.01) % Needed for GUI Pause button to work
  end
  if SIMTIME && ttime > SIMTIME % End simulation after certain number of seconds
    break
  end
end

idleSum = 0;
numJobs = [];
for ii = 1:length(machs)
  idleSum += sum(machs(ii).idleTimes);
end
for ii = 1:length(amrs)
  numJobs(ii) = amrs(ii).STATE.numJobs;
end
allocJobs = singleAllocJobs + multiAllocJobs;
