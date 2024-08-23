%% Initialization data for simulations
%% E. Halbach   2023
AUTOPLACE = 1;
if SIM == 1
  %% Machine specifications:
  MM = [4 1]; ms = [2 2];  % Machine layout array, machine size
  MMtypes = [1 1 1 1];
  corner = [5 3]; dx = 5; dy = 4; % Bottom-left machine, x and y spacing
  %% Agent specifications:
  rtypes = [1 1]; % Robot/agent types (1-AMR, 9-human)
  rcaps = [1 1]; % Agent capabilities
  rlocs = [5 3; 10 3]; % Initial robot locations
  speeds = [1 1];

elseif SIM == 2
  %% Machine specifications:
  MM = [4 3]; ms = [2 2]; % Machine layout array
  MMtypes = [1 1 1 1 1 1 1 1 1 1 1 1];
  corner = [5 3]; dx = 5; dy = 4; % Bottom-left machine, x and y spacing
  %% Agent specifications:
  rtypes = [1 1 1 1]; % Robot/agent types (1-AMR, 9-human)
  rcaps = [1 1 1 1]; % Agent capabilities
  rlocs = [5 3; 10 3; 15 3; 20 3]; % Initial robot locations
  speeds = [1 1 1 1];

elseif SIM == 3
  %% Machine specifications:
  MM = [4 3]; ms = [2 2];  % Machine layout array
  MMtypes = [1 1 1 1 1 1 1 1 1 1 1 1];
  corner = [5 3]; dx = 5; dy = 4; % Bottom-left machine, x and y spacing
  %% Agent specifications:
  rtypes = [1 1 1 1 9 9]; % Robot/agent types (1-AMR, 9-human)
  rcaps = [1 1 1 1 1 1]; % Agent capabilities
  rlocs = [5 3; 10 3; 15 3; 20 3; 5 3; 15 3]; % Initial robot locations
  speeds = [1 1 1 1 1 1];

elseif SIM == 4
  %% Machine specifications:
  MM = [4 3]; ms = [2 2];  % Machine layout array
  MMtypes = [1 2 3 1 2 3 1 2 3 1 2 3];
  corner = [5 3]; dx = 5; dy = 4; % Bottom-left machine, x and y spacing
  %% Agent specifications:
  rtypes = [1 1 1 1 9 9]; % Robot/agent types (1-AMR, 9-human)
  rcaps = [ 1 0 1 1 1 1; % Agent capabilities
            1 1 0 1 1 1;
            0 1 1 1 1 1 ];
  rlocs = [5 3; 10 3; 15 3; 20 3; 5 3; 15 3]; % Initial robot locations
  speeds = [1 1 1 1 1 1];
%  homeFree = [0 0 0 0];

elseif SIM == 5
  %% Machine specifications:
  MM = [4 3]; ms = [2 2];  % Machine layout array
  MMtypes = [1 2 3 1 2 3 1 2 3 1 2 3];
  corner = [5 3]; dx = 5; dy = 4; % Bottom-left machine, x and y spacing
  %% Agent specifications:
  rtypes = [1 1 1 1 9 9]; % Robot/agent types (1-AMR, 9-human)
  rcaps = [ 1 0 1 1 1 1; % Agent capabilities
            1 1 0 1 1 1;
            0 1 1 1 1 1 ];
  rlocs = [5 3; 10 3; 15 3; 20 3; 5 3; 15 3]; % Initial robot locations
  speeds = [1 1 1 1 1 1];

elseif SIM == 55
  %% Machine specifications:
  MM = [4 3]; ms = [2 2];  % Machine layout array; machine size
  MMtypes = [1 2 3 1 2 3 1 2 3 1 2 3];
  corner = [5 3]; dx = 5; dy = 4; % Bottom-left machine, x and y spacing
  %% Randomly generated initial machine work times, for same init. cond. when comparing:
%  MMinit = [0.756026 0.060831 0.147487 0.080489 0.111295 0.445268 0.029450 0.158174 0.834493 0.459277 0.055504 0.177340];
  %% Agent specifications:
  rtypes = [1 1 1 1 1 1 1];% 9 9]; % Robot/agent types (1-AMR, 9-human)
  rcaps = [ 1 0 0 1 0 1 1; % Agent capabilities
            0 1 0 1 1 0 1;
            0 0 1 0 1 1 1];
  rlocs = [6.5 0]+[0 3; 2 3; 4 3; 6 3; 8 3; 10 3; 12 3];%; 20 3; 5 3]; % Initial robot locations
  speeds = [1 1 1 1 1 1 1];

elseif SIM == 56
  %% Machine specifications:
  MM = [4 3]; ms = [2 2];  % Machine layout array; machine size
%  MMtypes = [1 1 1 1 1 2 1 2 2 1 2 3];
  MMtypes = [1 2 3 1 2 3 1 2 3 1 2 3];
  corner = [5 3]; dx = 5; dy = 4; % Bottom-left machine, x and y spacing
  %% Agent specifications:
  rtypes = [1 1 1 1 1 1 1 1]; % Robot/agent types (1-AMR, 9-human)
  rcaps = [ 1 1 1 1 1 1 1 1; % Agent capabilities
            0 0 0 0 1 1 1 1;
            0 0 0 0 0 0 1 1];
  rlocs = [5 3; 10 3; 15 3; 20 3; 5 3; 10 3; 15 3; 20 3];%; 5 3]; % Initial robot locations
  speeds = [1 1 1 1 1 1 1 1];

elseif SIM == 57
  %% Machine specifications:
  MM = [4 3]; ms = [2 2];  % Machine layout array; machine size
%  MMtypes = [1 1 1 1 1 2 1 2 2 1 2 3];
  MMtypes = [1 2 3 1 2 3 1 2 3 1 2 3];
  corner = [5 3]; dx = 5; dy = 4; % Bottom-left machine, x and y spacing
  %% Agent specifications:
  rtypes = [1 1 1 1 1 1 1 1]; % Robot/agent types (1-AMR, 9-human)
  rcaps = [ 1 0 0 1 0 1 1 1; % Agent capabilities
            0 1 0 1 1 0 1 1;
            0 0 1 0 1 1 1 1];
  rlocs = [5 3; 10 3; 15 3; 20 3; 5 3; 10 3; 15 3; 20 3];%; 5 3]; % Initial robot locations
  speeds = [1 1 1 1 1 1 1 1];

elseif SIM == 58
  %% Machine specifications:
  MM = [4 3]; ms = [2 2];  % Machine layout array; machine size
  MMtypes = [1 2 3 1 2 3 1 2 3 1 2 3];
  corner = [5 3]; dx = 5; dy = 4; % Bottom-left machine, x and y spacing
  %% Agent specifications:
  rtypes = [1 1 1 1 1 1];% 9 9]; % Robot/agent types (1-AMR, 9-human)
  rcaps = [ 1 0 1 1 0 1; % Agent capabilities
            1 1 0 1 1 0;
            0 1 1 0 1 1];
  rlocs = [5 3; 10 3; 15 3; 20 3; 5 3; 10 3];%; 20 3; 5 3]; % Initial robot locations
  speeds = [1 1 1 1 1 1];

elseif SIM == 555
  %% Machine specifications:
  MM = [5 4]; ms = [2 2];  % Machine layout array; machine size
  MMtypes = [1 2 3 4 1 2 3 4 1 2 3 4 1 2 3 4 1 2 3 4];
  corner = [5 3]; dx = 5; dy = 4; % Bottom-left machine, x and y spacing
  %% Agent specifications:
  rtypes = [1 1 1 1 1 1 1 1 9]; % Robot/agent types (1-AMR, 9-human)
  rcaps = [ 1 0 0 0 1 0 0 1 1; % Agent capabilities
            0 1 0 0 1 1 0 0 1;
            0 0 1 0 0 1 1 0 1;
            0 0 0 1 0 0 1 1 1];
  rlocs = [5 3; 10 3; 15 3; 20 3; 25 3; 5 3; 10 3; 15 3; 20 3]; %; 25 3]; % Initial robot locations
  speeds = [1 1 1 1 1 1 1 1 1];

elseif SIM == 6
  %% Machine specifications:
  MM = [4 3]; ms = [2 2];  % Machine layout array
  MMtypes = [1 2 3 1 2 3 1 2 3 1 2 3];
  corner = [4 3]; dx = 5; dy = 4; % Bottom-left machine, x and y spacing
  %% Agent specifications:
  rtypes = [1 1 1 1]; % Robot/agent types (1-AMR, 9-human)
  rcaps = [ 1 0 0 1; % Agent capabilities
            0 1 0 1;
            0 0 1 1 ];
  rlocs = corner + [0 0; dx 0; 2*dx 0; 3*dx 0]; % Initial robot locations
  speeds = [0.5 0.5 0.5 1];

elseif SIM == 7
  %% Machine specifications:
  MM = [4 2]; ms = [2 2];  % Machine layout array
  MMtypes = [1 2 1 2 1 2 1 2];
  corner = [4 3]; dx = 5; dy = 4; % Bottom-left machine, x and y spacing
  %% Agent specifications:
  rtypes = [1 1 1 1]; % Robot/agent types (1-AMR, 9-human)
  rcaps = [ 1 0 1 1; % Agent capabilities
            0 1 1 1 ];
  rlocs = corner + [0 0; dx 0; 2*dx 0; 3*dx 0]; % Initial robot locations
  speeds = [0.5 0.5 1 1];

elseif SIM == 8
  %% Machine specifications:
  MM = [3 3]; ms = [2 2];  % Machine layout array
  MMtypes = [1 2 3 1 2 3 1 2 3];
  corner = [4 3]; dx = 5; dy = 4; % Bottom-left machine, x and y spacing
  %% Agent specifications:
  rtypes = [1 1 1]; % Robot/agent types (1-AMR, 9-human)
  rcaps = [ 1 0 1; % Agent capabilities
            1 1 0;
            0 1 1 ];
  rlocs = corner + [0 0; dx 0; 2*dx 0]; % Initial robot locations
  speeds = [0.5 0.5 0.5];

elseif SIM == 9 % Printer room 2x4 layout
  AUTOPLACE = 0;
  wallsX = [1.5 5 5 1.5 1.5]; wallsY = [7 7 12 12 7]; % For plotting walls
  %% Machine specifications:
  MM = [2 4]; ms = [0.5 0.5];  % Machine layout array
  fx = 2200; fy = 200; fw = 150 + MM(1)*250; fh = 300 + MM(2)*150; % Figure window specs
  MMtypes = [1 1 1 1 1 1 1 1];
  corner = [2.7 8.5]; dx = 1.2; dy = 0.7; % Bottom-left machine, x and y spacing
  mlocs = [2.7 9.2; 2.7 9.9; 2.7 10.6; 2.7 11.2; 3.9 9.2; 3.9 9.9; 3.9 10.6; 3.9 11.2];
  mlocs(1:4,1) -= ms(1)/2;  mlocs(5:8,1) += ms(1)/2;
  paths = { [3.3 8.5; 3.3 9.2; 2.7 9.2] ; [3.3 8.5; 3.3 9.9; 2.7 9.9] ; [3.3 8.5; 3.3 10.6; 2.7 10.6] ; [3.3 8.5; 3.3 11.2; 2.7 11.2] ;
            [3.3 8.5; 3.3 9.2; 3.9 9.2] ; [3.3 8.5; 3.3 9.9; 3.9 9.9] ; [3.3 8.5; 3.3 10.6; 3.9 10.6] ; [3.3 8.5; 3.3 11.2; 3.9 11.2] };
  %% Agent specifications:
  rtypes = [1 1]; % Robot/agent types (1-AMR, 9-human)
  rcaps = [ 1 1; % Agent capabilities
            1 1 ];
  rlocs = [2.7 8.5; 3.9 8.5]; % Initial robot locations
  speeds = [0.5 0.5];

elseif SIM == 10 % Lab 4x3 layout
  AUTOPLACE = 0;
  wallsX = [14 26 26 14 14]; wallsY = [5 5 12 12 4]; % For plotting walls
  %% Machine specifications:
  MM = [4 3]; ms = [0.5 0.5];  % Machine layout array
  fx = 2200; fy = 200; fw = 150 + MM(1)*250; fh = 300 + MM(2)*150; % Figure window specs
  MMtypes = [1 2 3 1 2 3 1 2 3 1 2 3];
  corner = [1 1]; dx = 1.2; dy = 0.7; % Bottom-left machine, x and y spacing
  mlocs = [15.6 9.2; 15.6 10.2; 15.6 11.2; 16.8 9.2; 16.8 10.2; 16.8 11.2; 22.9 9.2; 22.9 10.2; 22.9 11.2; 24.1 9.2; 24.1 10.2; 24.1 11.2];
  %MMinit = [0.756026 0.060831 0.147487 0.080489 0.111295 0.445268 0.029450 0.158174 0.834493 0.459277 0.055504 0.177340];
  %mlocs(1:4,1) -= ms(1)/2;  mlocs(5:8,1) += ms(1)/2;
  paths = { [17.4 6.2; 17.4 8.4; 16.2 8.4; 16.2 9.2; 15.6 9.2] ; [17.4 6.2; 17.4 8.4; 16.2 8.4; 16.2 10.2; 15.6 10.2] ; [17.4 6.2; 17.4 8.4; 16.2 8.4; 16.2 11.2; 15.6 11.2] ;
            [17.4 6.2; 17.4 9.2; 16.8 9.2] ; [17.4 6.2; 17.4 10.2; 16.8 10.2] ; [17.4 6.2; 17.4 11.2; 16.8 11.2] ;
            [24.7 6.2; 24.7 8.4; 23.5 8.4; 23.5 9.2; 22.9 9.2] ; [24.7 6.2; 24.7 8.4; 23.5 8.4; 23.5 10.2; 22.9 10.2] ; [24.7 6.2; 24.7 8.4; 23.5 8.4; 23.5 11.2; 22.9 11.2] ;
            [24.7 6.2; 24.7 9.2; 24.1 9.2] ; [24.7 6.2; 24.7 10.2; 24.1 10.2] ; [24.7 6.2; 24.7 11.2; 24.1 11.2] };
  %% Agent specifications:
  rtypes = [1 1 1 1 1 1 1 1]; % Robot/agent types (1-AMR, 9-human)

  rcaps =  [1 0 1 0 1 0 1 1; % Agent capabilities
            1 1 1 0 0 1 0 1;
            1 1 0 1 0 0 1 1];

##  rcaps = [1 1 1 1 0 1
##           1 1 0 1 1 1
##           0 1 1 1 1 1];
  rlocs = [17.55 6.2; 18.55 6.2; 19.55 6.2; 20.55 6.2; 21.55 6.2; 22.55 6.2; 23.55 6.2;24.55 6.2]; % Initial robot locations
  speeds = [0.2 0.2 0.2 0.2 0.2 0.2 0.2 0.2];

end

numR = length(rtypes); % Number of agents
%% Generate machine locations and paths:
numM = MM(1)*MM(2); % Number of machines
if AUTOPLACE
  fx = 2200; fy = 200; fw = 50+MM(1)*250; fh = MM(2)*150+300; % Figure window specs
  wallsX = [corner(1)-dx/2 (corner(1)+(MM(1)-1)*dx+dx)*[1 1] (corner(1)-dx/2)*[1 1]]; wallsY = [(corner(2)-2)*[1 1] (corner(2)+MM(2)*dy+dy)*[1 1] corner(2)-2]; % For plotting walls
  mlocs = zeros(numM,2);
  for ii = 1:MM(1)
    for jj = 1:MM(2)
      mlocs((ii-1)*MM(2)+jj,:) = corner + [2 4] + [(ii-1)*dx (jj-1)*dy];
      paths{(ii-1)*MM(2)+jj} = corner + [(ii-1)*dx 0; (ii-1)*dx dy+(jj-1)*dy; (ii-1)*dx+ms(1)/2 dy+(jj-1)*dy];
    end
  end
end
