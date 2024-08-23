function driveDist = DriveDist(home, path)
%% This function returns the driving distance from an AMR's HOME position to the
%% end of the path to a machine. Part of the AURORA time entropy simulation.
%% E. Halbach   VTT   2022

driveDist = norm( path(1,:) - home );
for ii = 2:rows(path)
  driveDist = driveDist + norm( path(ii,:) - path(ii-1,:) );
end
