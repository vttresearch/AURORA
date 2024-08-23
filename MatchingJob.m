function match = MatchingJob(jobs,MMtypes,rcaps,agent)
%% This function checks if any of the jobs in the queue match the capabilities 
%% of the agent.
%% E. Halbach   VTT   2023

match = 0;

if jobs(1) > 0
  for ii = 1:length(jobs)
    if rcaps(MMtypes(jobs(ii)),agent)   % rcaps(MMtypes(nextJob(1)),ii)
      match = jobs(ii);
      return
    end
  end
end