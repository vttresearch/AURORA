function Ht = SystemEntropy(rcaps,agents,agent)
%% This function calculates the remaining system entropy of available "agents" 
%% based on their capabilities "rcaps" if "agent" would be selected for a task 
%% (i.e. "agent" is not included in the calculation).
%% E. Halbach   VTT   2023

%% Get Nall
Nall = 0;
for ii = 1:length(agents)
  if ii ~= agent && agents(ii) == 1
    Nall = Nall + sum(rcaps(:,ii));
  end
end

%% Get Nj, Pj, Ht
Ht = 0;
if Nall > 0
  for jj = 1:rows(rcaps)
    Nj = 0;
    for ii = 1:columns(rcaps)
      if ii ~= agent && agents(ii) == 1 && rcaps(jj,ii) == 1
        Nj = Nj + 1;
      end
    end
    Pj = Nj/Nall;
    if Pj > 0
      Ht = Ht - Pj*log2(Pj);
    end
  end
end