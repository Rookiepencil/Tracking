function [Tracks, deletedTracks] = TrackManagement(Tracks, N_tent, M_tent, N_conf, M_conf)

    delIdx = [];  

    for i = 1:length(Tracks)
        
        % add current assocation result to assoHistory
        Tracks(i).assodata_Tentative = [Tracks(i).assodata_Tentative, Tracks(i).assocResultCurrent];
        
        if Tracks(i).Status == "Tentative"
            % if length is smaller than tentative window size do nothing
            % just skip
            if length(Tracks(i).assodata_Tentative) < N_tent
                continue;
            end

            % get asso history in the window
            tentativeWindow = Tracks(i).assodata_Tentative(end-N_tent+1:end);

            if sum(tentativeWindow) >= M_tent % here need to optimized becasue if you have three in a row just confirm you do not need wait 
                Tracks(i).Status = "Confirmed"; 

            elseif sum(tentativeWindow) < M_tent
                delIdx = [delIdx, i];
                Track_Need_Delete = [Track_Need_Delete;i];
            end

        elseif Tracks(i).Status == "Confirmed"
            if length(Tracks(i).assodata_Tentative) < N_conf
                continue;
            end
            
            confirmedWindow = Tracks(i).assodata_Tentative(end-N_conf+1:end);

            if sum(confirmedWindow) < M_conf
                delIdx = [delIdx, i];
                Track_Need_Delete = [Track_Need_Delete;i];
            end
        end
    end

     if ~isempty(Track_Need_Delete)
       deletedTracks = [deletedTracks,Tracks(Track_Need_Delete)];
       Tracks(Track_Need_Delete) = [];
     end

end