function Tracks = TrackManagement(Tracks, selected_Idx , N_tent ,M_tent, N_conf, M_conf)
Track_Need_Delete = [];
for i = 1:length(Tracks)
    % 1. length?
    % 2. if len>4 - > window=4 (sliding window)
    % 3. win (size=4)
    % 4. ten? confirm? 
    if (Tracks(i).Status == "Tentative")
        if ~isempty(selected_Idx)
            assosuccess = 1;
            Tracks(i).assodata_Tentative = [Tracks(i).assodata_Tentative, assosuccess];
        else
            assosuccess = 0;
            Tracks(i).assodata_Tentative = [Tracks(i).assodata_Tentative, assosuccess];
        end
        

        if length(Tracks(i).assodata_Tentative) > N_tent
            Tracks(i).assodata_Tentative(1) = [];
        end

        if length(Tracks(i).assodata_Tentative) == N_tent
            if sum(Tracks(i).assodata_Tentative) >= M_tent 
                Tracks(i).Status = 'Confirmed';
                Tracks(i).assodata_Confirm = zeros(1, N_conf);
                Tracks(i).assodata_Tentative = [];
            elseif sum(Tracks(i).assodata_Tentative) == 0 
                Track_Need_Delete = [Track_Need_Delete;i];
            end
        end


    elseif (Tracks(i).Status == "Confirmed")
        if ~isempty(selected_Idx)

            assosuccess = 1;
            Tracks(i).assodata_Confirm = [Tracks(i).assodata_Confirm, assosuccess];
        else
            assosuccess = 0;
            Tracks(i).assodata_Confirm = [Tracks(i).assodata_Confirm, assosuccess];
            
        end

        if length(Tracks(i).assocHistoryConf) > N_conf
                Tracks(i).assodata_Confirm(1) = [];
        end
        if length(Tracks(i).assocHistoryConf) == N_conf

            if sum(Tracks(i).assodata_Confirm) < M_conf
                Track_Need_Delete = [Track_Need_Delete, i];
            end

        end

    end
    
end

 if ~isempty(Track_Need_Delete)
   Tracks(Track_Need_Delete) = [];
 end


end

