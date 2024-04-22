classdef Visualization_Processes2D
    %VISUALIZATION_PROCESSES2D Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Start
        End
        Visualization_MAP
        Path
    end
    
    methods
        function VP2D = Cost_Map_Visualization(VP2D,G_MAP,Space_Sizes,Visited,Figure_Num)
            %   In_Progress_Cost_Visualization Construct an instance of this class
            %   Shows the Costs of the search domain
            VP2D.Visualization_MAP = G_MAP;
            for i = 1:Space_Sizes(1)
            for j = 1:Space_Sizes(2)
                if Visited(i,j) == 0
                    VP2D.Visualization_MAP(i,j) = 0;
                end
            end
            end
            figure(Figure_Num)
                title('Search Visualization')
                imagesc(VP2D.Visualization_MAP)
                a = colorbar;
                ylabel(a,'G Cost')
                hold on
        end
        
        function VP2D = Path_Visualization(VP2D,Current_Point,Parent_Node,Figure_Num)
            % Shows the current Ideal path for whatever current node is passed throuh
            i = 1;
            while (Current_Point(1) ~= VP2D.Start(1)) + (Current_Point(2) ~= VP2D.Start(2)) > 0
                VP2D.Path(i,:) = Current_Point;
                Current_Point = Parent_Node(Current_Point(1),Current_Point(2),:);
                i = i + 1;
            end
            VP2D.Path(i,:) = VP2D.Start;
            clear i
            figure(Figure_Num)
                plot(VP2D.Path(:,2),VP2D.Path(:,1),'g','LineWidth',2.5)
                hold on
                scatter([VP2D.Start(2),VP2D.End(2)],[VP2D.Start(1),VP2D.End(1)],[50,50],"filled","square",'g')
                hold off
        end

    end
end

