classdef Visualization_Processes3D
    %VISUALIZATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Obstacle_Points
        Shades
        Start
        End
        Points_From_Center = [-0.5,-0.5,-0.5;
                               0.5,-0.5,-0.5;
                               0.5, 0.5,-0.5;
                              -0.5, 0.5,-0.5;
                              -0.5,-0.5, 0.5;
                               0.5,-0.5, 0.5;
                               0.5, 0.5, 0.5;
                              -0.5, 0.5, 0.5];
        Face_Numbers = [1,2,3,4;
                        1,2,6,5;
                        4,1,5,8;
                        2,3,7,6;
                        3,4,8,7;
                        5,6,7,8;] 
       Number_of_Faces {int32}
    end
    
    methods
        function VP3D = Obstacle_Point_Generation(VP3D,Obstacle_MAP,MAP_Sizes)
            VP3D.Number_of_Faces = 1; 
            for i = 1:MAP_Sizes(1)
            for j = 1:MAP_Sizes(2)
            for k = 1:MAP_Sizes(3)
                if Obstacle_MAP(i,j,k)
                    Prob =  (Obstacle_MAP(i,j,k))/256;
                    % We have an obstacle center
                    for q = 1:6
                        % x
                        VP3D.Obstacle_Points(:,VP3D.Number_of_Faces,1) = VP3D.Points_From_Center(VP3D.Face_Numbers(q,:),1) + i;
                        VP3D.Obstacle_Points(:,VP3D.Number_of_Faces,2) = VP3D.Points_From_Center(VP3D.Face_Numbers(q,:),2) + j;
                        VP3D.Obstacle_Points(:,VP3D.Number_of_Faces,3) = VP3D.Points_From_Center(VP3D.Face_Numbers(q,:),3) + k;
                        VP3D.Shades(1,VP3D.Number_of_Faces) = Prob;
                        VP3D.Number_of_Faces = VP3D.Number_of_Faces + 1; 
                    end
                end
            end
            end 
            end

        end
        
        function Obstacle_Visualization(VP3D,MAP_Sizes,Figure_Num)
            figure(Figure_Num)
                faces = fill3(VP3D.Obstacle_Points(:,:,1),VP3D.Obstacle_Points(:,:,2),VP3D.Obstacle_Points(:,:,3),ones(1,VP3D.Number_of_Faces - 1));
                for i = 1:(VP3D.Number_of_Faces - 1)
                  faces(i).FaceAlpha = VP3D.Shades(i);
                end
                colormap('gray')            
                axis([0, MAP_Sizes(1) + 1, 0, MAP_Sizes(2) + 1, 0, MAP_Sizes(3) + 1])
                colorbar
                hold on
        end
        
        function Path_Visualization(VP3D,Path,Figure_Num)
            figure(Figure_Num)
                plot3(Path(:,1),Path(:,2),Path(:,3),'g','LineWidth',2.5)
                hold on
                scatter3([VP3D.Start(1),VP3D.End(1)],[VP3D.Start(2),VP3D.End(2)],[VP3D.Start(3),VP3D.End(3)],[50,50],"filled","square",'g')
                hold off
        end
    end
end

