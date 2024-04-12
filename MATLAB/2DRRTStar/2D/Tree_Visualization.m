function  p1 = Tree_Visualization(Nodes,Edges)
    a = size(Edges);

    for i = 1:a(1)
        
        p1 = plot([Nodes(Edges(i,1),2),Nodes(Edges(i,2),2)],[Nodes(Edges(i,1),1),Nodes(Edges(i,2),1)],'r');
        hold on
    end
   % scatter(Nodes(:,2),Nodes(:,1))

end

