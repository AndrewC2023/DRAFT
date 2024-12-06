/*
 * Author: Andrew Campbell
 * Date: 12-03-2024
 */

#include "2DVisualizer.hpp"

namespace Visualization
{

    Visualizer2D::Visualizer2D(std::shared_ptr<Algorithms::TwoD::GridManager2D>& grid) : _grid(grid)
    {
        _figure = matplot::figure();
        matplot::figure(_figure);
    }

    void Visualizer2D::plotPolygon(const std::vector<Algorithms::TwoD::PointXY>& polygon)
    {
        std::vector<float> X;
        std::vector<float> Y;

        for(const auto& point : polygon)
        {
            X.push_back(convert2IndexSpace(point).x);
            Y.push_back(convert2IndexSpace(point).y);
        } 
        // close the polygon
        X.push_back(convert2IndexSpace(polygon.at(0)).x);
        Y.push_back(convert2IndexSpace(polygon.at(0)).y);

        matplot::plot(X,Y);
        matplot::hold(true);

    }

    void Visualizer2D::plotGrid(float time)
    {
        _grid->getGridDomain(xMin, xMax, yMin, yMax); // sets them by reference
        float cellSize = _grid->getCellSize();

        int numXcolumns = static_cast<int>((xMax - xMin) / cellSize);
        int numYrows = static_cast<int>((yMax - yMin) / cellSize);

        std::vector<std::vector<int>> occupancyGrid;
        occupancyGrid.resize(numYrows, std::vector<int>(numXcolumns, 0));

        std::cout << "number for grid cells: " << _grid->getCells(time).size() << "\n";
        for(const auto& cell : _grid->getCells(time))
        {
            auto index = cell.getIndex();
            if(cell.getState() == State::OBSTACLE)
            {
                occupancyGrid.at(index.y).at(index.x) = 1;
            } else {
                occupancyGrid.at(index.y).at(index.x) = 0;
            }
        }

        // matplot::ylim({100,0});

        matplot::imagesc(occupancyGrid);
        matplot::colormap(matplot::palette::greys());
        matplot::hold(true);

    }

    void Visualizer2D::plotPath(const std::deque<Algorithms::TwoD::PointXY>& path)
    {

        std::vector<float> X;
        std::vector<float> Y;

        for(const auto& point : path)
        {
            X.push_back(convert2IndexSpace(point).x);
            Y.push_back(convert2IndexSpace(point).y);
        }

        matplot::plot(X,Y);
        matplot::hold(true);
    }

    void Visualizer2D::plotTree(const std::vector<Algorithms::TwoD::RRTStarNode>& tree)
    {

        for(int i = 0; i < tree.size() - 1; i++)
        {   
            Algorithms::TwoD::PointXY currentPosition = tree.at(i).position;
            Algorithms::TwoD::PointXY parentPosition = tree.at(tree.at(i).parentIndex).position;

            std::vector<float> X;
            std::vector<float> Y;
            
            X.push_back(convert2IndexSpace(currentPosition).x);
            Y.push_back(convert2IndexSpace(currentPosition).y);
            X.push_back(convert2IndexSpace(parentPosition).x);
            Y.push_back(convert2IndexSpace(parentPosition).y);

            matplot::plot(X,Y)->color("r");
            matplot::hold(true);
        }
    }

    const Algorithms::TwoD::PointXY Visualizer2D::convert2IndexSpace(const Algorithms::TwoD::PointXY point)
    {
        float cellSize = _grid->getCellSize();
        return Algorithms::TwoD::PointXY((point.x + 0.5 * cellSize) / cellSize, (point.y + 0.5 * cellSize) / cellSize);
    }

    const Algorithms::TwoD::PointXY Visualizer2D::convert2PointSpace(const Algorithms::TwoD::PointXY IndexPoint)
    {
        float cellSize = _grid->getCellSize();
        return Algorithms::TwoD::PointXY(IndexPoint.x * cellSize - 0.5 * cellSize, IndexPoint.y * cellSize - 0.5 * cellSize);
    }

    const Algorithms::TwoD::PointXY Visualizer2D::convert2PointSpace(const Algorithms::TwoD::IndexXY index)
    {
        float cellSize = _grid->getCellSize();
        return Algorithms::TwoD::PointXY(static_cast<float>(index.x) * cellSize - 0.5 * cellSize, static_cast<float>(index.y) * cellSize - 0.5 * cellSize);
    }

    void Visualizer2D::show(std::string title)
    {
            // todo Image sc has the Z axis pointing into the screen not out of it, fix this at some point pls or else all plots are mirrored immages of reality
        _figure->title(title);
        _figure->size(8000, 8000);

        std::cout << "Showing Figure: " << title << std::endl;
        matplot::show();
        // std::cin.get();
    }

}