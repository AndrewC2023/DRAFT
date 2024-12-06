/*
 * Author: Andrew Campbell
 * Date: 12-03-2024
 */

#include "2DVisualizer.hpp"

namespace Visualization
{
    Visualizer2D::Visualizer2D(std::shared_ptr<Algorithms::TwoD::GridManager2D> grid) : _grid(std::move(grid))
    {
        auto _figure = matplot::figure();
        matplot::figure(_figure);
    }

    void Visualizer2D::plotGrid()
    {
        _grid->getGridDomain(xMin, xMax, yMin, yMax); // sets them by reference
        float cellSize = _grid->getCellSize();

        int numXcolumns = static_cast<int>((xMax - xMin) / cellSize);
        int numYrows = static_cast<int>((yMax - yMin) / cellSize);

        std::vector<std::vector<int>> occupancyGrid;

        for(auto& cell : _grid->getCells())
        {
            auto index = cell.getIndex();
            if(cell.getState() == State::OBSTACLE)
            {
                occupancyGrid[index.y][index.x] = 1;
            } else {
                occupancyGrid[index.y][index.x] = 0;
            }
        }

        matplot::imagesc(occupancyGrid);
        matplot::colormap(matplot::palette::greys());

    }

    void Visualizer2D::plotPath(std::deque<Algorithms::TwoD::PointXY>& path)
    {

        std::vector<float> X;
        std::vector<float> Y;

        for(auto& point : path)
        {
            X.push_back(convert2IndexSpace(point).x);
            Y.push_back(convert2IndexSpace(point).y);
        }

        matplot::plot(X,Y);

    }

    void Visualizer2D::plotTree(std::vector<Algorithms::TwoD::RRTStarNode>& tree)
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

    void Visualizer2D::show()
    {

        matplot::show();
    }

}