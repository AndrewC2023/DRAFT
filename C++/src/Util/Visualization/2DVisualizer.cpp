/*
 * Author: Andrew Campbell
 * Date: 12-03-2024
 */

#include "2DVisualizer.hpp"
#include <omp.h>

// TODO: Add a method to plot a single point
// TODO: allow grind to get plotted on real space axes
//      - likely willl need a lot of post processing to graph the grid in real space
namespace Visualization
{

    Visualizer2D::Visualizer2D()
    {
        _figure.clear();
        _figure.push_back(matplot::figure());
        matplot::figure(_figure.at(0));
        _cellSize = -1;
    } // constructor

    /**
     * @brief Plots a 2D polygon on a graph.
     * 
     * This function takes a vector of 2D points representing the vertices of a polygon,
     * converts them to index space, and plots the polygon using the matplot library.
     * The polygon is closed by connecting the last point to the first point.
     * 
     * @param polygon A vector of Points representing the vertices of the polygon.
     */
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

    } // plotPolygon

    void Visualizer2D::newFigure()
    {
        _figure.push_back(matplot::figure());
        matplot::figure(_figure.at(_figure.size() - 1));
    } // newFigure

    void Visualizer2D::setFigure(int index)
    {
        if(index < _figure.size())
        {
            matplot::figure(_figure.at(index));
        } else {
            throw std::runtime_error("Invalid figure index");
        }
    } // setFigure

    /**
     * @brief Plots a 2D grid visualization based on the occupancy state of cells at a given time.
     * 
     * This function retrieves the grid domain boundaries and cell size, calculates the number of columns and rows,
     * and initializes an occupancy grid. It then populates the occupancy grid based on the state of each cell 
     * (either obstacle or free space) at the specified time. Finally, it uses the Matplot++ library to visualize 
     * the occupancy grid with a grayscale colormap.
     * 
     * @param time The time at which to retrieve the cell states for plotting.
     * 
     * @note this method uses the grey color scheme but this works well only if the grid is binary
     */
    void Visualizer2D::plotGrid(std::shared_ptr<Algorithms::TwoD::GridManager2D>& grid)
    {
        grid->getGridDomain(xMin, xMax, yMin, yMax); // sets them by reference
        _cellSize = grid->getCellSize();

        int numXcolumns = static_cast<int>((xMax - xMin) / _cellSize);
        int numYrows = static_cast<int>((yMax - yMin) / _cellSize);

        std::vector<std::vector<int>> occupancyGrid;
        occupancyGrid.resize(numYrows, std::vector<int>(numXcolumns, 0));

        for(const auto& cell : grid->getCells(0.0))
        {
            auto index = cell.getIndex();
            if(cell.getState() == State::OBSTACLE)
            {
                occupancyGrid.at(index.y).at(index.x) = 1;
            } else if(cell.getState() == State::UNCERTAIN)
            {
                occupancyGrid.at(index.y).at(index.x) = cell.getOdds();
            } else {
                occupancyGrid.at(index.y).at(index.x) = 0;
            }
        }

        // matplot::ylim({100,0});

        matplot::imagesc(occupancyGrid);
        matplot::colorbar().label("Occupancy Probability");
        matplot::hold(true);

    } // plotGrid

    void Visualizer2D::plotGrid(std::shared_ptr<Algorithms::TwoD::HybridGridManager2D>& grid, float time)
    {
        grid->getGridDomain(xMin, xMax, yMin, yMax); // sets them by reference
        _cellSize = grid->getCellSize();

        int numXcolumns = static_cast<int>((xMax - xMin) / _cellSize);
        int numYrows = static_cast<int>((yMax - yMin) / _cellSize);

        std::vector<std::vector<float>> occupancyGrid;
        occupancyGrid.resize(numYrows, std::vector<float>(numXcolumns, 0));

        for(const auto& cell : grid->getCells(time))
        {
            auto index = cell.getIndex();
            if(cell.getState() == State::OBSTACLE)
            {
                occupancyGrid.at(index.y).at(index.x) = 1;
                //std::cout << "Cell odds: " << cell.getOdds() << std::endl;
            } else if(cell.getState() == State::UNCERTAIN)
            {
                occupancyGrid.at(index.y).at(index.x) = cell.getOdds();
            } else {
                occupancyGrid.at(index.y).at(index.x) = 0;
            }
        }

        // matplot::ylim({100,0});

        matplot::imagesc(occupancyGrid);
        matplot::colorbar().label("Occupancy Probability");
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

        matplot::plot(X,Y)->color("g").line_width(4);

        std::vector<double> X_SG;
        std::vector<double> Y_SG;
        X_SG.push_back(static_cast<double>(X[0]));
        X_SG.push_back(static_cast<double>(X[X.size() - 1]));
        Y_SG.push_back(static_cast<double>(Y[0]));
        Y_SG.push_back(static_cast<double>(Y[Y.size() - 1]));

        auto scatter_plot = matplot::scatter(X_SG, Y_SG);
        scatter_plot->marker("o");       // Set marker style
        scatter_plot->marker_size(10);   // Set marker size
        scatter_plot->color("g");        // Set color to green
       

        matplot::text(X[0],Y[0] + 1,"Start")->color("black").font_size(20);
        matplot::text(X[X.size() - 1],Y[Y.size() - 1] + 1,"Goal")->color("black").font_size(20);
        matplot::hold(true);
    } // plotPath

    [[gnu::hot]] void Visualizer2D::plotTree(const std::vector<Algorithms::TwoD::RRTStarNode>& tree)
    {
        #pragma omp parallel for
        std::vector<float> allX, allY;
        for (int i = 0; i < tree.size() - 1; i++) 
        {

            Algorithms::TwoD::PointXY currentPosition = tree.at(i).position;
            Algorithms::TwoD::PointXY parentPosition = tree.at(tree.at(i).parentIndex).position;

            // Add the current position and its parent's position
            allX.push_back(convert2IndexSpace(currentPosition).x);
            allY.push_back(convert2IndexSpace(currentPosition).y);

            allX.push_back(convert2IndexSpace(parentPosition).x);
            allY.push_back(convert2IndexSpace(parentPosition).y);

            // Add NaN to disconnect the line after plotting each segment
            allX.push_back(std::numeric_limits<float>::quiet_NaN());
            allY.push_back(std::numeric_limits<float>::quiet_NaN());
        }

        // Plot the entire batch in one command
        matplot::plot(allX, allY)->color("r").line_width(3);
    } // plotTree

    /**
     * @brief Converts a point from world space to index space.
     *
     * @param point The point in world coordinates to be converted.
     * @return The converted point in index coordinates.
     */
    const Algorithms::TwoD::PointXY Visualizer2D::convert2IndexSpace(const Algorithms::TwoD::PointXY point)
    {
        if(_cellSize == -1)
        {
            throw std::runtime_error("Cell size not set");
        }
        return Algorithms::TwoD::PointXY((point.x + 0.5 * _cellSize) / _cellSize, (point.y + 0.5 * _cellSize) / _cellSize);
    } // convert2IndexSpace

    /**
     * @brief Converts a point from index space to point space.
     * 
     * @param IndexPoint The point in index space to be converted.
     * @return The converted point in world coordinates.
     */
    const Algorithms::TwoD::PointXY Visualizer2D::convert2PointSpace(const Algorithms::TwoD::PointXY IndexPoint)
    {
        if(_cellSize == -1)
        {
            throw std::runtime_error("Cell size not set");
        }
        return Algorithms::TwoD::PointXY(IndexPoint.x * _cellSize - 0.5 * _cellSize, IndexPoint.y * _cellSize - 0.5 * _cellSize);
    } // convert2PointSpace

    /**
     * @brief Converts a grid index to a point in 2D space.
     *
     * @param index The grid index to be converted, represented as an IndexXY object.
     * @return A PointXY object representing the corresponding point in 2D space.
     */
    const Algorithms::TwoD::PointXY Visualizer2D::convert2PointSpace(const Algorithms::TwoD::IndexXY index)
    {
        if(_cellSize == -1)
        {
            throw std::runtime_error("Cell size not set");
        }
        return Algorithms::TwoD::PointXY(static_cast<float>(index.x) * _cellSize - 0.5 * _cellSize, static_cast<float>(index.y) * _cellSize - 0.5 * _cellSize);
    } // convert2PointSpace

    void Visualizer2D::show(std::string title)
    {
            // todo Image sc has the Z axis pointing into the screen not out of it, fix this at some point pls or else all plots are mirrored immages of reality
        matplot::gcf()->title(title);
        // Negative value moves title down

        matplot::gcf()->position(2000,1000,2000,2000);
        
        matplot::gca()->x_axis().label_font_size(20);    // X axis font size
        matplot::gca()->y_axis().label_font_size(20);    // Y axis font size
        // matplot::gca()->x_axis(); // X tick labels
        // matplot::gca()->y_axis(); // Y tick label
        matplot::gca()->x_axis().label("X");
        matplot::gca()->y_axis().label("Y");
        matplot::gca()->y_axis().reverse(false);

        std::cout << "Showing Figure: " << title << std::endl;
        matplot::show();
        // std::cin.get();
    } // show

}