#ifndef GRAHPICSMANAGER_HPP
#define GRAHPICSMANAGER_HPP

/**
 * @file graphics_managerh
 * @author {MehdiBelal} ({Mehdi.Belal@tii.ae})
 * @brief TOOD
 * @date 2020-01-31
 */

#include <arrc/coverage/Graphics.h>

template <typename T>
struct Gaussian
{
    Vector2<T> mean;
    T var;

    Gaussian(Vector2<T> mean, T var) : mean(mean), var(var) { }
};

template <typename T = double>
class GraphicsManager
{
public:
    GraphicsManager(T area_size_x, T area_size_y, T area_left, T area_bottom, T var)
    {
        m_graphics = new Graphics{area_size_x, area_size_y, area_left, area_bottom, var};
        m_graphics->clear();
    }

    /**
     * @brief drawDiagram - method used to draw a Voronoi Diagram into the window, for how it is
     *                      currently strucutured, it must be the first methods that has to be called
     *                      for a drawing since it is the methods that calls the clear() to "refresh"
     *                      before the new iteration
     *
     * @param diagram - Diagram<T>, diagram to draw
     * @param gaussians - std::vector<Gaussian<T>>, eventual list of gaussians to add to the draw
     */
    void drawDiagram(Diagram<T> diagram, std::vector<Gaussian<T>> gaussians = std::vector<Gaussian<T>>())
    {
        m_graphics->clear();
        m_graphics->drawDiagram(diagram);
        m_graphics->drawPoints(diagram);
        m_graphics->drawGlobalReference(sf::Color(255,255,0), sf::Color(255,255,255));
        if(gaussians.size() != 0) { drawGaussians(gaussians); }
        m_graphics->display();
    }

private:
    Graphics* m_graphics = nullptr;

    /**
     * @brief drawGaussians - private method used to draw a set of gaussians into the space
     * @param gaussians - std::vector<Gaussian<T>>, list of gaussians to draw
     */
    void drawGaussians(std::vector<Gaussian<T>> gaussians)
    {
        std::vector<Vector2<T> > _means;
        std::vector<T> _vars;
        for (auto _g : gaussians)
        {
            _means.push_back(_g.mean);
            _vars.push_back(_g.var);
        }
        m_graphics->drawGaussianContours(_means, _vars);
    }

};

template class Gaussian<double>;
template class GraphicsManager<double>;

#endif // GRAHPICSMANAGER_HPP
