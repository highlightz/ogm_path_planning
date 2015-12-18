#include "ogmpp_planners/ogmpp_planner_factory.hpp"

namespace ogmpp_planners
{
  OgmppAbstractPlanner* OgmppPlannerFactory::getPlanner(std::string type)
  {
    
    if(type == "uniform_prm") return new prms::UniformSampling();
    else if(type == "random_prm") return new prms::RandomSampling();
    else if(type == "halton_prm") return new prms::HaltonSampling();
    else if(type == "cell_based_prm") return new prms::CellBasedSampling();
    return NULL;
  }

}
