#include "ogmpp_planners/ogmpp_planner_factory.hpp"

#include "ogmpp_planners/ogmpp_prms/ogmpp_uniform_sampling.hpp"
#include "ogmpp_planners/ogmpp_prms/ogmpp_random_sampling.hpp"
#include "ogmpp_planners/ogmpp_prms/ogmpp_halton_sampling.hpp"
#include "ogmpp_planners/ogmpp_prms/ogmpp_cell_based_sampling.hpp"
#include "ogmpp_planners/ogmpp_prms/ogmpp_random_halton_sampling.hpp"
#include "ogmpp_planners/ogmpp_prms/ogmpp_gaussian_sampling.hpp"
#include "ogmpp_planners/ogmpp_prms/ogmpp_obstacle_based_sampling.hpp"

namespace ogmpp_planners
{
  OgmppAbstractPlanner* OgmppPlannerFactory::getPlanner(std::string type)
  {
    
    if(type == "uniform_prm") return new prms::UniformSampling();
    else if(type == "random_prm") return new prms::RandomSampling();
    else if(type == "halton_prm") return new prms::HaltonSampling();
    else if(type == "cell_based_prm") return new prms::CellBasedSampling();
    else if(type == "random_halton_prm") return new prms::RandomHaltonSampling();
    else if(type == "gaussian_prm") return new prms::GaussianSampling();
    else if(type == "obstacle_based_prm") return new prms::ObstacleBasedSampling();
    return NULL;
  }

}
