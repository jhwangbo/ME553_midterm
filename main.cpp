#include <iostream>
#include "raisim/RaisimServer.hpp"

#define MAKE_STR(x) _MAKE_STR(x)
#define _MAKE_STR(x) #x

int main() {
    std::string rsc_dir = std::string(MAKE_STR(RSC_DIR));
    /// create raisim world
    raisim::World world;
    world.setTimeStep(0.001);

    /// create objects
    world.addGround();
    auto cartPole = world.addArticulatedSystem(raisim::Path(rsc_dir + "\\doubleCartPole\\doubleCartPole.urdf").getString());
    cartPole->setIntegrationScheme(raisim::ArticulatedSystem::IntegrationScheme::RUNGE_KUTTA_4);

    /// cartPole state
    Eigen::VectorXd jointNominalConfig(cartPole->getGeneralizedCoordinateDim());
    jointNominalConfig.setZero();
    jointNominalConfig[1] = 0.01;
    cartPole->setGeneralizedCoordinate(jointNominalConfig);

    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(cartPole);

    for (int i=0; i<200000; i++) {
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        server.integrateWorldThreadSafe();
    }

    server.killServer();

    return 0;
}
