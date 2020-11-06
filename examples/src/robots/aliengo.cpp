//
// Created by Jemin Hwangbo on 10/15/19.
// MIT License
//
// Copyright (c) 2019-2019 Robotic Systems Lab, ETH Zurich
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#include <raisim/OgreVis.hpp>
#include "raisimBasicImguiPanel.hpp"
#include "raisimKeyboardCallback.hpp"
#include "helper.hpp"

void setupCallback() {
  auto vis = raisim::OgreVis::get();

  /// light
  vis->getLight()->setDiffuseColour(1, 1, 1);
  vis->getLight()->setCastShadows(true);
  Ogre::Vector3 lightdir(-3, -3, -0.5);
  lightdir.normalise();
  vis->getLightNode()->setDirection({lightdir});

  /// load  textures
  vis->addResourceDirectory(vis->getResourceDir() + "/material/checkerboard");
  vis->loadMaterialFile("checkerboard.material");

  /// shdow setting
  vis->getSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
  vis->getSceneManager()->setShadowTextureSettings(2048, 3);

  /// scale related settings!! Please adapt it depending on your map size
  // beyond this distance, shadow disappears
  vis->getSceneManager()->setShadowFarDistance(30);
  // size of contact points and contact forces
  vis->setContactVisObjectSize(0.06, .6);
  // speed of camera motion in freelook mode
  vis->getCameraMan()->setTopSpeed(5);
}

int main(int argc, char **argv) {
  raisim::World::setActivationKey(raisim::loadResource("activation.raisim"));

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.01);

  auto vis = raisim::OgreVis::get();

  /// these method must be called before initApp
  vis->setWorld(&world);
  vis->setWindowSize(1800, 1200);
  vis->setImguiSetupCallback(imguiSetupCallback);
  vis->setImguiRenderCallback(imguiRenderCallBack);
  vis->setKeyboardCallback(raisimKeyboardCallback);
  vis->setSetUpCallback(setupCallback);
  vis->setAntiAliasing(2);
  vis->setDesiredFPS(25);

  raisim::gui::manualStepping = true;

  /// starts visualizer thread
  vis->initApp();

  /// create raisim objects
  auto ground = world.addGround();

  /// create visualizer objects
  vis->createGraphicalObject(ground, 20, "floor", "checkerboard_green");

  /// laikago joint PD controller
  Eigen::VectorXd jointNominalConfig(19), jointVelocityTarget(18);
  Eigen::VectorXd jointState(18), jointForce(18), jointPgain(18), jointDgain(18);
  jointPgain.setZero();
  jointDgain.setZero();
  jointVelocityTarget.setZero();
  jointPgain.tail(12).setConstant(200.0);
  jointDgain.tail(12).setConstant(10.0);
  std::vector<raisim::ArticulatedSystem*> aliengos;
  std::vector<std::vector<raisim::GraphicObject>*> aliengo_vis;

  const int N = 12;

  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      aliengos.push_back(world.addArticulatedSystem(raisim::loadResource("aliengo/aliengo.urdf")));

      aliengo_vis.push_back(vis->createGraphicalObject(aliengos.back(), "aliengo" + std::to_string(i) + "/" + std::to_string(j)));
      aliengos.back()->setGeneralizedCoordinate({double((i-N/2)*1.5), double((j-N/2)*1.5), 0.48, 1, 0.0, 0.0, 0.0, 0.0, 0.5, -1, 0, 0.5, -1,
                                         0.00, 0.5, -1, 0, 0.5, -0.7});
      aliengos.back()->setGeneralizedForce(Eigen::VectorXd::Zero(aliengos.back()->getDOF()));
      aliengos.back()->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
      aliengos.back()->setPdGains(jointPgain, jointDgain);
      aliengos.back()->setName("aliengo" + std::to_string(i) + "/" + std::to_string(j));
    }
  }


  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 0.2);
  std::srand(std::time(nullptr));

  // lambda function for the controller
  auto controller = [&aliengos, &generator, &distribution]() {
    static size_t controlDecimation = 0;

    if (controlDecimation++ % 100 != 0)
      return;

    /// laikago joint PD controller
    Eigen::VectorXd jointNominalConfig(19), jointVelocityTarget(18);
    jointVelocityTarget.setZero();

    for (int i = 0; i < N; i++) {
      for (int j = 0; j < N; j++) {
        if (controlDecimation++ % 2500 == 0)
          aliengos[N*i+j]->setGeneralizedCoordinate({double((i-N/2)*1.5), double((j-N/2)*1.5), 0.48, 1, 0.0, 0.0, 0.0, 0.0,
                                                 0.5, -1,
                                                 0, 0.5, -1, 0.00, 0.5, -1, 0, 0.5, -1});

        jointNominalConfig << 0, 0, 0, 0, 0, 0, 0, 0.0, 0.5, -1, 0, 0.5, -1, 0.00, 0.5, -1, 0, 0.5, -1.;

        for (size_t k = 0; k < aliengos[0]->getGeneralizedCoordinateDim(); k++)
          jointNominalConfig(k) += distribution(generator);

        aliengos[N*i+j]->setPdTarget(jointNominalConfig, jointVelocityTarget);
      }
    }
  };

  vis->setControlCallback(controller);

  /// set camera
  vis->select(aliengo_vis[0]->at(0));
  vis->getCameraMan()->setYawPitchDist(Ogre::Radian(0.), Ogre::Radian(-1.), 3);

  /// run the app
  vis->run();

  /// terminate
  vis->closeApp();

  return 0;
}
