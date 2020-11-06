/**
 * Copyright 2017
 *
 * This file is part of On-line POMDP Planning Toolkit (OPPT).
 * OPPT is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License published by the Free Software Foundation,
 * either version 2 of the License, or (at your option) any later version.
 *
 * OPPT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with OPPT.
 * If not, see http://www.gnu.org/licenses/.
 */
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <oppt/plugin/Plugin.hpp>
#include <oppt/robotHeaders/InverseKinematics/InverseKinematics.hpp>
#include <oppt/gazeboInterface/GazeboInterface.hpp>
#include "AudioClassificationActionDiscretizer.hpp"
#include "AudioClassificationTransitionPluginOptions.hpp"
#include "AudioClassificationUserData.hpp"
#include "MovoRobotInterface/MovoRobotInterface.hpp"
#include <chrono>

namespace oppt
{

class AudioClassificationTransitionPlugin: public TransitionPlugin
{
public :
    AudioClassificationTransitionPlugin():
        TransitionPlugin() {

            // ros::NodeHandle nh("~");

    }

    virtual ~AudioClassificationTransitionPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        parseOptions_<AudioClassificationTransitionPluginOptions>(optionsFile);
        // nh = std::make_unique<ros::NodeHandle>();
        // ros::NodeHandle nodeHandle_("my_namespace");

        auto options = static_cast<const AudioClassificationTransitionPluginOptions *>(options_.get());
        auto actionSpace = robotEnvironment_->getRobot()->getActionSpace();

        // Setup a custom action discretizer.
        // This action discretizer will generate 8 actions. See AudioClassificationActionDiscretizer.hpp
        std::shared_ptr<ActionSpaceDiscretizer> robotActionDiscretizer =
            std::shared_ptr<ActionSpaceDiscretizer>(new AudioClassificationActionDiscretizer(actionSpace,
                    std::vector<unsigned int>()));
        actionSpace->setActionSpaceDiscretizer(robotActionDiscretizer);

        // Get a pointer to the end-effector link
        std::string endEffectorLinkName = options->endEffectorLink;
        endEffectorLink_ = getLinkPointer_(endEffectorLinkName);
        if (!endEffectorLink_)
            ERROR("End effector link '" + endEffectorLinkName + "' couldn't be found");

        // Get a pointer to the cup link
        std::string cupLinkName = options->cupLink;
        cupLink_ = getLinkPointer_(cupLinkName);
        if (!cupLink_)
            ERROR("Cup link '" + cupLinkName + "' couldn't be found");

        // Setup the ik solver
        setupIKSolver_();

        endEffectorMotionDistance_ = options->endEffectorMotionDistance;

        // If this is the execution plugin, initialize the Movo API
        if (robotEnvironment_->isExecutionEnvironment())
            initializeMovoInterface_();
        return true;
    }

    virtual PropagationResultSharedPtr propagateState(const PropagationRequest* propagationRequest) const override {
        // First update Gazebo with the world state contained in the current state
        robotEnvironment_->getGazeboInterface()->setWorldState(propagationRequest->currentState->getGazeboWorldState().get());
        VectorFloat currentStateVector = propagationRequest->currentState->as<VectorState>()->asVector();
        VectorFloat endEffectorVelocity(6, 0.0);

        unsigned int actionBinNumber = propagationRequest->action->as<DiscreteVectorAction>()->getBinNumber();

        // Determines if, and which macro action is being executed.
        // 0 = no macro action
        // 1 = slide action
        // 2 = bang action
        // 3 = move cup to location A
        // 4 = move cup to location B
        int macroAction = 0;

        switch (actionBinNumber) {
        case 0:
            // X_PLUS
            endEffectorVelocity[0] += endEffectorMotionDistance_;
            break;
        case 1:
            // X_MINUS
            endEffectorVelocity[0] -= endEffectorMotionDistance_;
            break;
        case 2:
            // Y_PLUS
            endEffectorVelocity[1] += endEffectorMotionDistance_;
            break;
        case 3:
            // Y_MINUS
            endEffectorVelocity[1] -= endEffectorMotionDistance_;
            break;
        case 4:
            // SLIDE
            macroAction = 1;
            break;
        case 5:
            macroAction = 2;
            // BANG
            break;
        case 6:
            // Move location A
            macroAction = 3;
            break;
        case 7:
            // Move location B
            macroAction = 4;
            break;
        default:
            ERROR("Action not recognized");
            break;
        }

        if (robotEnvironment_->isExecutionEnvironment())
        {
            cout << "Robot Execution : Press Enter to continue" << endl;
            getchar();

        }


        VectorFloat nextJointAngles;
        if (macroAction != 0) {
            // A macro action is being executed
            nextJointAngles =
                performMacroAction_(currentStateVector,
                                    propagationRequest->currentState->getUserData()->as<AudioClassificationUserData>()->endEffectorPose,
                                    macroAction);

        }
        else
        {
            // One of the end effector motion actions (X_PLUS, X_MINUS, Y_PLUS, Y_MINUS) is being executed

            nextJointAngles = applyEndEffectorVelocity_(currentStateVector, endEffectorVelocity, actionBinNumber);
        }

        // The first 7 dimensions of the next state vector are the new joint angles
        VectorFloat nextStateVector = nextJointAngles;


        // The next 3 dimensions of the next state vector describe the relative position of
        // the cup with respect to the end effector
    #ifdef GZ_GT_7
        nextStateVector.push_back(cupLink_->WorldPose().Pos().X() - endEffectorLink_->WorldPose().Pos().X());
        nextStateVector.push_back(cupLink_->WorldPose().Pos().Y() - endEffectorLink_->WorldPose().Pos().Y());
        nextStateVector.push_back(cupLink_->WorldPose().Pos().Z() - endEffectorLink_->WorldPose().Pos().Z());
    #else
        nextStateVector.push_back(cupLink_->GetWorldPose().pos.x - endEffectorLink_->GetWorldPose().pos.x);
        nextStateVector.push_back(cupLink_->GetWorldPose().pos.y - endEffectorLink_->GetWorldPose().pos.y);
        nextStateVector.push_back(cupLink_->GetWorldPose().pos.z - endEffectorLink_->GetWorldPose().pos.z);
    #endif

        // Add the object property to the state vector (same as current state)
        nextStateVector.push_back(currentStateVector[10]);

        PropagationResultSharedPtr propagationResult(new PropagationResult);
        propagationResult->nextState = RobotStateSharedPtr(new VectorState(nextStateVector));
        propagationResult->nextState->setUserData(makeUserData_());
        propagationResult->nextState->setGazeboWorldState(robotEnvironment_->getGazeboInterface()->getWorldState(true));

        // REMOVE ME
        // LOGGING("Should run fine");
        // getchar();

        auto userDataNew = makeUserDataGrasping();
        propagationResult->nextState->setUserData(userDataNew);
        propagationResult->collisionReport = static_cast<AudioClassificationUserData *>(propagationResult->nextState->getUserData().get())->collisionReport;
        return propagationResult;
    }

private:

    /** @brief A pointer to the end effector link */
    gazebo::physics::Link *endEffectorLink_ = nullptr;

    /** @brief A pointer to the cup link */
    gazebo::physics::Link *cupLink_ = nullptr;

    /**
     * @brief The default motion distance of the end-effector (in meters) for each directional action
     * This distance can be modified in the configuration file (via transitionPluginOptions.endEffectorMotionDistance)
     */
    FloatType endEffectorMotionDistance_ = 0.05;
    FloatType endEffectorMotionDistanceSecondMacroAction_ = 0.06;

    /** @brief The interface to the physical robot */
    std::unique_ptr<MovoRobotInterface> movoRobotInterface_ = nullptr;

    // std::unique_ptr<ros::NodeHandle> nh = nullptr;
    // ros::NodeHandle nh("my_namespace");
    ros::NodeHandle nh;

    

private:
    void initializeMovoInterface_() {
        nh.setParam("/AudioClassification_Stop", false);
        nh.setParam("/AudioClassification_Record", false);
        std::string localIP =
            static_cast<const AudioClassificationTransitionPluginOptions *>(options_.get())->localIP;
        movoRobotInterface_ = std::unique_ptr<MovoRobotInterface>(new MovoRobotInterface(robotEnvironment_));
        movoRobotInterface_->init(localIP);

        // Move the arm to the initial joint angles
        VectorFloat initialState = static_cast<const AudioClassificationTransitionPluginOptions *>(options_.get())->initialState;
        VectorFloat initialJointAngles(initialState.begin(), initialState.begin() + 7);
        movoRobotInterface_->openGripper();
        std::this_thread::sleep_for(std::chrono::seconds(1));        
        movoRobotInterface_->moveToInitialJointAngles(initialJointAngles);
    }    



    RobotStateUserDataSharedPtr makeUserDataGrasping() const {

        RobotStateUserDataSharedPtr userData(new AudioClassificationUserData());
        auto ud = static_cast<AudioClassificationUserData*>(userData.get());

        ud->collisionReport = robotEnvironment_->getRobot()->makeDiscreteCollisionReportDirty();

        return userData;
    }

    VectorFloat applyEndEffectorVelocity_(const VectorFloat &currentStateVector, const VectorFloat &endEffectorVelocity, int action = 0, bool secondMacro = false) const {
        auto tracIkSolver = static_cast<oppt::TracIKSolver *>(robotEnvironment_->getRobot()->getIKSolver());
        VectorFloat currentJointAngles = getCurrentJointAngles_(currentStateVector);

        // Get the vector of joint velocities corresponding to the end effector velocity
        VectorFloat jointVelocities =
            tracIkSolver->jointVelocitiesFromTwist(currentJointAngles, endEffectorVelocity);

        // The next joint angles are then simply the current joint angles, plus the joint velocities
        VectorFloat newJointAngles = applyJointVelocities_(currentJointAngles, jointVelocities, secondMacro);

        // Update the Gazebo model with the next joint angles
        robotEnvironment_->getGazeboInterface()->setStateVector(newJointAngles);

        // Check if the cup is inside the gripper. If yes, move the cup with the gripper
        VectorFloat relativeCupPosition({currentStateVector[7],
                                         currentStateVector[8],
                                         currentStateVector[9]
                                        });

        // Here we assume that if the euclidean distance between the end effector and the cup
        // is smaller than 0.01 meters, the cup will "travel" with the end effector.
        // TODO: Make sure the cup remains in its current positions when the end effector
        // moves away from the cup
        FloatType l2norm = sqrt(std::inner_product(relativeCupPosition.begin(), relativeCupPosition.end(), relativeCupPosition.begin(), 0.0));
        if ((l2norm < 0.01) && (action != 1)) 
        {
            GZPose endEffectorPose = LinkWorldPose(endEffectorLink_);
#ifdef GZ_GT_7
            geometric::Pose newCupPose(endEffectorPose.Pos().X(),
                                       endEffectorPose.Pos().Y(),
                                       endEffectorPose.Pos().Z(),
                                       0.0,
                                       0.0,
                                       0.0);
#else
            geometric::Pose newCupPose(endEffectorPose.pos.x,
                                       endEffectorPose.pos.y,
                                       endEffectorPose.pos.z,
                                       0.0,
                                       0.0,
                                       0.0);
#endif
            cupLink_->SetWorldPose(newCupPose.toGZPose());
        }

        return newJointAngles;
    }

    /**
     * @brief Get the current joint angles of the robot. If this is the planning plugin,
     * the current joint angles are simply obtained from the current state vector. If this is the
     * execution plugin, we read the current joint angles from the robot
     */
    VectorFloat getCurrentJointAngles_(const VectorFloat &currentStateVector) const {
        if (robotEnvironment_->isExecutionEnvironment())
            return movoRobotInterface_->getCurrentJointAngles();
        VectorFloat currentJointAngles(currentStateVector.begin(), currentStateVector.begin() + 7);
        return currentJointAngles;
    }

    /**
     * @brief Apply a vector of joint velocities to the robot and return the resulting vector of joint angles
     */
    VectorFloat applyJointVelocities_(const VectorFloat &currentJointAngles, const VectorFloat &jointVelocities, bool secondMacro = false) const {
        if (robotEnvironment_->isExecutionEnvironment()) {
            FloatType durationMS = 1000.0;
            if (secondMacro)
            {
                cout<<"Message for second macro"<<endl;
                durationMS = 500.0;
            }
            movoRobotInterface_->applyJointVelocities(jointVelocities, durationMS);
            return movoRobotInterface_->getCurrentJointAngles();
        }

        return addVectors(currentJointAngles, jointVelocities);
    }

    /**
     * @brief Perform one of the macro actions (SLIDE, BANG, Move to location A, Move to location B)
     */
    VectorFloat performMacroAction_(const VectorFloat &currentStateVector,
                                    const geometric::Pose &currentEndEffectorPose,
                                    const int &macroAction) const {
        VectorFloat newJointAngles;
        VectorFloat jointAnglesBeforeLift;
        VectorFloat jointAnglesAfterLift;
        FloatType timestart = oppt::clock_ms();
        
        if (macroAction == 1 or macroAction == 2) {
            // In case we execute the SLIDE or BANG macro action, we first have to move the
            // end effector to the current cup position
            newJointAngles = moveEndEffectorToCupPosition_(currentStateVector, currentEndEffectorPose);
        }

        if (macroAction == 1) {
            // Afer moving the end effector to the cup position, we additional push the cup by a predefined distance
            // specified by endEffectorMotionDistance_

            // Get the updated end effector pose after we moved it to the cup position
            geometric::Pose newEndEffectorPose(LinkWorldPose(endEffectorLink_));

            // Then we push the cup forward in x-direction (relative to the end effector link)
            geometric::Pose pushVector(endEffectorMotionDistance_, 0.0, 0.0, 0.0, 0.0, 0.0);

            // Compute the desired end effector pose after pushing the cup
            geometric::Pose newEndEffectorWorldPoseAfterPushing = pushVector + newEndEffectorPose;

            // The end effector velocity necessary to bring the end effector from its current pose (the pose after
            // moving it to the cup position) to the desired end effector pose after pushing
            VectorFloat endEffectorVelocity(6, 0.0);
            endEffectorVelocity[0] = newEndEffectorWorldPoseAfterPushing.position.x() - newEndEffectorPose.position.x();
            endEffectorVelocity[1] = newEndEffectorWorldPoseAfterPushing.position.y() - newEndEffectorPose.position.y();
            endEffectorVelocity[2] = newEndEffectorWorldPoseAfterPushing.position.z() - newEndEffectorPose.position.z();

            // Apply this velocity to the end effector. This will result in a new set of joint angles that correspong
            // to the resulting end effector pose (after pushing the object)
            // if (robotEnvironment_->isExecutionEnvironment())
            // {

            //     cout<<"xyz coords before sliding action: "<<endEffectorLink_->GetWorldPose().pos.x<<" "<<endEffectorLink_->GetWorldPose().pos.y<<" "<<endEffectorLink_->GetWorldPose().pos.z<<endl;            
            //     FloatType elapsedSinceStart = 0.0;
            //     auto timeStart = std::chrono::system_clock::now();
            //     newJointAngles = applyEndEffectorVelocity_(newJointAngles, endEffectorVelocity);
            //     elapsedSinceStart = (FloatType)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - timeStart).count());
            //     cout << "time to execute sliding action: " << elapsedSinceStart<< endl;
            //     cout<<"xyz coords after sliding action: "<<endEffectorLink_->GetWorldPose().pos.x<<" "<<endEffectorLink_->GetWorldPose().pos.y<<" "<<endEffectorLink_->GetWorldPose().pos.z<<endl;
            // }
            // else
            // {
            //     newJointAngles = applyEndEffectorVelocity_(newJointAngles, endEffectorVelocity);
            // }
            if (robotEnvironment_->isExecutionEnvironment())
            {   
                // movoRobotInterface_->closeGripper();
                std::this_thread::sleep_for(std::chrono::seconds(2));
                nh.setParam("/AudioClassification_Record", true);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                cout<<"====================>START SIGNAL SENT"<<endl;
                newJointAngles = applyEndEffectorVelocity_(newJointAngles, endEffectorVelocity);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                nh.setParam("/AudioClassification_Record", false);
                cout<<"====================>END SIGNAL SENT"<<endl;
                // std::this_thread::sleep_for(std::chrono::seconds(2));
                // movoRobotInterface_->openGripper();

            }
            else
            {
                newJointAngles = applyEndEffectorVelocity_(newJointAngles, endEffectorVelocity);
            }    

            // We simply set the resulting world pose of the cup (after pushing it) to be equal to the resulting
            // end effector world pose
            geometric::Pose newCupWorldPoseAfterPushing(newEndEffectorWorldPoseAfterPushing.position.x(),
                    newEndEffectorWorldPoseAfterPushing.position.y(),
                    newEndEffectorWorldPoseAfterPushing.position.z(),
                    0.0,
                    0.0,
                    0.0);

            cupLink_->SetWorldPose(newCupWorldPoseAfterPushing.toGZPose());
        }

        if (macroAction == 2) {
            // For the BANG action we assume that the resulting joint angles are equal
            // to the ones we got after moving the end effector to the cup position.
            // So we actually don't have to do anything here


            if (robotEnvironment_->isExecutionEnvironment())
            {
                // cout<<"xyz coords before any action: "<<endEffectorLink_->GetWorldPose().pos.x<<" "<<endEffectorLink_->GetWorldPose().pos.y<<" "<<endEffectorLink_->GetWorldPose().pos.z<<endl;
                movoRobotInterface_->closeGripper();
                std::this_thread::sleep_for(std::chrono::seconds(1));
                VectorFloat endEffectorVelocity(6, 0.0);
                endEffectorVelocity[2] = endEffectorMotionDistance_;
                // FloatType elapsedSinceStart = 0.0;
                // auto timeStart = std::chrono::system_clock::now();
                jointAnglesBeforeLift = movoRobotInterface_->getCurrentJointAngles();
                nh.setParam("/AudioClassification_Record", true);
                cout<<"====================>START SIGNAL SENT"<<endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                newJointAngles = applyEndEffectorVelocity_(newJointAngles, endEffectorVelocity);
                // jointAnglesAfterLift = movoRobotInterface_->getCurrentJointAngles();
                // auto distance = math::euclideanDistance<FloatType>(jointAnglesBeforeLift, jointAnglesAfterLift);
                // printVector(jointAnglesBeforeLift, "joint angles before lift");
                // printVector(jointAnglesAfterLift, "joint angles after lift");
                // cout<<"EUCLIDEAN DISTANCE BETWEEN JOINT ANGLES BEFORE AND AFTER LIFT : "<< distance << endl;
                // elapsedSinceStart = (FloatType)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - timeStart).count());
                // cout << "time to execute lift action: " << elapsedSinceStart << endl;
                // cout<<"xyz coords after lifting : "<<endEffectorLink_->GetWorldPose().pos.x<<" "<<endEffectorLink_->GetWorldPose().pos.y<<" "<<endEffectorLink_->GetWorldPose().pos.z<<endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                endEffectorVelocity[2] = -(2*endEffectorMotionDistanceSecondMacroAction_);
                // elapsedSinceStart = 0.0;
                // timeStart = std::chrono::system_clock::now();
                newJointAngles = applyEndEffectorVelocity_(newJointAngles, endEffectorVelocity, 0, true);
                cout<<"====================>END SIGNAL SENT"<<endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                nh.setParam("/AudioClassification_Record", false);
                // elapsedSinceStart = (FloatType)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - timeStart).count());
                // cout << "time to execute downward action: " << elapsedSinceStart << endl;
                // cout<<"xyz coords after all actions : "<<endEffectorLink_->GetWorldPose().pos.x<<" "<<endEffectorLink_->GetWorldPose().pos.y<<" "<<endEffectorLink_->GetWorldPose().pos.z<<endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                movoRobotInterface_->openGripper();
                std::this_thread::sleep_for(std::chrono::seconds(1));
                movoRobotInterface_->sendTargetJointAngles_(jointAnglesBeforeLift, 1000.0);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                
            }

        }

        if (macroAction == 3 or macroAction == 4) {
            // For the Move to location A and Move to location B macro actions,
            // we can (for the moment) simply return the current joint angles,
            // because executing those actions will result in a terminal state
            if (robotEnvironment_->isExecutionEnvironment())
            {
                nh.setParam("/AudioClassification_Stop", true);
            }
            newJointAngles = VectorFloat(currentStateVector.begin(), currentStateVector.begin() + 7);
        }

        return newJointAngles;
    }


    /**
     * @brief Moves the end effector to the current cup position
     */
    VectorFloat moveEndEffectorToCupPosition_(const VectorFloat & currentStateVector,
            const geometric::Pose & currentEndEffectorPose) const {
        // Get the relative XYZ-position of the cup (with respect to the end effector) from the current state
        VectorFloat currentCupPoseVector(6, 0.0);
        currentCupPoseVector[0] = currentStateVector[7];
        currentCupPoseVector[1] = currentStateVector[8];
        currentCupPoseVector[2] = currentStateVector[9];
        geometric::Pose currentCupRelativePose(currentCupPoseVector[0],
                                               currentCupPoseVector[1],
                                               currentCupPoseVector[2],
                                               currentCupPoseVector[3],
                                               currentCupPoseVector[4],
                                               currentCupPoseVector[5]);

        // Compute the current world pose of the cup
        geometric::Pose currentCupWorldPose = currentCupRelativePose + currentEndEffectorPose;

        // Compute the desired end-effector velocity as the difference between the current world pose of the cup
        // and the current world pose of the end effector
        VectorFloat endEffectorVelocity({currentCupWorldPose.position.x() - currentEndEffectorPose.position.x(),
                                         currentCupWorldPose.position.y() - currentEndEffectorPose.position.y(),
                                         currentCupWorldPose.position.z() - currentEndEffectorPose.position.z(),
                                         0.0,
                                         0.0,
                                         0.0
                                        });

        // Call the applyEndEffectorVelocity_ method to get the set of joint angles that results from
        // applying the end effector velocity to the current pose of the end effector
        return applyEndEffectorVelocity_(currentStateVector, endEffectorVelocity);
    }

    /**
     * @brief Initialized the inverse kinematics solver
     */
    void setupIKSolver_() {
        auto options = static_cast<const AudioClassificationTransitionPluginOptions *>(options_.get());
        std::string urdfFile = options->urdfFile;
        std::string baseLink = options->baseLink;
        std::string endEffectorLink = options->endEffectorLink;

        if (oppt::resources::FileExists(urdfFile) == false)
            ERROR("URDF file '" + urdfFile + "' doesn't exist");

        std::string urdfPath = oppt::resources::FindFile(urdfFile);
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();
        IKSolverUniquePtr ikSolver =
            std::make_unique<TracIKSolver>(randomEngine.get(), urdfPath, baseLink, endEffectorLink);
        auto tracIkSolver = static_cast<oppt::TracIKSolver *>(ikSolver.get());
        if (tracIkSolver->init() == false)
            ERROR("IKSolver could not be initialized");
        robotEnvironment_->getRobot()->setIKSolver(std::move(ikSolver));
    }

    OpptUserDataSharedPtr makeUserData_() const {
        OpptUserDataSharedPtr userData(new AudioClassificationUserData);
        userData->as<AudioClassificationUserData>()->endEffectorPose = geometric::Pose(LinkWorldPose(endEffectorLink_));
        return userData;

    }

    /**
     * @brief Helper function to get a pointer to a link with the given link name
     */
    gazebo::physics::Link *getLinkPointer_(const std::string & linkName) const {
        gazebo::physics::Link *linkPtr = nullptr;
        auto links = robotEnvironment_->getGazeboInterface()->getLinks();
        for (auto &link : links) {
            if (link->GetScopedName().find(linkName) != std::string::npos) {
                linkPtr = link;
                break;
            }
        }

        return linkPtr;
    }

    GZPose LinkWorldPose(const gazebo::physics::Link* link) const {
        // Returns link world pose according to gazebo api enabled
    #ifdef GZ_GT_7
        return link->WorldPose();
    #else
        return link->GetWorldPose();
    #endif

    }


};

OPPT_REGISTER_TRANSITION_PLUGIN(AudioClassificationTransitionPlugin)

}
