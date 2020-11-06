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
#include <oppt/plugin/Plugin.hpp>
#include "oppt/opptCore/Distribution.hpp"
namespace oppt
{
class AudioClassificationObservationPlugin: public ObservationPlugin
{
public :
    AudioClassificationObservationPlugin():
        ObservationPlugin() {

    }

    virtual ~AudioClassificationObservationPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        makeObservationDistribution();
        return true;
    }

    virtual ObservationResultSharedPtr getObservation(const ObservationRequest* observationRequest) const override {
        ObservationResultSharedPtr observationResult(new ObservationResult);
        VectorFloat observationVec({0.0});
        observationResult->state = observationRequest->currentState.get();
        observationResult->action = observationRequest->action;
        auto Action = observationRequest->action;
        unsigned int binNumber = Action->as<DiscreteVectorAction>()->getBinNumber();
        VectorFloat stateVec = observationRequest->currentState->as<VectorState>()->asVector();
        FloatType sample = dist_(*(robotEnvironment_->getRobot()->getRandomEngine().get()));

        if (binNumber == 4) //ACTION 1
        {
            if (stateVec[stateVec.size() - 1] == 0) // PRINGLES CAN
            {
                if (sample <= 0.50)
                {
                    observationVec[0] = 0;
                }
                else if (sample <= 0.80)
                {
                    observationVec[0] = 1;
                }
                else if (sample <= 0.95)
                {
                    observationVec[0] = 2;
                }
                else
                {
                    observationVec[0] = 3;
                }            
                // cout<<"Pringles can action 1 : "<<observationVec[0]<<"  "<<observationVec[1]<<endl;
            }
            else if (stateVec[stateVec.size() - 1] == 1)// COFFEE MUG
            {
                if (sample <= 0.67)
                {
                    observationVec[0] = 1;
                }
                else if (sample <= 0.78)
                {
                    observationVec[0] = 0;
                }
                else if (sample <= 0.89)
                {
                    observationVec[0] = 2;
                }
                else
                {
                    observationVec[0] = 3;
                }
                // cout<<"Coffee mug action 1 : "<<observationVec[0]<<"  "<<observationVec[1]<<endl;
            }
            else if (stateVec[stateVec.size() - 1] == 2)// COFFEE PLASTIC CUP
            {
                if (sample <= 0.67)
                {
                    observationVec[0] = 2;
                }
                else if (sample <= 0.78)
                {
                    observationVec[0] = 1;
                }
                else if (sample <= 0.89)
                {
                    observationVec[0] = 0;
                }
                else
                {
                    observationVec[0] = 3;
                }
                // cout<<"Coffee mug action 1 : "<<observationVec[0]<<"  "<<observationVec[1]<<endl;
            }
            else                             // PAPER BASE 
            {
                if (sample <= 0.5)
                {
                    observationVec[0] = 3;
                }
                else if (sample <= 0.75)
                {
                    observationVec[0] = 1;
                }
                else if (sample <= 0.99)
                {
                    observationVec[0] = 2;
                }
                else
                {
                    observationVec[0] = 0;
                }
                // cout<<"Coffee mug action 1 : "<<observationVec[0]<<"  "<<observationVec[1]<<endl;
            }
            
        }
        else if (binNumber == 5)    //ACTION 2
        {
            if (stateVec[stateVec.size() - 1] == 0) // PRINGLES CAN
            {
                if (sample <= 0.6)
                {
                    observationVec[0] = 0;
                }
                else if (sample <= 0.9)
                {
                    observationVec[0] = 2;
                }
                else if (sample <= 0.95)
                {
                    observationVec[0] = 1;
                }
                else 
                {
                    observationVec[0] = 3;
                }
                
                
                // cout<<"Pringles can action 1 : "<<observationVec[0]<<"  "<<observationVec[1]<<endl;
            }
            else if (stateVec[stateVec.size() - 1] == 1)// COFFEE MUG
            {
                if (sample <= 0.6)
                {
                    observationVec[0] = 1;
                }
                else if (sample <= 0.9) 
                {
                    observationVec[0] = 2;
                }
                else if (sample <= 0.95) 
                {
                    observationVec[0] = 0;
                }
                else 
                {
                    observationVec[0] = 3;
                }
                // cout<<"Coffee mug action 1 : "<<observationVec[0]<<"  "<<observationVec[1]<<endl;
            }
            else if (stateVec[stateVec.size() - 1] == 2)// COFFEE PLASTIC CUP
            {
                if (sample <= 0.67)
                {
                    observationVec[0] = 2;
                }
                else if (sample <= 0.78)
                {
                    observationVec[0] = 1;
                }
                else if (sample <= 0.89)
                {
                    observationVec[0] = 0;
                }
                else
                {
                    observationVec[0] = 3;
                }
                // cout<<"Coffee mug action 1 : "<<observationVec[0]<<"  "<<observationVec[1]<<endl;
            }
            else                             // PAPER BASE 
            {
                if (sample <= 0.67)
                {
                    observationVec[0] = 3;
                }
                else if (sample <= 0.78)
                {
                    observationVec[0] = 1;
                }
                else if (sample <= 0.89)
                {
                    observationVec[0] = 2;
                }
                else
                {
                    observationVec[0] = 0;
                }
                // cout<<"Coffee mug action 1 : "<<observationVec[0]<<"  "<<observationVec[1]<<endl;
            }
        }
        else
        {   
            observationVec[0] = 5;
        }

        
        observationResult->observation = ObservationSharedPtr(new VectorObservation(observationVec));
        return observationResult;
    }

    virtual double calcLikelihood(const RobotStateSharedPtr &state,
                                  const Action *action,
                                  const Observation *observation) const override {
        unsigned int binNumber = action->as<DiscreteVectorAction>()->getBinNumber();
        VectorFloat stateVec = state->as<VectorState>()->asVector();
        VectorFloat observationVec = observation->as<VectorObservation>()->asVector();
        FloatType pdf = 0.0;
        if (binNumber == 4) //ACTION1
        {
            if (stateVec[stateVec.size() - 1] == 0) //PRINGLES CAN
            {
                if (observationVec[observationVec.size() - 1] == 0)
                    return 0.50;
                else if (observationVec[observationVec.size() - 1] == 1)
                    return 0.30;
                else if (observationVec[observationVec.size() - 1] == 2)
                    return 0.15;
                else
                    return 0.05;
            }
            else if (stateVec[stateVec.size() - 1] == 1)// COFFEE MUG
            {
                if (observationVec[observationVec.size() - 1] == 0)
                    return 0.11;
                else if (observationVec[observationVec.size() - 1] == 1)
                    return 0.67;
                else if (observationVec[observationVec.size() - 1] == 2)
                    return 0.11;
                else
                    return 0.11;
            }
            else if (stateVec[stateVec.size() - 1] == 2)// COFEE PLASTIC CUP
            {
                if (observationVec[observationVec.size() - 1] == 0)
                    return 0.11;
                else if (observationVec[observationVec.size() - 1] == 1)
                    return 0.11;
                else if (observationVec[observationVec.size() - 1] == 2)
                    return 0.67;
                else
                    return 0.11;
            }
            else                                        // PAPER BASE
            {
                if (observationVec[observationVec.size() - 1] == 0)
                    return 0.01;
                else if (observationVec[observationVec.size() - 1] == 1)
                    return 0.25;
                else if (observationVec[observationVec.size() - 1] == 2)
                    return 0.25;
                else
                    return 0.5;
            }                        

        }
        else if (binNumber == 5) //ACTION2
        {
            if (stateVec[stateVec.size() - 1] == 0) //PRINGLES CAN
            {
                if (observationVec[observationVec.size() - 1] == 0)
                    return 0.6;
                else if (observationVec[observationVec.size() - 1] == 1)
                    return 0.05;
                else if (observationVec[observationVec.size() - 1] == 2)
                    return 0.3;
                else
                    return 0.05;
            }
            else if (stateVec[stateVec.size() - 1] == 1)// COFFEE MUG
            {
                if (observationVec[observationVec.size() - 1] == 0)
                    return 0.05;
                else if (observationVec[observationVec.size() - 1] == 1)
                    return 0.6;
                else if (observationVec[observationVec.size() - 1] == 2)
                    return 0.3;
                else
                    return 0.05;
            }
            else if (stateVec[stateVec.size() - 1] == 2)// COFEE PLASTIC CUP
            {
                if (observationVec[observationVec.size() - 1] == 0)
                    return 0.11;
                else if (observationVec[observationVec.size() - 1] == 1)
                    return 0.11;
                else if (observationVec[observationVec.size() - 1] == 2)
                    return 0.67;
                else
                    return 0.11;
            }
            else                                        // PAPER BASE
            {
                if (observationVec[observationVec.size() - 1] == 0)
                    return 0.11;
                else if (observationVec[observationVec.size() - 1] == 1)
                    return 0.11;
                else if (observationVec[observationVec.size() - 1] == 2)
                    return 0.11;
                else
                    return 0.67;
            }
        }
        else
        {
            if (observationVec[observationVec.size() - 1] == 5)
                return 1.0;
            else
                cout<<observationVec[0]<<"  "<<observationVec[1]<<endl;
                ERROR("IMPOSSIBLE");
        }

    }   

    bool makeObservationDistribution() 
    {
        auto randomEngine = robotEnvironment_->getRobot()->getRandomEngine();

        coffee_mug_action1 = std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
        coffee_mug_action2 = std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
        pringles_can_action1 = std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
        pringles_can_action2 = std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
        coffee_plastic_cup_action1 = std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
        coffee_plastic_cup_action2 = std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
        coffee_cup_paper_base_action1 = std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
        coffee_cup_paper_base_action2 = std::unique_ptr<MultivariateNormalDistribution<FloatType>>(new MultivariateNormalDistribution<FloatType>(randomEngine));
// -----------------------------------------------------------------------------------------------------------------------1
        Matrixdf mean = Matrixdf::Zero(2, 1);
        Matrixdf covarianceMatrix = Matrixdf::Identity(2, 2);
        mean(0,0) = 1121.0;
        mean(1,0) = 0.0178;
        covarianceMatrix(0,0) = 30.0*30.0;
        covarianceMatrix(1,1) = 1.265e-6;
        coffee_mug_action1->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        coffee_mug_action1->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);
// --------------------------------------------------------------------------------------------------------------------- 2
        mean = Matrixdf::Zero(2, 1);
        covarianceMatrix = Matrixdf::Identity(2, 2);
        mean(0,0) = 1308.0;
        mean(1,0) = 0.01220;
        covarianceMatrix(0,0) = 90.0*90.0;
        covarianceMatrix(1,1) = 3e-6;
        coffee_mug_action2->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        coffee_mug_action2->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);
// -----------------------------------------------------------------------------------------------------------------------3
        mean = Matrixdf::Zero(2, 1);
        covarianceMatrix = Matrixdf::Identity(2, 2);
        mean(0,0) = 1426.0;
        mean(1,0) = 0.0155;//0.01452;
        covarianceMatrix(0,0) = 47.0*47.0;
        covarianceMatrix(1,1) = 1.0965e-6;//4.26e-6;
        pringles_can_action1->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        pringles_can_action1->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);
// ----------------------------------------------------------------------------------------------------------------------- 4
        mean = Matrixdf::Zero(2, 1);
        covarianceMatrix = Matrixdf::Identity(2, 2);
        mean(0,0) = 2423.0;
        mean(1,0) = 0.00763;//0.01597;
        covarianceMatrix(0,0) = 92.0*92.0;
        covarianceMatrix(1,1) = 1.4e-6;//4.53e-6;
        pringles_can_action2->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        pringles_can_action2->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);
// ----------------------------------------------------------------------------------------------------------------------- 5
        mean = Matrixdf::Zero(2, 1);
        covarianceMatrix = Matrixdf::Identity(2, 2);
        mean(0,0) = 1277.0;
        mean(1,0) = 0.016;//0.00888;
        covarianceMatrix(0,0) = 33.0*33.0;
        covarianceMatrix(1,1) = 5.45e-6;//6.92e-6;
        coffee_plastic_cup_action1->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        coffee_plastic_cup_action1->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);
// ----------------------------------------------------------------------------------------------------------------------- 6
        mean = Matrixdf::Zero(2, 1);
        covarianceMatrix = Matrixdf::Identity(2, 2);
        mean(0,0) = 1547.0;
        mean(1,0) = 0.011;//0.0168;
        covarianceMatrix(0,0) = 96.0*96.0;
        covarianceMatrix(1,1) = 8e-6;//1.26e-6;
        coffee_plastic_cup_action2->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        coffee_plastic_cup_action2->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);
// ----------------------------------------------------------------------------------------------------------------------- 7
        mean = Matrixdf::Zero(2, 1);
        covarianceMatrix = Matrixdf::Identity(2, 2);
        mean(0,0) = 4500.0;
        mean(1,0) = 5.00825;
        covarianceMatrix(0,0) = 59.0*59.0;
        covarianceMatrix(1,1) = 3.54e-6;
        coffee_cup_paper_base_action1->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        coffee_cup_paper_base_action1->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);
// ----------------------------------------------------------------------------------------------------------------------- 8
        mean = Matrixdf::Zero(2, 1);
        covarianceMatrix = Matrixdf::Identity(2, 2);
        mean(0,0) = 4500.0;
        mean(1,0) = 5.00844;
        covarianceMatrix(0,0) = 88.0*88.0;
        covarianceMatrix(1,1) = 5.63e-6;
        coffee_cup_paper_base_action2->as<MultivariateNormalDistribution<FloatType>>()->setMean(mean);
        coffee_cup_paper_base_action2->as<MultivariateNormalDistribution<FloatType>>()->setCovariance(covarianceMatrix);

    }

private:
    mutable std::uniform_real_distribution<FloatType> dist_;

    std::unique_ptr<Distribution<FloatType>> coffee_mug_action1;
    std::unique_ptr<Distribution<FloatType>> coffee_mug_action2;
    std::unique_ptr<Distribution<FloatType>> pringles_can_action1;
    std::unique_ptr<Distribution<FloatType>> pringles_can_action2;
    std::unique_ptr<Distribution<FloatType>> coffee_plastic_cup_action1;
    std::unique_ptr<Distribution<FloatType>> coffee_plastic_cup_action2;
    std::unique_ptr<Distribution<FloatType>> coffee_cup_paper_base_action1;
    std::unique_ptr<Distribution<FloatType>> coffee_cup_paper_base_action2;

};

OPPT_REGISTER_OBSERVATION_PLUGIN(AudioClassificationObservationPlugin)

}
