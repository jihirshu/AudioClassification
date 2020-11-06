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

namespace oppt
{
class AudioClassificationRewardPlugin: public RewardPlugin
{
public :
    AudioClassificationRewardPlugin():
        RewardPlugin() {

    }

    virtual ~AudioClassificationRewardPlugin() = default;

    virtual bool load(const std::string& optionsFile) override {
        return true;
    }

    virtual double getReward(const PropagationResultSharedPtr& propagationResult) const override {
        VectorFloat stateVec = propagationResult->nextState->as<VectorState>()->asVector();
        auto Action = propagationResult->action;
        unsigned int binNumber = Action->as<DiscreteVectorAction>()->getBinNumber();

        if ((binNumber == 4)|| (binNumber == 5)) // SLIDE OR BANG ACTION
        {
            return -1.0;
        }
        else if (binNumber == 6) // MOVE TO LOCATION A
        {
            cout<<"FINAL STATE : "<<stateVec[stateVec.size() - 1]<<endl;
            if ((stateVec[stateVec.size() - 1] == 0) || (stateVec[stateVec.size() - 1] == 2)) 
            {
                return 10.0;
            }
            else
            {
                return -100.0;
            }
        }
        else if (binNumber == 7) // MOVE TO LOCATION B
        {
            cout<<"FINAL STATE : "<<stateVec[stateVec.size() - 1]<<endl;
            if ((stateVec[stateVec.size() - 1] == 1) || (stateVec[stateVec.size() - 1] == 3))
            {
                return 10.0;
            }
            else
            {
                return -100.0;
            }
        }
        else // MOVEMENT ACTIONS
        {
            return -10.0;
        }

    }

    virtual std::pair<double, double> getMinMaxReward() const override 
    {
        return std::make_pair(0.0,
                              0.0);
    }
};

OPPT_REGISTER_REWARD_PLUGIN(AudioClassificationRewardPlugin)

}
