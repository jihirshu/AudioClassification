#ifndef _AUDIO_CLASSIFICATION_ACTION_DISCRETIZER_HPP_
#define _AUDIO_CLASSIFICATION_ACTION_DISCRETIZER_HPP_
#include <oppt/robotHeaders/ActionSpaceDiscretizer.hpp>


namespace oppt {
class AudioClassificationActionDiscretizer: public CustomActionSpaceDiscretizer {
public:
	AudioClassificationActionDiscretizer(ActionSpaceSharedPtr &actionSpace,
	                                     const std::vector<unsigned int>& discretization):
		CustomActionSpaceDiscretizer(actionSpace, discretization) {

	}

	virtual ~AudioClassificationActionDiscretizer() = default;

	virtual std::vector<ActionSharedPtr> getAllActionsInOrder(const unsigned int& numStepsPerDimension) const override {
		std::vector<ActionSharedPtr> allActions(8, nullptr);
		allActions[0] = ActionSharedPtr(new DiscreteVectorAction({0.0}));
		allActions[0]->as<DiscreteVectorAction>()->setBinNumber(0);
		allActions[1] = ActionSharedPtr(new DiscreteVectorAction({1.0}));
		allActions[1]->as<DiscreteVectorAction>()->setBinNumber(1);
		allActions[2] = ActionSharedPtr(new DiscreteVectorAction({2.0}));
		allActions[2]->as<DiscreteVectorAction>()->setBinNumber(2);
		allActions[3] = ActionSharedPtr(new DiscreteVectorAction({3.0}));
		allActions[3]->as<DiscreteVectorAction>()->setBinNumber(3);
		// Sliding action
		allActions[4] = ActionSharedPtr(new DiscreteVectorAction({4.0}));
		allActions[4]->as<DiscreteVectorAction>()->setBinNumber(4);

		// Bang action
		allActions[5] = ActionSharedPtr(new DiscreteVectorAction({5.0}));
		allActions[5]->as<DiscreteVectorAction>()->setBinNumber(5);


		// Move to location A
		allActions[6] = ActionSharedPtr(new DiscreteVectorAction({6.0}));
		allActions[6]->as<DiscreteVectorAction>()->setBinNumber(6);

		// Move to location B
		allActions[7] = ActionSharedPtr(new DiscreteVectorAction({7.0}));
		allActions[7]->as<DiscreteVectorAction>()->setBinNumber(7);

		return allActions;
	}
};

}

#endif