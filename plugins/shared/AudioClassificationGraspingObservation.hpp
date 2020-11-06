#ifndef _AudioClassification_OBSERVATION_HPP_
#define _AudioClassification_OBSERVATION_HPP_
#include <oppt/robotHeaders/Observation.hpp>

namespace oppt {
class AudioClassificationGraspingObservation: public VectorObservation {
public:
	AudioClassificationGraspingObservation(VectorFloat& observationVec):
		VectorObservation(observationVec) {

	}

	AudioClassificationGraspingObservation(const VectorFloat& observationVec):
		VectorObservation(observationVec) {

	}

	virtual ~AudioClassificationGraspingObservation() {}

	virtual FloatType distanceTo(const Observation& otherObservation) const override {

		FloatType dist = 0.0;


		VectorFloat observationVec = static_cast<const AudioClassificationGraspingObservation &>(otherObservation).asVector();
		
		for (int i = 0; i < 2; ++i) 
		{
			if (i=0)
			{
				dist += std::pow((observationVec_[i] - observationVec[i])/20.0, 2);
			}
			else
			{
				dist += std::pow((observationVec_[i] - observationVec[i])*20.0, 2);	
			}
		}
		

		return sqrt(dist);	

	}

protected:
	FloatType diffAngles(const FloatType& a1, const FloatType& a2) const {
		return atan2(sin(a1 - a2), cos(a1 - a2));
	}

};
}

#endif