#ifndef _MOVO_USER_DATA_HPP_
#define _MOVO_USER_DATA_HPP_
#include <oppt/opptCore/core.hpp>
#include <oppt/opptCore/RobotStateUserData.hpp>
#include <oppt/opptCore/geometric/Pose.hpp>

namespace oppt {
class AudioClassificationUserData: public RobotStateUserData {
public:
	AudioClassificationUserData() = default;

	virtual ~AudioClassificationUserData() = default;

	_NO_COPY_BUT_MOVE(AudioClassificationUserData)	

	geometric::Pose endEffectorPose;	

	virtual void print(std::ostream &os) const override {
		os << "endEffectorPose: " << endEffectorPose << endl;
	}

	virtual void serialize(std::ostream& os, const std::string &prefix = "") const override {
		os << prefix << ": " << endEffectorPose.toGZPose();
	}

	CollisionReportSharedPtr collisionReport = nullptr;
};
}

#endif