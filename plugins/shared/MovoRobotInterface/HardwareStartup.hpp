#ifndef _HARDWARE_STARTUP_HPP_
#define _HARDWARE_STARTUP_HPP_
#include <oppt/opptCore/core.hpp>
#include "MovoAPI.hpp"

namespace oppt {
class HardwareStartup {
public:
	HardwareStartup();

	~HardwareStartup() = default;

	_NO_COPY_BUT_MOVE(HardwareStartup)

	bool startupSequence(const std::string &localIP);

	std::unique_ptr<movo::MovoAPI> releaseMovoAPI();

private:
	std::unique_ptr<movo::MovoAPI> movoAPI_ = nullptr;

	bool startupSequenceFinished_ = false;
};
}

#endif