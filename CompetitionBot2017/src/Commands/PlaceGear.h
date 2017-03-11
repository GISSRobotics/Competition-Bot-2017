#ifndef PlaceGear_H
#define PlaceGear_H

#include "../CommandBase.h"

class PlaceGear : public CommandBase {
public:
	PlaceGear();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // PlaceGear_H
