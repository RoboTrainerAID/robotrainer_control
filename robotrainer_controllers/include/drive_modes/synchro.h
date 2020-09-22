#ifndef SYNCHRO_MODE_H
#define SYNCHRO_MODE_H

#include "drive_modes/drive_mode_base_class.h"

// Simulates a platform with synchro drive: Translation is allowed, rotation is
// not.
class SynchroMode : public DriveMode {
    public:
        SynchroMode();

        // Sets va to zero. vx and vy remain same.
        void apply(double& vx, double& vy, double& va);
};

#endif
