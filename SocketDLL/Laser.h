#ifndef __ROBOTICS_LASER_H__
#define __ROBOTICS_LASER_H__

namespace Robotics {

	class Laser {
		public:
			virtual bool Open(const char *pAddr) = 0;
			virtual bool Scan(int *aScan) = 0;
			virtual bool Close() = 0;
	};

}

#endif