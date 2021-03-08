#ifndef hectormapmutex_h__
#define hectormapmutex_h__

#include "util/MapLockerInterface.h"

#include <boost/thread/mutex.hpp>

namespace hector_mapping
{

class HectorMapMutex : public MapLockerInterface {
public:
  virtual void lockMap() {
    mapModifyMutex_.lock();
  }

  virtual void unlockMap() {
    mapModifyMutex_.unlock();
  }

  boost::mutex mapModifyMutex_;
};

}  // namespace hector_mapping

#endif
