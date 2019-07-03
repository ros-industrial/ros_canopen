#ifndef SOCKETCAN_INTERFACE_FILTER_H
#define SOCKETCAN_INTERFACE_FILTER_H

#include <vector>

#include "interface.h"

namespace can {

class FrameFilter {
public:
  virtual bool pass(const can::Frame &frame) const = 0;
  virtual ~FrameFilter() {}
};
using FrameFilterSharedPtr = std::shared_ptr<FrameFilter>;

class FrameMaskFilter : public FrameFilter {
public:
  static const uint32_t MASK_ALL = 0xffffffff;
  static const uint32_t MASK_RELAXED = ~Frame::EXTENDED_MASK;
  FrameMaskFilter(uint32_t can_id, uint32_t mask = MASK_RELAXED, bool invert = false)
  : mask_(mask), masked_id_(can_id & mask), invert_(invert)
  {}
  virtual bool pass(const can::Frame &frame) const{
    const uint32_t k = frame.key();
    return ((mask_ & k) == masked_id_) != invert_;
  }
private:
  const uint32_t mask_;
  const uint32_t masked_id_;
  const bool invert_;
};

class FrameRangeFilter  : public FrameFilter {
public:
  FrameRangeFilter(uint32_t min_id, uint32_t max_id, bool invert = false)
  : min_id_(min_id), max_id_(max_id), invert_(invert)
  {}
  virtual bool pass(const can::Frame &frame) const{
    const uint32_t k = frame.key();
    return (min_id_ <= k && k <= max_id_) != invert_;
  }
private:
  const uint32_t min_id_;
  const uint32_t max_id_;
  const bool invert_;
};

class FilteredFrameListener : public CommInterface::FrameListener {
public:
  using FilterVector = std::vector<FrameFilterSharedPtr>;
  FilteredFrameListener(CommInterfaceSharedPtr comm, const Callable &callable, const FilterVector &filters)
  : CommInterface::FrameListener(callable),
    filters_(filters),
    listener_(comm->createMsgListener([this](const Frame &frame) {
        for(FilterVector::const_iterator it=this->filters_.begin(); it != this->filters_.end(); ++it) {
          if((*it)->pass(frame)){
            (*this)(frame);
            break;
          }
        }
    }))
  {}
  const std::vector<FrameFilterSharedPtr> filters_;
  CommInterface::FrameListenerConstSharedPtr listener_;
};

} // namespace can

#endif /*SOCKETCAN_INTERFACE_FILTER_H*/
