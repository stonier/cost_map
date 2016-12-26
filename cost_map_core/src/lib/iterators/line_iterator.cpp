/**
 * @file /cost_map_core/src/lib/iterators/line_iterator.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include <cost_map_core/iterators/line_iterator.hpp>
#include <grid_map_core/GridMapMath.hpp>


using namespace std;

namespace cost_map {

LineIterator::LineIterator(const cost_map::CostMap& cost_map, const Position& start, const Position& end)
{
  Index startIndex, endIndex;
  if (getIndexLimitedToMapRange(cost_map, start, end, startIndex) &&
      getIndexLimitedToMapRange(cost_map, end, start, endIndex))
  {
    initialize(cost_map, startIndex, endIndex);
  }
}

LineIterator::LineIterator(const cost_map::CostMap& cost_map, const Index& start, const Index& end)
{
  initialize(cost_map, start, end);
}

LineIterator& LineIterator::operator =(const LineIterator& other)
{
  index_ = other.index_;
  start_ = other.start_;
  end_ = other.end_;
  iCell_ = other.iCell_;
  nCells_ = other.nCells_;
  increment1_ = other.increment1_;
  increment2_ = other.increment2_;
  denominator_ = other.denominator_;
  numerator_ = other.numerator_;
  numeratorAdd_ = other.numeratorAdd_;
  mapLength_ = other.mapLength_;
  mapPosition_ = other.mapPosition_;
  resolution_ = other.resolution_;
  bufferSize_ = other.bufferSize_;
  bufferStartIndex_ = other.bufferStartIndex_;
  return *this;
}

bool LineIterator::operator !=(const LineIterator& other) const
{
  return (index_ != other.index_).any();
}

const Index& LineIterator::operator *() const
{
  return index_;
}

LineIterator& LineIterator::operator ++()
{
  numerator_ += numeratorAdd_;  // Increase the numerator by the top of the fraction
  if (numerator_ >= denominator_) {
    numerator_ -= denominator_;
    index_ += increment1_;
  }
  index_ += increment2_;
  ++iCell_;
  return *this;
}

bool LineIterator::isPastEnd() const
{
  return iCell_ >= nCells_;
}

bool LineIterator::initialize(const cost_map::CostMap& cost_map, const Index& start, const Index& end)
{
    start_ = start;
    end_ = end;
    mapLength_ = cost_map.getLength();
    mapPosition_ = cost_map.getPosition();
    resolution_ = cost_map.getResolution();
    bufferSize_ = cost_map.getSize();
    bufferStartIndex_ = cost_map.getStartIndex();
    Index submapStartIndex;
    Size submapBufferSize;
    initializeIterationParameters();
    return true;
}

bool LineIterator::getIndexLimitedToMapRange(const cost_map::CostMap& cost_map,
                                             const Position& start, const Position& end,
                                             Index& index)
{
  Position newStart = start;
  Vector direction = (end - start).normalized();
  while (!cost_map.getIndex(newStart, index)) {
    newStart += (cost_map.getResolution() - std::numeric_limits<double>::epsilon()) * direction;
    if ((end - newStart).norm() < cost_map.getResolution() - std::numeric_limits<double>::epsilon())
      return false;
  }
  return true;
}

void LineIterator::initializeIterationParameters()
{
  iCell_ = 0;
  index_ = start_;

  Size delta = (end_ - start_).abs();

  if (end_.x() >= start_.x()) {
    // x-values increasing.
    increment1_.x() = 1;
    increment2_.x() = 1;
  } else {
    // x-values decreasing.
    increment1_.x() = -1;
    increment2_.x() = -1;
  }

  if (end_.y() >= start_.y()) {
    // y-values increasing.
    increment1_.y() = 1;
    increment2_.y() = 1;
  } else {
    // y-values decreasing.
    increment1_.y() = -1;
    increment2_.y() = -1;
  }

  if (delta.x() >= delta.y()) {
    // There is at least one x-value for every y-value.
    increment1_.x() = 0; // Do not change the x when numerator >= denominator.
    increment2_.y() = 0; // Do not change the y for every iteration.
    denominator_ = delta.x();
    numerator_ = delta.x() / 2;
    numeratorAdd_ = delta.y();
    nCells_ = delta.x() + 1; // There are more x-values than y-values.
  } else {
    // There is at least one y-value for every x-value
    increment2_.x() = 0; // Do not change the x for every iteration.
    increment1_.y() = 0; // Do not change the y when numerator >= denominator.
    denominator_ = delta.y();
    numerator_ = delta.y() / 2;
    numeratorAdd_ = delta.x();
    nCells_ = delta.y() + 1; // There are more y-values than x-values.
  }
}

} /* namespace cost_map */
