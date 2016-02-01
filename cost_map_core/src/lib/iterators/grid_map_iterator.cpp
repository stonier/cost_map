/*
 * GridMapIterator.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <cost_map_core/CostMapMath.hpp>
#include <cost_map_core/iterators/CostMapIterator.hpp>


namespace cost_map {

CostMapIterator::CostMapIterator(const cost_map::CostMap& gridMap)
{
  size_ = gridMap.getSize();
  startIndex_ = gridMap.getStartIndex();
  endIndex_ = startIndex_ + gridMap.getSize() - Eigen::Array2i::Ones();
  mapIndexWithinRange(endIndex_, size_);
  index_ = startIndex_;
  isPastEnd_ = false;
}

CostMapIterator::CostMapIterator(const CostMapIterator* other)
{
  size_ = other->size_;
  startIndex_ = other->startIndex_;
  endIndex_ = other->endIndex_;
  index_ = other->index_;
  isPastEnd_ = other->isPastEnd_;
}

CostMapIterator& CostMapIterator::operator =(const CostMapIterator& other)
{
  size_ = other.size_;
  startIndex_ = other.startIndex_;
  endIndex_ = other.endIndex_;
  index_ = other.index_;
  isPastEnd_ = other.isPastEnd_;
  return *this;
}

bool CostMapIterator::operator !=(const CostMapIterator& other) const
{
  return (index_ != other.index_).any();
}

const Index& CostMapIterator::operator *() const
{
  return index_;
}

const Index CostMapIterator::getUnwrappedIndex() const
{
  return getIndexFromBufferIndex(index_, size_, startIndex_);
}

CostMapIterator& CostMapIterator::operator ++()
{
  isPastEnd_ = !incrementIndex(index_, size_, startIndex_);
  return *this;
}

CostMapIterator CostMapIterator::end() const
{
  CostMapIterator res(this);
  res.index_ = endIndex_;
  return res;
}

bool CostMapIterator::isPastEnd() const
{
  return isPastEnd_;
}

} /* namespace cost_map */
