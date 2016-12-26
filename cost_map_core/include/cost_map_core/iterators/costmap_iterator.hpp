/**
 * @file /cost_map_core/include/cost_map_core/iterators/costmap_iterator.hpp
 */

#pragma once

#include <Eigen/Core>
#include "../cost_map.hpp"

namespace cost_map {

/*!
 * Iterator class to iterate trough the entire grid map.
 */
class CostMapIterator
{
public:

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   */
  CostMapIterator(const cost_map::CostMap &gridMap);

  /*!
   * Copy constructor.
   * @param other the object to copy.
   */
  CostMapIterator(const CostMapIterator* other);

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  CostMapIterator& operator =(const CostMapIterator& other);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const CostMapIterator& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const Index& operator *() const;

  /*!
   * Retrieve the index as unwrapped index, i.e., as the corresponding index of a
   * grid map with no circular buffer offset.
   */
  const Index getUnwrappedIndex() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  CostMapIterator& operator ++();

  /*!
   * Return the end iterator
   * @return the end iterator (useful when performing normal iterator processing with ++).
   */
  CostMapIterator end() const;

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPastEnd() const;

private:

  //! Size of the buffer.
  Size size_;

  //! Start index of the circular buffer.
  Index startIndex_;

  //! End index of the circular buffer.
  Index endIndex_;

  //! Current index.
  Index index_;

  //! Is iterator out of scope.
  bool isPastEnd_;
};

} /* namespace */
