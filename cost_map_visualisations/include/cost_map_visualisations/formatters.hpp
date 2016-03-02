/**
 * @file /cost_map_visualisations/include/cost_map_visualisations/formatters.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef cost_map_visualisations_FORMATTERS_HPP_
#define cost_map_visualisations_FORMATTERS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/formatters.hpp>
#include <grid_map/grid_map.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace grid_map {

/*****************************************************************************
** Interfaces
*****************************************************************************/

class Formatter {
public:
public:
  /**
   * @brief Default constructor.
   *
   * Initialises the format tags for width, and precision, and a layer to
   * format. Note this only formats a single layer.
   *
   * @param layer_name : look for this layer in any passed grid to format.
   * @param w : width (default - no width constraints)
   * @param p : the number of decimal places of precision (default - 4)
   **/
  Formatter(const std::string& layer_name, const int &w = -1, const unsigned int &p = 2)
  : format(w, p, ecl::RightAlign)
  , tmp_width(w)
  , tmp_precision(p)
  , tmp_formatting(false)
  , ready_to_format(false)
  , grid_(nullptr)
  , layer_name_(layer_name)
{}
  virtual ~Formatter() {}
  /**
   * @brief Sets the precision format parameter.
   *
   * @param p : the number of decimal places of precision.
   * @return Formatter& : this formatter readied for use with a stream.
   **/
  Formatter& precision( const unsigned int &p ) {
    format.precision(p);
    return *this;
  }
  /**
   * @brief Sets the width format parameter.
   *
   * Sets the width format parameter.
   *
   * @param w : the width to use for inserted floats (-1 is no width constraint).
   * @return FloatMatrixFormatter& : this formatter readied for use with a stream.
   **/
  Formatter& width( const int &w ) {
    format.width(w);
    return *this;
  }
  /**
   * @brief Returns the current precision setting.
   * @return unsigned int : the precision value.
   */
  unsigned int precision() { return format.precision(); }

  /**
   * @brief Returns the current width setting.
   * @return int : the witdh value (-1 for no width constraint).
   */
  int width() { return format.width(); }

  /**
   * @brief Format a sophus transform with permanently stored precision/width.
   *
   * This function directly formats the specified input value
   * with the stored settings.
   * @code
   * cout << format(nav_msgs::OccupancyGrid()) << endl;
   * @endcode
   * @param matrix : the matrix to be formatted (gets temporarily stored as a pointer).
   * @return FloatMatrixFormatter& : this formatter readied for use with a stream.
   **/
  Formatter& operator() (const nav_msgs::OccupancyGrid & grid ) {
    grid_ = &grid;
    ready_to_format = true;
    return (*this);
  }
  /**
   * @brief Format a matrix with temporarily specified precision/width.
   *
   * This function directly formats the specified input value
   * and temporary settings.
   *
   * @code
   * nav_msgs::OccupancyGrid grid;
   * cout << format(grid,3,-1) << endl; // precision of 3 and no width constraint
   * @endcode
   *
   * @param matrix : the matrix to be formatted (gets temporarily stored as a pointer).
   * @param w : the width to use for inserted floats (-1 is no width constraint).
   * @param p : the number of decimal places of precision.
   * @return FloatMatrixFormatter& : this formatter readied for use with a stream.
   **/
  Formatter& operator() (const nav_msgs::OccupancyGrid & grid, const int &w, const unsigned int &p) {
          grid_ = &grid;
          tmp_precision = p;
          tmp_width = w;
          tmp_formatting = true;
          ready_to_format = true;
          return (*this);
  }
  /**
   * @brief Stream the formatter.
   *
   * Insertion operator for sending the formatter  to an output stream.
   *
   * @param ostream : the output stream.
   * @param formatter : the formatter to be inserted.
   * @tparam OutputStream : the type of the output stream to be inserted into.
   * @tparam Derived : matrix type.
   * @return OutputStream : continue streaming with the updated output stream.
   *
   * @exception StandardException : throws if the formatter has un-specified _matrix [debug mode only]
   */
  template <typename OutputStream>
  friend OutputStream& operator << ( OutputStream& ostream, Formatter & formatter ) ecl_assert_throw_decl(ecl::StandardException);

private:
  ecl::Format<float> format;
  int tmp_width;
  unsigned int tmp_precision;
  bool tmp_formatting;
  bool ready_to_format;
  const nav_msgs::OccupancyGrid *grid_;
  const std::string layer_name_;
};

template <typename OutputStream>
OutputStream& operator << (OutputStream& ostream, Formatter & formatter ) ecl_assert_throw_decl(ecl::StandardException) {

  ecl_assert_throw( formatter.grid_, ecl::StandardException(LOC,ecl::UsageError,"The formatter cannot print any data - "
          "grid object was not initialised "
          "please pass the your grid through () operator") );

  ecl_assert_throw(formatter.ready_to_format, ecl::StandardException(LOC,ecl::UsageError,"The formatter cannot print any data - "
          "either there is no data available, or you have tried to use the "
          "formatter more than once in a single streaming operation. "
          "C++ produces unspecified results when functors are used multiply "
          "in the same stream sequence, so this is not permitted here.") );

  if ( formatter.ready_to_format ) {
    unsigned int prm_precision = formatter.format.precision();;
    int prm_width = formatter.format.width();
    if ( formatter.tmp_formatting ) {
      formatter.format.precision(formatter.tmp_precision);
      formatter.format.width(formatter.tmp_width);
    }

    /*********************
    ** Stream Grid Map
    **********************/
    grid_map::Index costmap_index;
    grid_map::Matrix& data = formatter.grid->get("totals");
    int count = 0;
    for (grid_map::GridMapIterator iterator(*(formatter.grid)); !iterator.isPastEnd(); ++iterator) {
      int i = (*iterator)(0);
      int j = (*iterator)(1);
      ostream << formatter.format(data(i,j));
      ++count;
      if ( count == formatter.grid->getSize().x() ) {
        count = 0;
        ostream << "\n";
      }
    }

    /********************
    ** Reset Variables
    ********************/
    if ( formatter.tmp_formatting ) {
      formatter.format.precision(prm_precision);
      formatter.format.width(prm_width);
      formatter.tmp_formatting = false;
    }
    formatter.ready_to_format = false;
  }
  return ostream;
}

} // namespace grid_map

/*****************************************************************************
 ** ECL Format Type
 *****************************************************************************/

namespace ecl {

  template <>
  class Format<nav_msgs::OccupancyGrid> : public grid_map::Formatter {
  public:
    Format() : grid_map::Formatter() {}
    Format(const int &w, const unsigned int &p) : grid_map::Formatter(w, p) {}
  };

} // namespace ecl

/*****************************************************************************
** Trailers
*****************************************************************************/

#endif /* cost_map_visualisations_FORMATTERS_HPP_ */
