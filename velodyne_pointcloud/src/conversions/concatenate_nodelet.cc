
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "concatenate.h"

namespace velodyne_pointcloud
{
  class ConcatenateNodelet: public nodelet::Nodelet
  {
  public:

    ConcatenateNodelet() {}
    ~ConcatenateNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<Concatenate> concat_;
  };

  /** @brief Nodelet initialization. */
  void ConcatenateNodelet::onInit()
  {
    concat_.reset(new Concatenate(getNodeHandle(), getPrivateNodeHandle()));
  }

} // namespace velodyne_pointcloud


// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_pointcloud, ConcatenateNodelet,
                        velodyne_pointcloud::ConcatenateNodelet,
                        nodelet::Nodelet);
