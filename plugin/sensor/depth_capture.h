#ifndef MUJOCO_PLUGIN_SENSOR_DEPTH_CAPTURE_H_
#define MUJOCO_PLUGIN_SENSOR_DEPTH_CAPTURE_H_

#include <vector>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

namespace mujoco::plugin::sensor {

class DepthCapture {
 public:
  static DepthCapture* Create(const mjModel* m, mjData* d, int instance);
  DepthCapture(DepthCapture&&) = default;
  ~DepthCapture() = default;

  void Reset(const mjModel* m, int instance);
  void Compute(const mjModel* m, mjData* d, int instance);
  void Visualize(const mjModel* m, mjData* d, const mjvOption* opt,
                 mjvScene* scn, int instance);

  static void RegisterPlugin();

  int ncol_;
  int nrow_;
  mjtNum fov_x_;
  mjtNum fov_y_;
  mjtNum max_distance_;

 private:
  DepthCapture(const mjModel* m, mjData* d, int instance, int ncol, int nrow,
               mjtNum fov_x, mjtNum fov_y, mjtNum max_distance);
  std::vector<mjtNum> ray_directions_;
};

}  // namespace mujoco::plugin::sensor

#endif  // MUJOCO_PLUGIN_SENSOR_DEPTH_CAPTURE_H_
