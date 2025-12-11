#include "depth_capture.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include <iostream>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>

namespace mujoco::plugin::sensor
{
    namespace
    {
        // Checks that a plugin config attribute is a valid number
        bool CheckAttr(const std::string& input) {
            if (input.empty()) {
                std::cerr << "[DepthCapture] CheckAttr: input is empty" << std::endl;
                return false;
            }

            char* end;
            std::string value = input;
            // Remove whitespace - use lambda for proper function signature
            value.erase(std::remove_if(value.begin(), value.end(),
                [](unsigned char c) { return std::isspace(c); }), value.end());

            if (value.empty()) {
                std::cerr << "[DepthCapture] CheckAttr: value is empty after removing whitespace" << std::endl;
                return false;
            }

            strtod(value.c_str(), &end);
            bool valid = (end == value.data() + value.size());

            if (!valid) {
                std::cerr << "[DepthCapture] CheckAttr: '" << input << "' is not a valid number" << std::endl;
            }

            return valid;
        }

        // Safely get plugin config with null check
        const char* GetPluginConfigSafe(const mjModel* m, int instance, const char* attr) {
            const char* value = mj_getPluginConfig(m, instance, attr);
            if (!value) {
                std::cerr << "[DepthCapture] GetPluginConfigSafe: attribute '" << attr
                         << "' not found for instance " << instance << std::endl;
            }
            return value;
        }
    }

    DepthCapture* DepthCapture::Create(const mjModel* m, mjData* d, int instance)
    {
        std::cout << "[DepthCapture] Create: Starting creation for instance " << instance << std::endl;

        // Get all config values with null checks
        const char* ncol_str = GetPluginConfigSafe(m, instance, "ncol");
        const char* nrow_str = GetPluginConfigSafe(m, instance, "nrow");
        const char* fov_x_str = GetPluginConfigSafe(m, instance, "fov_x");
        const char* fov_y_str = GetPluginConfigSafe(m, instance, "fov_y");
        const char* max_distance_str = GetPluginConfigSafe(m, instance, "max_distance");

        // ncol - required parameter
        int ncol = 10;  // default value
        if (ncol_str && CheckAttr(std::string(ncol_str))) {
            ncol = strtod(ncol_str, nullptr);
            std::cout << "[DepthCapture] Create: ncol = " << ncol << std::endl;
        } else {
            std::cout << "[DepthCapture] Create: ncol not specified or invalid, using default = " << ncol << std::endl;
        }

        if (ncol <= 0) {
            std::cerr << "[DepthCapture] Create ERROR: 'ncol' must be positive, got " << ncol << std::endl;
            mju_error("'ncol' must be positive");
            return nullptr;
        }

        // nrow - required parameter
        int nrow = 10;  // default value
        if (nrow_str && CheckAttr(std::string(nrow_str))) {
            nrow = strtod(nrow_str, nullptr);
            std::cout << "[DepthCapture] Create: nrow = " << nrow << std::endl;
        } else {
            std::cout << "[DepthCapture] Create: nrow not specified or invalid, using default = " << nrow << std::endl;
        }

        if (nrow <= 0) {
            std::cerr << "[DepthCapture] Create ERROR: 'nrow' must be positive, got " << nrow << std::endl;
            mju_error("'nrow' must be positive");
            return nullptr;
        }

        // fov_x - required parameter
        mjtNum fov_x = 45.0;  // default value
        if (fov_x_str && CheckAttr(std::string(fov_x_str))) {
            fov_x = strtod(fov_x_str, nullptr);
            std::cout << "[DepthCapture] Create: fov_x = " << fov_x << " degrees" << std::endl;
        } else {
            std::cout << "[DepthCapture] Create: fov_x not specified or invalid, using default = " << fov_x << " degrees" << std::endl;
        }

        if (fov_x <= 0 || fov_x > 180) {
            std::cerr << "[DepthCapture] Create ERROR: 'fov_x' must be in range (0, 180] degrees, got " << fov_x << std::endl;
            mju_error("'fov_x' must be in range (0, 180] degrees");
            return nullptr;
        }

        // fov_y - required parameter
        mjtNum fov_y = 45.0;  // default value
        if (fov_y_str && CheckAttr(std::string(fov_y_str))) {
            fov_y = strtod(fov_y_str, nullptr);
            std::cout << "[DepthCapture] Create: fov_y = " << fov_y << " degrees" << std::endl;
        } else {
            std::cout << "[DepthCapture] Create: fov_y not specified or invalid, using default = " << fov_y << " degrees" << std::endl;
        }

        if (fov_y <= 0 || fov_y > 180) {
            std::cerr << "[DepthCapture] Create ERROR: 'fov_y' must be in range (0, 180] degrees, got " << fov_y << std::endl;
            mju_error("'fov_y' must be in range (0, 180] degrees");
            return nullptr;
        }

        // max_distance - required parameter
        mjtNum max_distance = 10.0;  // default value
        if (max_distance_str && CheckAttr(std::string(max_distance_str))) {
            max_distance = strtod(max_distance_str, nullptr);
        } else {
            std::cerr << "[DepthCapture] Create: max_distance not specified or invalid, using default = " << max_distance << std::endl;
        }

        if (max_distance <= 0) {
            std::cerr << "[DepthCapture] Create ERROR: 'max_distance' must be positive, got " << max_distance << std::endl;
            mju_error("'max_distance' must be positive");
            return nullptr;
        }
        return new DepthCapture(m, d, instance, ncol, nrow, fov_x, fov_y, max_distance);
    }

    DepthCapture::DepthCapture(const mjModel* m, mjData* d, int instance,
                               int ncol, int nrow, mjtNum fov_x, mjtNum fov_y,
                               mjtNum max_distance)
        : ncol_(ncol),
          nrow_(nrow),
          fov_x_(fov_x),
          fov_y_(fov_y),
          max_distance_(max_distance)
    {
        // Make sure sensor is attached to a site
        int sensor_id = -1;
        int site_id = -1;
        for (int i = 0; i < m->nsensor; ++i) {
            if (m->sensor_type[i] == mjSENS_PLUGIN && m->sensor_plugin[i] == instance) {
                sensor_id = i;
                if (m->sensor_objtype[i] != mjOBJ_SITE) {
                    std::cerr << "[DepthCapture] Constructor ERROR: Sensor " << sensor_id
                             << " must be attached to a site, but objtype = " << m->sensor_objtype[i] << std::endl;
                    mju_error("DepthCapture sensor must be attached to a site");
                }

                site_id = m->sensor_objid[i];
                break;
            }
        }

        if (sensor_id == -1) {
            std::cerr << "[DepthCapture] Constructor WARNING: Could not find sensor for instance " << instance << std::endl;
        }

        // Pre-compute ray directions
        int nrays = ncol * nrow;
        ray_directions_.resize(nrays * 3);

        mjtNum fov_x_rad = fov_x * mjPI / 180.0;
        mjtNum fov_y_rad = fov_y * mjPI / 180.0;

        for (int row = 0; row < nrow; ++row) {
            for (int col = 0; col < ncol; ++col) {
                int idx = row * ncol + col;

                mjtNum u = (ncol > 1) ? (2.0 * col / (ncol - 1) - 1.0) : 0.0;
                mjtNum v = (nrow > 1) ? (2.0 * row / (nrow - 1) - 1.0) : 0.0;

                mjtNum angle_x = u * fov_x_rad / 2.0;
                mjtNum angle_y = v * fov_y_rad / 2.0;

                mjtNum* dir = &ray_directions_[idx * 3];
                dir[0] = std::tan(angle_x);
                dir[1] = std::tan(angle_y);
                dir[2] = -1.0;

                mjtNum len = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
                dir[0] /= len;
                dir[1] /= len;
                dir[2] /= len;
            }
        }
    }

    void DepthCapture::Reset(const mjModel* m, int instance)
    {
        // No state to reset
    }

    void DepthCapture::Compute(const mjModel* m, mjData* d, int instance)
    {
        mj_markStack(d);

        // Get sensor id
        int id = -1;
        for (id = 0; id < m->nsensor; ++id) {
            if (m->sensor_type[id] == mjSENS_PLUGIN &&
                m->sensor_plugin[id] == instance) {
                break;
            }
        }

        if (id >= m->nsensor) {
            std::cerr << "[DepthCapture] Compute ERROR: Could not find sensor for instance " << instance << std::endl;
            mj_freeStack(d);
            return;
        }

        // Get sensor data
        mjtNum* sensordata = d->sensordata + m->sensor_adr[id];
        int nrays = ncol_ * nrow_;

        // Get site id and frame
        int site_id = m->sensor_objid[id];
        mjtNum* site_pos = d->site_xpos + 3 * site_id;
        mjtNum* site_mat = d->site_xmat + 9 * site_id;
        auto body = m->site_bodyid[site_id];
        // Perform raycasting
        int hits = 0;
        for (int i = 0; i < nrays; ++i) {
            const mjtNum* dir_local = &ray_directions_[i * 3];

            mjtNum dir_world[3];
            mju_mulMatVec(dir_world, site_mat, dir_local, 3, 3);

            int geomid = -1;
            mjtByte geomgroup[mjNGROUP] = {1, 1, 0, 0, 0, 0};
            mjtNum dist = mj_ray(m, d, site_pos, dir_world, geomgroup, 1, body, &geomid);

            if (dist < 0 || dist > max_distance_) {
                sensordata[i] = max_distance_;
            } else {
                sensordata[i] = dist;
                hits++;
            }
        }

        mj_freeStack(d);
    }

    void DepthCapture::Visualize(const mjModel* m, mjData* d,
                                  const mjvOption* opt, mjvScene* scn,
                                  int instance)
    {
        // Optional visualization
    }

    void DepthCapture::RegisterPlugin()
    {
        mjpPlugin plugin;
        mjp_defaultPlugin(&plugin);

        plugin.name = "mujoco.sensor.depth_capture";
        plugin.capabilityflags |= mjPLUGIN_SENSOR;

        const char* attributes[] = {"ncol", "nrow", "fov_x", "fov_y", "max_distance"};
        plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
        plugin.attributes = attributes;

        plugin.nstate = +[](const mjModel* m, int instance) {
            return 0;
        };

        plugin.nsensordata = +[](const mjModel* m, int instance, int sensor_id) {
            const char* ncol_str = mj_getPluginConfig(m, instance, "ncol");
            int ncol = 10;
            if (ncol_str) {
                ncol = strtod(ncol_str, nullptr);
                if (ncol == 0) ncol = 10;
            }

            const char* nrow_str = mj_getPluginConfig(m, instance, "nrow");
            int nrow = 10;
            if (nrow_str) {
                nrow = strtod(nrow_str, nullptr);
                if (nrow == 0) nrow = 10;
            }

            int total = ncol * nrow;
            return total;
        };

        plugin.needstage = mjSTAGE_POS;

        plugin.init = +[](const mjModel* m, mjData* d, int instance) {
            auto* depthCapture = DepthCapture::Create(m, d, instance);
            if (!depthCapture) {
                std::cerr << "[DepthCapture] init callback ERROR: Failed to create DepthCapture instance" << std::endl;
                return -1;
            }

            d->plugin_data[instance] = reinterpret_cast<uintptr_t>(depthCapture);
            return 0;
        };

        plugin.destroy = +[](mjData* d, int instance) {
            delete reinterpret_cast<DepthCapture*>(d->plugin_data[instance]);
            d->plugin_data[instance] = 0;
        };

        plugin.reset = +[](const mjModel* m, mjtNum* plugin_state, void* plugin_data,
                           int instance) {
            auto* depthCapture = reinterpret_cast<class DepthCapture*>(plugin_data);
            depthCapture->Reset(m, instance);
        };

        plugin.compute =
            +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
                auto* depthCapture =
                    reinterpret_cast<class DepthCapture*>(d->plugin_data[instance]);
                depthCapture->Compute(m, d, instance);
            };

        plugin.visualize = +[](const mjModel* m, mjData* d, const mjvOption* opt,
                               mjvScene* scn, int instance) {
            auto* depthCapture =
                reinterpret_cast<class DepthCapture*>(d->plugin_data[instance]);
            depthCapture->Visualize(m, d, opt, scn, instance);
        };

        mjp_registerPlugin(&plugin);
    }

}  // namespace mujoco::plugin::sensor
