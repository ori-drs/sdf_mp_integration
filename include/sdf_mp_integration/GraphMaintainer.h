/**
 *  @file  GraphMaintainer.h
 *  @brief GraphMaintainer
 *  @author Mark Finean
 *  @date  04 February, 2021
 **/

#ifndef SDF_MP_INTEGRATION_GRAPHMAINTAINER_H
#define SDF_MP_INTEGRATION_GRAPHMAINTAINER_H

#include <gpmp2/planner/BatchTrajOptimizer.h>
#include <gpmp2/planner/TrajOptimizerSetting.h>
#include <gpmp2/config.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

namespace sdf_mp_integration {

  template <class ROBOT, class GP, class SDFHandler, class OBS_FACTOR, class OBS_FACTOR_GP, 
            class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
  class GraphMaintainer {

    private:
      gtsam::NonlinearFactorGraph graph_;
      double delta_t_, inter_dt_;
      gpmp2::TrajOptimizerSetting setting_;
      ROBOT arm_; 
      const SDFHandler* sdf_handler_;
      typename ROBOT::Pose start_conf_;
      typename ROBOT::Velocity start_vel_;
      typename ROBOT::Pose end_conf_;
      typename ROBOT::Velocity end_vel_;
      gtsam::Values init_values_;

    public:
      
      //  constructor
      GraphMaintainer() {};      
      
      GraphMaintainer(const ROBOT& arm, const SDFHandler& sdf_handler,
                      const typename ROBOT::Pose& start_conf, const typename ROBOT::Velocity& start_vel,
                      const typename ROBOT::Pose& end_conf, const typename ROBOT::Velocity& end_vel,
                      const gtsam::Values& init_values, const gpmp2::TrajOptimizerSetting& setting);      
      
      ~GraphMaintainer() {};


      // void constructGraph();

      gtsam::Values optimize();


  };
} // sdf_mp_integration namespace

#endif