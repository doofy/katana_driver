/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2010  University of Osnabrück
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * katana_trajectory_filter.cpp
 *
 *  Created on: 04.02.2011
 *      Author: Martin Günther <mguenthe@uos.de>
 *      Author: Michael Stypa <mstypa@uos.de>
 */

#include <ros/ros.h>
#include <class_loader/class_loader.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
//#include <moveit/robot_state/conversions.h>

namespace katana_planner_request_adapters
{
  class TrajectoryFilter : public planning_request_adapter::PlanningRequestAdapter
  {
    private:
      /// the maximum number of points that the output trajectory should have
      static const size_t MAX_NUM_POINTS = 16;

      /**
       *  How many points to delete at once?
       *
       *  The two extreme cases are:
       *   - DELETE_CHUNK_SIZE = 1:   each point will be deleted separately, then
       *                              the durations will be recomputed (most accurate)
       *   - DELETE_CHUNK_SIZE = INF: all points will be deleted at once (fastest)
       */
      static const size_t DELETE_CHUNK_SIZE = 10;

      bool smooth(const robot_trajectory::RobotTrajectoryPtr &trajectory_in,
          robot_trajectory::RobotTrajectory &trajectory_out) const
      {
        size_t num_points = trajectory_in->getWayPointCount();
        //trajectory_out = trajectory_in;

        //if (!spline_smoother::checkTrajectoryConsistency(trajectory_out))
          //return false;
        //TODO port to planningscene.ispathvalid()

        if (num_points <= MAX_NUM_POINTS)
          // nothing to do
          return true;

        while (true)
        {
          size_t num_points_delete = num_points - MAX_NUM_POINTS;

        if (num_points_delete > DELETE_CHUNK_SIZE)
            num_points_delete = DELETE_CHUNK_SIZE;
          else if (num_points_delete <= 0)
            break;

          remove_smallest_segments(trajectory_in, trajectory_out, num_points_delete);
          num_points = trajectory_out.getWayPointCount();
        }

        // delete all velocities and accelerations so they will be re-computed by Katana
        for (size_t i = 0; i < trajectory_out.getWayPointCount(); ++i)
        {
          // TODO check if port to setVariableVelocities is correct
          //trajectory_out.getWayPoint(i).setVariableVelocities(0);
          // TODO check if port to setVariableAccelerations is correct
          //trajectory_out.getWayPoint(i).setVariableAccelerations(0);
        }
        return true;
      }

      void remove_smallest_segments(
          const robot_trajectory::RobotTrajectoryPtr &trajectory_in,
          robot_trajectory::RobotTrajectory &trajectory_out,
          const size_t num_points_delete) const
      {
        size_t i;
        size_t num_points = trajectory_in->getWayPointCount();
        std::vector<std::pair<size_t, double> > segment_durations(num_points - 1);

        // calculate segment_durations
        for (i = 0; i < num_points; ++i)
        {
          double duration = trajectory_in->getWayPointDurationFromPrevious(i);
          ROS_INFO("duration = %f", duration);

          //segment_durations[i] = std::pair<size_t, double>(i, duration);
        }

        //for (size_t i = 0; i < segment_durations.size(); i++)
          //ROS_INFO("segment_durations[%3zu] = <%3zu, %f>", i, segment_durations[i].first, segment_durations[i].second);

        ROS_INFO("out count %zu", trajectory_out.getWayPointCount());
        trajectory_out.clear();
        for (i = 0; i < 16; i++) {
          trajectory_out.addSuffixWayPoint(trajectory_in->getWayPointPtr(i), 1);
        }

        ROS_INFO("out count %zu", trajectory_out.getWayPointCount());
/*
        // sort segment_durations by their duration, in ascending order
        std::vector<std::pair<size_t, double> > sorted_segment_durations = segment_durations;
        std::sort(sorted_segment_durations.begin(),
sorted_segment_durations.end(),
boost::bind(&std::pair<size_t, double>::second, _1) < boost::bind(&std::pair<size_t, double>::second, _2));

        for (size_t i = 0; i < sorted_segment_durations.size(); i++)
          ROS_INFO("sorted_segment_durations[%3zu] = <%3zu, %f>", i, sorted_segment_durations[i].first, sorted_segment_durations[i].second);

        // delete the smallest segments
        std::set<size_t> delete_set;
        for (size_t i = 0; i < num_points_delete; i++)
        {
          size_t point_to_delete = sorted_segment_durations[i].first;
          if (point_to_delete == 0)
          {
            // first segment too small -. merge right
            point_to_delete = 1;
          }
          else if (point_to_delete == num_points - 1)
          {
            // last segment too small -. merge left (default)
          }
          else
          {
            // some segment in the middle too small -. merge towards smaller segment
            if (segment_durations[point_to_delete - 1] > segment_durations[point_to_delete + 1])
              point_to_delete++;

            // note: this can lead to a situation where less than num_points_delete are actually deleted,
            // but we don't care
          }

          delete_set.insert(point_to_delete);
        }

        for (std::set<size_t>::iterator it = delete_set.begin(); it != delete_set.end(); it++)
          ROS_INFO("delete set entry: %zu", *it);

        //trajectory_out.clear();
        for (size_t i = 0; i < num_points; i++)
        {
          if (delete_set.find(i) == delete_set.end())
          {
            // segment i is not in the delete set -. copy
            //TODO fix second parameter: dt
          //double duration = trajectory_in->getWayPointDurationFromPrevious(i);
            trajectory_out.addSuffixWayPoint(trajectory_in->getWayPoint(i), 0);
          }
        }
*/
      }

    public:

      TrajectoryFilter() : planning_request_adapter::PlanningRequestAdapter()
    {
      }

      virtual std::string getDescription() const { return "Reduce trajectory points"; }


      virtual bool adaptAndPlan(const PlannerFn &planner,
          const planning_scene::PlanningSceneConstPtr& planning_scene,
          const planning_interface::MotionPlanRequest &req,
          planning_interface::MotionPlanResponse &res,
          std::vector<std::size_t> &added_path_index) const
      {
        bool result = false;

        ROS_INFO("Running '%s'", getDescription().c_str());

        result = planner(planning_scene, req, res);

        //ROS_INFO("planner result %d", result);

        ROS_INFO("Input Trajectory WayPoint Count: %zu", res.trajectory_->getWayPointCount());

        const robot_model::RobotModelConstPtr &kmodel = res.trajectory_->getRobotModel();
        const std::string &group = res.trajectory_->getGroupName();
        robot_trajectory::RobotTrajectory newtrajectory(kmodel, group);

        result = smooth(res.trajectory_, newtrajectory);

        res.trajectory_->swap(newtrajectory);

        ROS_INFO("Output Trajectory WayPoint Count: %zu", res.trajectory_->getWayPointCount());

        return result;
      }
  };
} // namespace katana_planning_request_adapters

CLASS_LOADER_REGISTER_CLASS(katana_planner_request_adapters::TrajectoryFilter,
    planning_request_adapter::PlanningRequestAdapter);
