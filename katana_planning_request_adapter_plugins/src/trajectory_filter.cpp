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
//#include <moveit/robot_state/conversions.h>

namespace katana_planning_request_adapters
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

      bool smooth(const robot_trajectory::RobotTrajectoryPtr trajectory_in,
          robot_trajectory::RobotTrajectoryPtr trajectory_out) const
      {

        size_t num_points = trajectory_in.getWayPointCount();
        trajectory_out = trajectory_in;

        //if (!spline_smoother::checkTrajectoryConsistency(trajectory_out))
          //return false;
        //TODO port to planningscene.ispathvalid()

        if (num_points <= MAX_NUM_POINTS)
          // nothing to do
          return true;

        while (true)
        {
          const robot_trajectory::RobotTrajectoryPtr
            trajectory_mid = trajectory_out;

          size_t num_points_delete = num_points - MAX_NUM_POINTS;

          if (num_points_delete > DELETE_CHUNK_SIZE)
            num_points_delete = DELETE_CHUNK_SIZE;
          else if (num_points_delete <= 0)
            break;

          remove_smallest_segments(trajectory_mid, trajectory_out, num_points_delete);
          num_points = trajectory_out.getWayPointCount();
        }

        // delete all velocities and accelerations so they will be re-computed by Katana
        for (size_t i = 0; i < trajectory_out.getWayPointCount(); ++i)
        {
          // TODO check if port to setVariableVelocities is correct
          trajectory_out.getWayPoint(i).setVariableVelocities(0);
          // TODO check if port to setVariableAccelerations is correct
          trajectory_out.getWayPoint(i).setVariableAccelerations(0);
        }

        return true;
      }

      void remove_smallest_segments(
          const robot_trajectory::RobotTrajectoryPtr trajectory_in,
          robot_trajectory::RobotTrajectoryPtr trajectory_out,
          const size_t num_points_delete) const
      {

        size_t num_points = trajectory_in.request.trajectory.points.size();
        std::vector<std::pair<size_t, double> > segment_durations(num_points - 1);

        // calculate segment_durations
        for (size_t i = 0; i < num_points - 1; ++i)
        {
          double duration = (trajectory_in.request.trajectory.points[i + 1].time_from_start
              - trajectory_in.request.trajectory.points[i].time_from_start).toSec();

          segment_durations[i] = std::pair<size_t, double>(i, duration);
        }

        for (size_t i = 0; i < segment_durations.size(); i++)
          ROS_DEBUG("segment_durations[%3zu] = <%3zu, %f>", i, segment_durations[i].first, segment_durations[i].second);

        // sort segment_durations by their duration, in ascending order
        std::vector<std::pair<size_t, double> > sorted_segment_durations = segment_durations;
        std::sort(sorted_segment_durations.begin(), sorted_segment_durations.end(), boost::bind(&std::pair<size_t, double>::second, _1)
            < boost::bind(&std::pair<size_t, double>::second, _2));

        for (size_t i = 0; i < sorted_segment_durations.size(); i++)
          ROS_DEBUG("sorted_segment_durations[%3zu] = <%3zu, %f>", i, sorted_segment_durations[i].first, sorted_segment_durations[i].second);

        // delete the smallest segments
        std::set<size_t> delete_set;
        for (size_t i = 0; i < num_points_delete; i++)
        {
          size_t point_to_delete = sorted_segment_durations[i].first;
          if (point_to_delete == 0)
          {
            // first segment too small --> merge right
            point_to_delete = 1;
          }
          else if (point_to_delete == num_points - 1)
          {
            // last segment too small --> merge left (default)
          }
          else
          {
            // some segment in the middle too small --> merge towards smaller segment
            if (segment_durations[point_to_delete - 1] > segment_durations[point_to_delete + 1])
              point_to_delete++;

            // note: this can lead to a situation where less than num_points_delete are actually deleted,
            // but we don't care
          }

          delete_set.insert(point_to_delete);
        }

        for (std::set<size_t>::iterator it = delete_set.begin(); it != delete_set.end(); it++)
          ROS_DEBUG("delete set entry: %zu", *it);

        trajectory_out.request.trajectory.points.resize(0);
        for (size_t i = 0; i < num_points; i++)
        {
          if (delete_set.find(i) == delete_set.end())
          {
            // segment i is not in the delete set --> copy
            trajectory_out.request.trajectory.points.push_back(trajectory_in.request.trajectory.points[i]);
          }
        }
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

        ROS_DEBUG("Running '%s'", getDescription().c_str());

        result = smooth();

        return result;
      }
  }
} // namespace katana_planning_request_adapters

CLASS_LOADER_REGISTER_CLASS(katana_planning_request_adapters::TrajectoryFilter,
    planning_request_adapter::PlanningRequestAdapter);
