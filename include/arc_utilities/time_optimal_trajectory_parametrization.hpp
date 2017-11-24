#include <stdlib.h>
#include <iostream>
#include <functional>
#include <Eigen/Geometry>
#include <vector>
#include <chrono>
#include <list>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/arc_helpers.hpp>

#ifndef TIME_OPTIMAL_TRAJECTORY_PARAMETRIZATION_HPP
#define TIME_OPTIMAL_TRAJECTORY_PARAMETRIZATION_HPP

namespace time_optimal_trajectory_parametrization
{
    class PathSegment
    {
    public:

        PathSegment(const double length = 0.0) : length_(length) {}

        virtual ~PathSegment() {}

        double getLength() const { return length_; }

        virtual Eigen::VectorXd getConfig(double s) const = 0;

        virtual Eigen::VectorXd getTangent(double s) const = 0;

        virtual Eigen::VectorXd getCurvature(double s) const = 0;

        virtual std::list<double> getSwitchingPoints() const = 0;

        virtual PathSegment* clone() const = 0;

        double position;

    protected:

        double length_;
    };



    class Path
    {
    public:
        Path(const std::list<Eigen::VectorXd>& path, double maxDeviation = 0.0);

        Path(const Path& path);

        ~Path();

        double getLength() const;

        Eigen::VectorXd getConfig(double s) const;

        Eigen::VectorXd getTangent(double s) const;

        Eigen::VectorXd getCurvature(double s) const;

        double getNextSwitchingPoint(double s, bool& discontinuity) const;

        std::list<std::pair<double, bool>> getSwitchingPoints() const;

    private:

        PathSegment* getPathSegment(double &s) const;

        double length;

        std::list<std::pair<double, bool> > switchingPoints;

        std::list<PathSegment*> pathSegments;
    };

    class Trajectory
    {
    public:
        // Generates a time-optimal trajectory
        Trajectory(const std::list<Eigen::VectorXd>& waypoints, const Eigen::VectorXd& max_velocity, const Eigen::VectorXd& max_acceleration, const double max_deviation, const double timestep);

        Trajectory(const Path &path, const Eigen::VectorXd &maxVelocity, const Eigen::VectorXd &maxAcceleration, double timeStep = 0.001);

        ~Trajectory(void);

        // Call this method after constructing the object to make sure the trajectory generation succeeded without errors.
        // If this method returns false, all other methods have undefined behavior.
        bool isValid() const;

        // Returns the optimal duration of the trajectory
        double getDuration() const;

        // Return the position/configuration or velocity vector of the robot for a given point in time within the trajectory.
        Eigen::VectorXd getPosition(double time) const;
        Eigen::VectorXd getVelocity(double time) const;

        // Outputs the phase trajectory and the velocity limit curve in 2 files for debugging purposes.
        void outputPhasePlaneTrajectory() const;

    private:
        struct TrajectoryStep {
            TrajectoryStep() {}
            TrajectoryStep(double pathPos, double pathVel) :
                pathPos(pathPos),
                pathVel(pathVel)
            {}
            double pathPos;
            double pathVel;
            double time;
        };

        bool getNextSwitchingPoint(double pathPos, TrajectoryStep &nextSwitchingPoint, double &beforeAcceleration, double &afterAcceleration);
        bool getNextAccelerationSwitchingPoint(double pathPos, TrajectoryStep &nextSwitchingPoint, double &beforeAcceleration, double &afterAcceleration);
        bool getNextVelocitySwitchingPoint(double pathPos, TrajectoryStep &nextSwitchingPoint, double &beforeAcceleration, double &afterAcceleration);
        bool integrateForward(std::list<TrajectoryStep> &trajectory, double acceleration);
        void integrateBackward(std::list<TrajectoryStep> &startTrajectory, double pathPos, double pathVel, double acceleration);
        double getMinMaxPathAcceleration(double pathPosition, double pathVelocity, bool max);
        double getMinMaxPhaseSlope(double pathPosition, double pathVelocity, bool max);
        double getAccelerationMaxPathVelocity(double pathPos) const;
        double getVelocityMaxPathVelocity(double pathPos) const;
        double getAccelerationMaxPathVelocityDeriv(double pathPos);
        double getVelocityMaxPathVelocityDeriv(double pathPos);

        std::list<TrajectoryStep>::const_iterator getTrajectorySegment(double time) const;

        Path path;
        Eigen::VectorXd maxVelocity;
        Eigen::VectorXd maxAcceleration;
        unsigned int n;
        bool valid;
        std::list<TrajectoryStep> trajectory;
        std::list<TrajectoryStep> endTrajectory; // non-empty only if the trajectory generation failed.

        static const double eps;
        const double timeStep;

        mutable double cachedTime;
        mutable std::list<TrajectoryStep>::const_iterator cachedTrajectorySegment;
    };
}

#endif // TIME_OPTIMAL_TRAJECTORY_PARAMETRIZATION_HPP
