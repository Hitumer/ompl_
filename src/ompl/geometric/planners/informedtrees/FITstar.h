/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Technical University of Munich
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the University of Munich nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Authors: Marlin Strub, Liding Zhang, Xu Liang

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_FITSTAR_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_FITSTAR_

#include <memory>

#include "ompl/base/Cost.h"
#include "ompl/base/Planner.h"
#include "ompl/base/SpaceInformation.h"

#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/planners/informedtrees/fitstar/Direction.h"
#include "ompl/geometric/planners/informedtrees/fitstar/RandomGeometricGraph.h"
#include "ompl/geometric/planners/informedtrees/fitstar/ForwardQueue.h"
#include "ompl/geometric/planners/informedtrees/fitstar/ReverseQueue.h"

namespace ompl
{
    namespace geometric
    {
        /** \brief flexible Informed Trees (FIT*) */
        class FITstar : public ompl::base::Planner
        {
        public:
            /** \brief Constructs an instance of FIT* using the provided space information. */
            explicit FITstar(const std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo);

            /** \brief Destructs this instance of FIT*. */
            ~FITstar();

            /** \brief Setup the parts of the planner that rely on the problem definition being set. */
            void setup() override;

            /** \brief Solves the provided motion planning problem, respecting the given termination condition. */
            ompl::base::PlannerStatus
            solve(const ompl::base::PlannerTerminationCondition &terminationCondition) override;

            /** \brief Clears all internal planner structures but retains settings. Subsequent calls to solve() will
             * start from scratch. */
            void clear() override;

            /** \brief Clears all query-specific information, such as start and goal states and search trees. FIT*
             * retains the samples and collision checking cache of previous queries. */
            void clearQuery() override;

            /** \brief Returns the cost of the current best solution. */
            ompl::base::Cost bestCost() const;

            /** \brief Sets the number of samples per batch. */
            void setBatchSize(unsigned int numSamples);

            /** \brief Returns the number of samples per batch. */
            unsigned int getBatchSize() const;

            /** \brief adjust the number of samples per batch. */
            void adjustBatchSize();

            /** \brief Sets the initial number of collision checks on the reverse search. */
            void setInitialNumberOfSparseCollisionChecks(std::size_t numChecks);

            /** \brief Sets the radius factor. */
            void setRadiusFactor(double factor);

            /** \brief Returns the radius factor. */
            double getRadiusFactor() const;

            /** \brief adjust the radius factor. */
            void adjustRadiusFactor();

            /** \brief Sets the (initial) suboptimality factor. */
            void setSuboptimalityFactor(double factor);

            /** \brief Set whether pruning is enabled or not. */
            void enablePruning(bool prune);

            /** \brief Returns whether pruning is enabled or not. */
            bool isPruningEnabled() const;

            /** \brief Sets whether to track approximate solutions or not. */
            void trackApproximateSolutions(bool track);

            /** \brief Returns whether approximate solutions are tracked or not. */
            bool areApproximateSolutionsTracked() const;

            /** \brief Set whether to use a k-nearest RGG connection model. If false, FIT* uses an r-disc model. */
            void setUseKNearest(bool useKNearest);

            /** \brief Returns whether to use a k-nearest RGG connection model. If false, FIT* uses an r-disc model. */
            bool getUseKNearest() const;

            /** \brief Set whether to use a adaptive batch size RGG connection model. If false, FIT* uses static model.
             */
            void setUseAdaptiveBatchSize(bool useAdaptiveBatchSize);

            /** \brief Returns whether to use a adaptive batch size RGG connection model. If false, FIT* uses static
             * model. */
            bool getUseAdaptiveBatchSize() const;

            /** \brief Set the maximum number of goals FIT* will sample from sampleable goal regions. */
            void setMaxNumberOfGoals(unsigned int numberOfGoals);

            /** \brief Returns the maximum number of goals FIT* will sample from sampleable goal regions. */
            unsigned int getMaxNumberOfGoals() const;

            /** \brief Returns true if the forward queue is empty */
            bool isForwardQueueEmpty() const;

            /** \brief Returns a copy of the forward queue. */
            std::vector<fitstar::Edge> getForwardQueue() const;

            /** \brief Returns the effort of the edge at the top of the forward queue. */
            unsigned int getForwardEffort() const;

            /** \brief Returns true if the reverse queue is empty */
            bool isReverseQueueEmpty() const;

            /** \brief Returns a copy of the reverse queue. */
            std::vector<fitstar::Edge> getReverseQueue() const;

            /** \brief Returns copies of the edges in the reverse tree. */
            std::vector<fitstar::Edge> getReverseTree() const;

            /** \brief Returns the next edge in the forward queue. */
            fitstar::Edge getNextForwardEdge() const;

            /** \brief Returns the next edge in the reverse queue. */
            fitstar::Edge getNextReverseEdge() const;

            /** \brief Checks whether the state is a start state. */
            bool isStart(const std::shared_ptr<fitstar::State> &state) const;

            /** \brief Checks whether the state is a goal state. */
            bool isGoal(const std::shared_ptr<fitstar::State> &state) const;

            /** \brief Returns the planner data. */
            void getPlannerData(base::PlannerData &data) const override;

            /** \brief Set the seed used by the RNG and the StateSampler. The state sampler must already be allocated,
             * as a new state sampler will not take this seed. */
            void setLocalSeed(std::uint_fast32_t localSeed);

        protected:
            // ---
            // The settings that turn FIT* into EIRM*.
            // ---

            /** \brief Set wheter multiquery is enabled or not. */
            void enableMultiquery(bool multiquery);

            /** \brief Get wheter multiquery is enabled or not. */
            bool isMultiqueryEnabled() const;

            /** \brief Set start/goal pruning threshold. */
            void setStartGoalPruningThreshold(unsigned int threshold);

            /** \brief Get threshold at which we prune starts/goals. */
            unsigned int getStartGoalPruningThreshold() const;

        private:
            /** \brief Performs one iteration of FIT*. This either searches for a solution by advancing the forward
             * search, calculates more accurate heuristics by advancing the reverse search, or improves the current RGG
             * approximation by sampling more states. */
            void iterate(const ompl::base::PlannerTerminationCondition &terminationCondition);

            /** \brief Advances the reverse search by one iteration. */
            void iterateReverseSearch();

            /** \brief Advances the forward search by one iteration. */
            void iterateForwardSearch();

            /** \brief Improves the approximation by sampling more states and pruning states that cannot possibly
             * improve the current solution. */
            void improveApproximation(const ompl::base::PlannerTerminationCondition &terminationCondition);

            /** \brief Returns whether the to continue solving the problem or return the best solution. FIT* stops under
             *  three conditions:
             *    1. The termination condition is true; or
             *    2. The current solution satisfies the objective; or
             *    3. The current solution is the best possible solution given an admissible cost heuristic. */
            bool continueSolving(const ompl::base::PlannerTerminationCondition &terminationCondition) const;

            /** \brief Returns whether the reverse search must be continued. Check implementation comments for
             * suspension conditions. */
            bool continueReverseSearch() const;

            /** \brief Returns whether the forward search must be continued. Check implementation comments for
             * termination conditions. */
            bool continueForwardSearch() const;

            /** \brief Restarts the reverse search by resetting the reverse queue and search tree. */
            void restartReverseSearch();

            /** \brief Updates the exact solution by checking every goal in the graph. */
            void updateExactSolution();

            /** \brief Updates the approximate solution by checking all vertices in the forward search tree to find the
             * one closest to the goal in cost space. Updates the solution in the problem definition if appropriate. **/
            void updateApproximateSolution();

            /** \brief Updates the exact solution by checking all goals in the forward search tree. Also updates the
             * approximate solution by checking all vertices in the forward search tree, if approximate solutions are
             * being tracked. */
            ompl::base::PlannerStatus::StatusType updateSolution();

            /** \brief Ensures that the planner is setup. Returns the planner status ABORT if either the planner or the
             * state space cannot be setup and prints a error using OMPL_ERROR. */
            ompl::base::PlannerStatus::StatusType ensureSetup();

            /** \brief Ensures that the given problem has a start and goal state. Returns appropriate planner status and
             * prints an error using OMPL_ERROR. */
            ompl::base::PlannerStatus::StatusType
            ensureStartAndGoalStates(const ompl::base::PlannerTerminationCondition &terminationCondition);

            /** \brief Optimize the final path. */
            void finalPathOptimize(std::vector<std::shared_ptr<fitstar::State>> &states);

            /** \brief Constructs the forward path to a state. */
            std::shared_ptr<ompl::geometric::PathGeometric>
            getPathToState(const std::shared_ptr<fitstar::State> &state);

            /** \brief Updates the solution with a given goal state. */
            void updateExactSolution(const std::shared_ptr<fitstar::State> &goalState);

            /** \brief Checks whether the input vertex is the new best approximate solution and updates the solution in
             * the problem definition if so. **/
            void updateApproximateSolution(const std::shared_ptr<fitstar::State> &state);

            /** \brief Updates the current cost to come of a state using the information in the forward search tree. */
            void updateCurrentCostToCome(const std::shared_ptr<fitstar::State> &state);

            /** \brief Returns the cost to go to the goal. */
            ompl::base::Cost computeCostToGoToGoal(const std::shared_ptr<fitstar::State> &state) const;

            /** \brief Lets users know about a newly found solution via OMPL_INFORM. */
            void informAboutNewSolution() const;

            /** \brief Lets users know about the planner status via OMPL_INFORM. */
            void informAboutPlannerStatus(ompl::base::PlannerStatus::StatusType status) const;

            /** \brief Returns the number of vertices in the forward tree. */
            unsigned int countNumVerticesInForwardTree() const;

            /** \brief Returns the number of vertices in the reverse tree. */
            unsigned int countNumVerticesInReverseTree() const;

            /** \brief Returns all edges of the state in the RGG, as well as its parents and children. */
            std::vector<fitstar::Edge> expand(const std::shared_ptr<fitstar::State> &state, bool iterateForwardSearch) const;

            /** \brief Expands the state unless it is a goal. */
            std::vector<fitstar::Edge> expandUnlessGoal(const std::shared_ptr<fitstar::State> &state) const;

            /** \brief Returns whether the vertex has been closed during the current search. */
            bool isClosed(const std::shared_ptr<fitstar::Vertex> &vertex) const;

            /** \brief Returns whether the edge is in the forward tree. */
            bool isInForwardTree(const fitstar::Edge &edge) const;

            /** \brief Returns whether the edge is in the reverse tree. */
            bool isInReverseTree(const fitstar::Edge &edge) const;

            /** \brief Returns whether the edge can improve the reverse path. */
            bool doesImproveReversePath(const fitstar::Edge &edge) const;

            /** \brief Returns whether the edge can improve the reverse tree. */
            bool doesImproveReverseTree(const fitstar::Edge &edge, const ompl::base::Cost &edgeCost) const;

            /** \brief Returns whether the edge could improve the forward path according to an admissible heuristic. */
            bool couldImproveForwardPath(const fitstar::Edge &edge) const;

            /** \brief Returns whether the edge could improve the forward tree according to an admissible heuristic. */
            bool couldImproveForwardTree(const fitstar::Edge &edge) const;

            /** \brief Returns whether the edge does improve the forward path. */
            bool doesImproveForwardPath(const fitstar::Edge &edge, const ompl::base::Cost &edgeCost) const;

            /** \brief Returns whether the edge does improve the forward tree. */
            bool doesImproveForwardTree(const fitstar::Edge &edge, const ompl::base::Cost &edgeCost) const;

            /** \brief Returns the estimated cost to the target through the given edge. */
            ompl::base::Cost estimateCostToTarget(const fitstar::Edge &edge) const;

            /** \brief Returns the estimated effort to the target through the given edge. */
            unsigned int estimateEffortToTarget(const fitstar::Edge &edge) const;

            /** \brief Returns whether the edge is valid. */
            bool isValid(const fitstar::Edge &edge) const;

            /** \brief Returns whether the edge could be valid. Performs sparse collision detection on the edge. */
            bool couldBeValid(const fitstar::Edge &edge) const;

            /** \brief Returns whether the edge is valid at the given resolution, this function does the actual work of
             * checking states along the edge. */
            bool isValidAtResolution(const fitstar::Edge &edge, std::size_t numChecks) const;

            /** \brief Returns whether the cost is better than the other. */
            bool isBetter(const ompl::base::Cost &lhs, const ompl::base::Cost &rhs) const;

            /** \brief Combines two costs. */
            ompl::base::Cost combine(const ompl::base::Cost &lhs, const ompl::base::Cost &rhs) const;

            /** \brief Combines multiple costs. */
            template <typename... Costs>
            ompl::base::Cost combine(const ompl::base::Cost &cost, const Costs &... costs) const
            {
                return combine(cost, combine(costs...));
            }

            /** \brief Expands and inserts the forward roots into the forward queue. */
            void expandStartVerticesIntoForwardQueue();

            /** \brief Expands and inserts the reverse roots into the reverse queue. */
            void expandGoalVerticesIntoReverseQueue();

            /** \brief The sampling-based approximation of the state space. */
            fitstar::RandomGeometricGraph graph_;

            /** \brief The number of states added when the approximation is updated. */
            unsigned int batchSize_{199u};

            /** \brief Whether to use a adaptive batch size RGG. If false, FIT* uses an static batchsize. */
            bool useAdaptiveBatchSize_{true};

            /** \brief Max area of the sampled ellipse */
            double S_max_initial_{1.0};

            /** \brief Min area of the sampled ellipse */
            double S_min_initial_{0.0};

            /** \brief Max sample numbers */
            const unsigned int maxSamples_{399u};

            /** \brief Min sample numbers */
            const unsigned int minSamples_{1u};

            /** \brief Size of the sliding window for areas */
            const unsigned int historySize_{10u};

            /** \brief Deuqe to store recent areas */
            std::deque<double> areaHistory_;

            /** \brief vector to store all areas */
            std::unordered_map<double, int> areaList_;

            /** \brief The current suboptimality factor of the forward search. */
            double suboptimalityFactor_{std::numeric_limits<double>::infinity()};

            /** \brief The number of sparse collision detections on level 0. */
            std::size_t initialNumSparseCollisionChecks_{1u};

            /** \brief The number of sparse collision detections performed on the reverse search on the current level.
             */
            std::size_t numSparseCollisionChecksCurrentLevel_{1u};

            /** \brief The number of sparse collision detections performed on the reverse search on the previous level.
             */
            std::size_t numSparseCollisionChecksPreviousLevel_{0u};

            /** \brief Whether multiquery is enabled. */
            bool isMultiqueryEnabled_{false};

            /** \brief Whether pruning is enabled. */
            bool isPruningEnabled_{true};

            /** \brief Whether approximate solutions are tracked. */
            bool trackApproximateSolutions_{true};

            /** \brief The state used to do sparse collision detection with. */
            ompl::base::State *detectionState_;

            /** \brief The roots of the forward search tree (forest). */
            std::vector<std::shared_ptr<fitstar::Vertex>> startVertices_;

            /** \brief The roots of the reverse search tree (forest). */
            std::vector<std::shared_ptr<fitstar::Vertex>> goalVertices_;

            /** \brief The forward search queue. */
            std::unique_ptr<fitstar::ForwardQueue> forwardQueue_;

            /** \brief The reverse search queue. */
            std::unique_ptr<fitstar::ReverseQueue> reverseQueue_;

            /** \brief The current iteration. */
            std::size_t iteration_{0u};

            /** \brief The tag of the current reverse search. */
            std::size_t reverseSearchTag_{1u};

            /** \brief The tag of the current batch. */
            std::size_t startExpansionGraphTag_{0u};

            /** \brief An alias with a more expressive name to the problem of the base class. */
            std::shared_ptr<ompl::base::ProblemDefinition> &problem_ = ompl::base::Planner::pdef_;

            /** \brief An alias with a more expressive name to the space information of the base class. */
            std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo_ = ompl::base::Planner::si_;

            /** \brief A direct pointer to the state space. */
            std::shared_ptr<ompl::base::StateSpace> space_;

            /** \brief Direct access to the optimization objective of the problem. */
            std::shared_ptr<ompl::base::OptimizationObjective> objective_;

            /** \brief Direct access to the motion validator of the state space. */
            std::shared_ptr<ompl::base::MotionValidator> motionValidator_;

            /** \brief The cost of the current solution. */
            ompl::base::Cost solutionCost_{std::numeric_limits<double>::infinity()};

            /** \brief The cost of the last solution. */
            ompl::base::Cost lastsolutionCost_{std::numeric_limits<double>::infinity()};

            /** \brief The cost of the best reverse path. */
            ompl::base::Cost reverseCost_{std::numeric_limits<double>::signaling_NaN()};

            /** \brief The cost to come to the vertex that is closest to the goal (in cost space). */
            ompl::base::Cost approximateSolutionCost_{std::numeric_limits<double>::signaling_NaN()};

            /** \brief The cost to go to the goal from the current best approximate solution. */
            ompl::base::Cost approximateSolutionCostToGoal_{std::numeric_limits<double>::signaling_NaN()};

            /** \brief The number of processed edges. */
            mutable unsigned int numProcessedEdges_{0u};

            /** \brief The number of collision checked edges. */
            mutable unsigned int numCollisionCheckedEdges_{0u};
        };

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_FITSTAR_
