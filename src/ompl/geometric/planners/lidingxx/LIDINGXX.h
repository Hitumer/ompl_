#ifndef OMPL_GEOMETRIC_PLANNERS_LIDINGXX_LIDINGXX_
#define OMPL_GEOMETRIC_PLANNERS_LIDINGXX_LIDINGXX_


#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor LIDINGXX blabla
           @par Short description
           RRT is a tree-based motion planner that uses the following
           idea: RRT samples a random state @b qr in the state space,
           then finds the state @b qc among the previously seen states
           that is closest to @b qr and expands from @b qc towards @b
           qr, until a state @b qm is reached. @b qm is then added to
           the exploration tree.
           @par External documentation
           J. Kuffner and S.M. LaValle, RRT-connect: An efficient approach to single-query path planning, in <em>Proc.
           2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 995–1001, Apr. 2000. DOI:
           [10.1109/ROBOT.2000.844730](http://dx.doi.org/10.1109/ROBOT.2000.844730)<br>
           [[PDF]](http://ieeexplore.ieee.org/ielx5/6794/18246/00844730.pdf?tp=&arnumber=844730&isnumber=18246)
           [[more]](http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html)
        */   
            
        class LIDINGXX : public base::Planner
        {
        public:
    
            LIDINGXX(const base::SpaceInformationPtr &si, bool adaptiveRange = true);

    
            ~LIDINGXX(void) override;

            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            double getRange() const
            {
                return maxDistance_;
            }

            bool getIntermediateStates()const
            {
                return addIntermediateStates_;
            }

            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            /** \brief set different nearest neighbors*/
            template<template<typename T>class NN>
            void setNearestNeighbors()
            {
                if ((nn_ && nn_->size() !=0))
                    OMPL_WARN("calling setNearestNeighbors will clear all states,,,");
                clear();
                nn_ = std::make_shared<NN<Motion*>>();
                // tGoal_ = std::make_shared<NN<Motion*>>();
                setup();
            }

            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            double getGoalBias() const
            {
                return goalBias_;
            }

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            bool getAdaptiveRange()const
            {
                return adaptiveRange_;
            }

            void setAdaptiveRange(bool adaptiveRange)
            {
                adaptiveRange_ = adaptiveRange;
            }
            virtual void clear(void);
    
            // optional, if additional setup/configuration is needed, the setup() method can be implemented
            virtual void setup(void);
    
            virtual void getPlannerData(base::PlannerData &data) const;

        protected:


            class Motion
            {
            public:
                Motion()=default;

                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {}
                
                ~Motion() = default;

                const base::State *root{nullptr};
                base::State *state{nullptr};
                Motion *parent{nullptr};         
            };


            /** \brief A nearest-neighbor datastructure representing a tree of motions */
            using TreeData = std::shared_ptr<NearestNeighbors<Motion *>>;

            struct TreeGrowingInfo
            {
                base::State *xstate;
                Motion *xmotion;
                bool start;
            };
            

            enum GrowState
            {
                TRAPPED,
                ADVANCED,
                REACNED
            };

            GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion);


            /** \brief State sampler*/
            base::StateSamplerPtr sampler_;

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            double maxDistance_{0.};

            double adaptive_range(double maxDistance_);
            /** \brief The start tree */
            TreeData tStart_;

            TreeData nn_;
            
            /** \brief The goal tree */
            TreeData tGoal_;

            bool startTree_ {true};

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{.05};

            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};

            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool addIntermediateStates_;

            bool adaptiveRange_;

            /** \brief The pair of states in each tree connected during planning.  Used for PlannerData computation */
            std::pair<base::State *, base::State *> connectionPoint_;

            /** \brief Distance between the nearest pair of start tree and goal tree nodes. */
            double distanceBetweenTrees_;

            /** \brief The random number generator */
            RNG rng_;

        };
        
    }
}

#endif