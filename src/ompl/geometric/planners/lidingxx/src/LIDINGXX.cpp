#include "ompl/geometric/planners/lidingxx/LIDINGXX.h"
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
 
ompl::geometric::LIDINGXX::LIDINGXX(const base::SpaceInformationPtr &si, bool adaptiveRange)
  : base::Planner(si,"LIDINGXX")
{
    specs_.approximateSolutions = true;
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;

    // Planner::declareParam<double>("range", this, &LIDINGXX::setRange,&LIDINGXX::getRange,"0.:10.:100.");
    Planner::declareParam<double>("goal_bias", this,&LIDINGXX::setGoalBias,&LIDINGXX::getGoalBias,"0.:.05:1.");
    Planner::declareParam<bool>("adaptive range", this,&LIDINGXX::setAdaptiveRange,&LIDINGXX::getAdaptiveRange,"0,1");

    adaptiveRange_ = adaptiveRange;
}

ompl::geometric::LIDINGXX::~LIDINGXX()
{
    freeMemory();
}


double ompl::geometric::LIDINGXX::adaptive_range(double ADVdistan) {
    if (rng_.uniformBool()) //sizeof(ptc)/2
    {
//        ADVdistan = 1*(1+rng_.uniform01());
        ADVdistan = 1;
        std::cout << ADVdistan <<"---------111-----------" << std::endl;
    }
    else
    {
//        ADVdistan = 5*(1+rng_.uniform01());
        ADVdistan = 5;
        std::cout << ADVdistan << "--------333------------" << std::endl;
    }

    return ADVdistan;
}

void ompl::geometric::LIDINGXX::setup()
{
    Planner::setup();
    if (!adaptiveRange_){
        tools::SelfConfig sc(si_, getName());
        sc.configurePlannerRange(maxDistance_);
    }


    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::LIDINGXX::freeMemory()
{
    if (nn_ )
    {
        std::vector<Motion *> motions;
        if (nn_) nn_->list(motions);
        for (auto &motion :motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}   

void ompl::geometric::LIDINGXX::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_) nn_->clear();

    lastGoalMotion_ = nullptr;
}




ompl::base::PlannerStatus ompl::geometric::LIDINGXX::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
    

    // while (const base::State *st = pis_.nextStart())
    // {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, pis_.nextStart());
        nn_->add(motion);
    // } // set start point, using while loop in case of multi-start
    
    if (!sampler_) sampler_ = si_->allocStateSampler();


    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
                (int)(nn_->size())); 

    Motion *approxsol{nullptr};
    Motion *solution{nullptr};
    double approxdif = std::numeric_limits<double>::infinity(); 
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state; // random state
    base::State *xstate = si_->allocState(); // allocstate memory for current state
    bool solved = false; 
    
    while (!ptc())
    {
        if (adaptiveRange_){
            tools::SelfConfig sc(si_, getName());
            maxDistance_ = adaptive_range(maxDistance_);
            sc.configurePlannerRange(maxDistance_);
        }

        if ((goal !=nullptr)&& rng_.uniform01()< goalBias_ &&goal->canSample()) goal->sampleGoal(rstate);
        else sampler_->sampleUniform(rstate);

        Motion *nmotion = nn_->nearest(rmotion); // find the nearest state from the tree
        base::State *dstate = rstate;
        base::State *nstate = nmotion->state;
        double d = si_->distance(nstate,rstate); // calculate the distance
        std::cout<< nn_->nearest(rmotion) << " =============nearest=================="<<std::endl;
        std::cout<< d << " =============distance=================="<<std::endl;  

        if(d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nstate, rstate, maxDistance_/d,xstate);
            dstate = xstate; 
        }
        
        if (si_->checkMotion(nstate,dstate))
        {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, dstate);
                motion->parent = nmotion;
                nn_->add(motion);

                nmotion = motion;

            double dist = 0.0;
            bool sat = pdef_->getGoal().get()->isSatisfied(nmotion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = nmotion;
                break;
            }
            if (dist <approxdif)
            {
                approxdif = dist;
                approxsol = nmotion;
            }
        }
    }


    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_= solution;
        std::vector<Motion *>mpath;

        // find path
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
            // std::cout<< mpath.front() << " =============mpath=front======"<<std::endl;
            // std::cout<< mpath.max_size() << " =============mpath=cend======"<<std::endl;
            // std::cout<< mpath.data() << " =============mpath=data======"<<std::endl;
            // std::cout<< solution << " =============mpath=solution======"<<std::endl;  
        }
        
        //set solution path
        auto path = std::make_shared<PathGeometric>(si_);
        for (int i =mpath.size()-1; i>= 0 ; --i) path->append(mpath[i]->state);
        pdef_->addSolutionPath(path,approximate,approxdif,getName());
        solved = true;
        std::cout<< mpath.size() << " =============mpath=================="<<std::endl;
        std::cout << path << "=========path print ==========" << std::endl;

    }


    si_->freeState(xstate);
    if (rstate !=nullptr) si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    if (approxsol && !solved)
    {
        std::vector<Motion*>mpath;
        while (approxsol != nullptr)
        {
        mpath.push_back(approxsol);
        approxsol = approxsol->parent;
        }

        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size()-1; i>=0 ; --i) path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, true,approxdif,getName());
        return base::PlannerStatus::APPROXIMATE_SOLUTION;
        // std::cout<< mpath.size() << " =============mpath=================="<<std::endl; 
    }
    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::LIDINGXX::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
    std::vector<Motion *> motions;
    if (nn_) nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}