#include <vector>
#include <memory>
#include <queue>
#include <functional>
#include "ompl/geometric/planners/informedtrees/fitstar/TotalForce.h"

#include <algorithm>
#include <limits>

#include "ompl/geometric/planners/informedtrees/fitstar/State.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ompl
{
    namespace geometric
    {
        namespace fitstar
        {

            TotalForce::TotalForce(const std::shared_ptr<State> &state,
                                             std::vector<std::shared_ptr<State>> &states, const double &dim)
              : state_(state), states_(states), dimension_(dim)
            {
            }

            TotalForce::~TotalForce()
            {
            }

            typedef std::vector<double> Vector;

            void TotalForce::setState(const std::shared_ptr<State> &state)
            {
                if (!state)
                {
                    throw std::invalid_argument("Provided state is null");
                }

                std::lock_guard<std::mutex> lock(state_mutex_);
                state_ = state;

                return;
            }

            std::shared_ptr<State> TotalForce::getState() const
            {
                return state_;
            }

            std::vector<std::shared_ptr<State>> TotalForce::getStates() const
            {
                return states_;
            }

            Vector TotalForce::getVector(const std::shared_ptr<State> state1,
                                              const std::shared_ptr<State> state2) const
            {
                if (!state1)
                {
                    throw std::invalid_argument("Provided state1 is null");
                }

                if (!state2)
                {
                    throw std::invalid_argument("Provided state2 is null");
                }

                auto rstate1 = state1->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
                auto rstate2 = state2->raw()->as<ompl::base::RealVectorStateSpace::StateType>();

                if (!rstate1 || !rstate2)
                {
                    throw std::runtime_error("rstate pointer is null");
                }

                std::vector<double> vector(dimension_);

                for (size_t i = 0; i < dimension_; ++i)
                {
                    vector[i] = rstate2->values[i] - rstate1->values[i];
                }

                return vector;
            }


            double TotalForce::distance(const std::shared_ptr<State> state1, const std::shared_ptr<State> state2) const
            {
                if (!state1)
                {
                    throw std::invalid_argument("Provided state1 is null");
                }

                if (!state2)
                {
                    throw std::invalid_argument("Provided state2 is null");
                }

                auto cstate1 = state1->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
                auto cstate2 = state2->raw()->as<ompl::base::RealVectorStateSpace::StateType>();

                if (!cstate1 || !cstate2)
                {
                    throw std::runtime_error("rstate pointer is null");
                }
                double theta1 = 0., theta2 = 0., dx = 0., dy = 0., dist = 0.;


                for (unsigned int i = 0; i < dimension_; ++i)
                {
                    theta1 += cstate1->values[i];
                    theta2 += cstate2->values[i];
                    dx += cos(theta1) - cos(theta2);
                    dy += sin(theta1) - sin(theta2);
                    dist += sqrt(dx * dx + dy * dy);
                }

                return dist;
            }

            struct Compare {
                bool operator()(const std::pair<double, std::shared_ptr<State>>& a,
                                const std::pair<double, std::shared_ptr<State>>& b) {
                    return a.first > b.first;
                }
            };

            std::vector<std::shared_ptr<State>> TotalForce::NearestKSamples(const std::shared_ptr<State> state, 
            std::vector<std::shared_ptr<State>> Samples,int k){
            std::priority_queue<std::pair<double, std::shared_ptr<State>>,std::vector<std::pair<double, std::shared_ptr<State>>>, Compare> pq;

                    // 遍历所有样本，计算与给定状态的距离，并存储在优先队列中
                    for (const auto& sample : Samples) {
                        double dist = distance(state,sample);
                        pq.push({dist, sample});
                    }

                    // 从队列中取出最近的 k 个样本
                    std::vector<std::shared_ptr<State>> nearest;
                    for (int i = 0; i < k && !pq.empty(); ++i) {
                        nearest.push_back(pq.top().second);
                        pq.pop();
                    }

                    return nearest;
            }


                std::vector<double> TotalForce::force(const std::shared_ptr<State>& state1,const std::shared_ptr<State>& state2) {
                     // 获取两状态之间的向量
                    std::vector<double> vec = getVector(state1, state2);
                        // 计算距离
                    double dist = distance(state1,state2);

                        // 库伦定律：F = k * (1 / r^2)
                    double magnitude = 1.0 / (dist * dist);
                    magnitude = state2->isAttractive_ ? magnitude : -magnitude ; // 引力为正，斥力为负

                    for (size_t i = 0; i < vec.size(); ++i) {
                        vec[i] *= magnitude;
                    }

                    return vec;
                }

            void TotalForce::totalForce(const std::shared_ptr<State>& currentState, const std::vector<std::shared_ptr<State>>& Samples) {

                    std::vector<double> totalForceVec(dimension_, 0.0);
                    // 计算有效样本的引力
                    for (const auto& sample : Samples) {
                        auto f = force(currentState, sample);
                        for (size_t i = 0; i < dimension_; ++i) {
                            totalForceVec[i] += f[i];   
                        }
                    }
                        double totalMagnitude = 0.0;
                        for (size_t i = 0; i < dimension_; ++i) {
                            totalMagnitude += totalForceVec[i] * totalForceVec[i];
                        }
                        totalMagnitude = sqrt(totalMagnitude);
                        totalMagnitude_ = totalMagnitude;
                        bool flag;
                        for (double elem : totalForceVec) {
                            if (std::abs(elem) > std::numeric_limits<double>::epsilon()) {
                                TotalForce::totalForceVec_ = normalize(totalForceVec);
                            }
                            else 
                            TotalForce::totalForceVec_ = totalForceVec;
                        }
                        
                        // 打印合力的大小和方向（如果dimension_为2）

                            // std::cout << "_________Total Force Magnitude: " << totalMagnitude << std::endl;
                            // for(int i = 0; i< totalForceVec.size();i++)
                            // std::cout << "_________Total Force Magnitude Direction: " << totalForceVec[i]/totalMagnitude << std::endl;
                            
                    }

            std::vector<std::shared_ptr<State>> TotalForce::NearestEllipseticKSamples(
                const std::shared_ptr<State> state,
                const std::vector<double> &totalForceVec,
                std::vector<std::shared_ptr<State>> &Samples,
                int k, std::size_t dimension_) {

                // Define a priority queue for all samples
                std::priority_queue<std::pair<double, std::shared_ptr<State>>,
                                    std::vector<std::pair<double, std::shared_ptr<State>>>,
                                    Compare> queue;

                // Iterate over all samples and add them to the queue
                for (const auto& sample : Samples) {
                    // Check if the current sample is not the same as the state
                    if (sample != state) {
                        double dist = calculateEllipticalDistance(state, sample, totalForceVec, dimension_);
                        // std::cout << "________dist ::  " << dist << std:: endl;
                        if (dist <= INFINITY) {
                            queue.push(std::make_pair(dist, sample));
                        }
                    }
                }
                // Extract samples until K positive ones are found
                std::vector<std::shared_ptr<State>> nearest;
                int positiveCount = 0;

                while (!queue.empty() && positiveCount < k) {
                    auto currentSample = queue.top().second;
                    queue.pop();
                    if (currentSample->isAttractive_) {
                        positiveCount++;
                    }
                    nearest.push_back(currentSample);
                }
                std::cout << "numbre of nearest::" << nearest.size() << std::endl;
                return nearest;
            }

            //  void TotalForce::settotalForcewithStartValue(std::vector<double>& totalForceVecWithStart){

            //     totalForceVecWithStart_ = totalForceVecWithStart;

            //  }

            void TotalForce::totalForcewithStart(const std::shared_ptr<State>& currentState, const std::shared_ptr<State>& startstate,  const std::shared_ptr<State>& goalstate, bool iterateForwardSearch) {
                
                std::vector<double> singleForceVec(dimension_, 0.0);
                std::vector<double> totalForceVecwithStart(dimension_, 0.0);
                std::vector<double> totalForceVec = TotalForce::totalForceVec_;

                if(iterateForwardSearch){

                    goalstate->setAttractive();                
                    // 计算有效样本的引力
                     if (currentState == goalstate){
                            singleForceVec = singleForceVec;

                     }
                    else{

                        auto f = force(currentState, goalstate);

                        singleForceVec =  normalize(f);
                             for (auto& element : singleForceVec) {
                                                element *= 1;
                                            }
                        }   
                 
                }

                else
                {
                    startstate->setAttractive();                
                    // 计算有效样本的引力
                     if (currentState == startstate){
                            singleForceVec = singleForceVec;
                     }
                    else{

                        auto f = force(currentState, startstate);

                        singleForceVec = normalize(f);

                        for (auto& element : singleForceVec) {
                                                element *= 1;
                                            }


                    }   
                       
                }
                for(int i = 0; i< totalForceVecwithStart.size();i++){ 
                        //     std::cout << "singleForceVec to start/goal:: " <<  singleForceVec[i] << std:: endl;
                   
                        }
              
                        totalForceVecwithStart= normalize(singleForceVec + totalForceVec); 

                        std::vector<double>vectorgoaltostart = getVector(goalstate, startstate);
                        vectorgoaltostart = normalize(vectorgoaltostart);
                        std::vector<double>vectorstarttogoal = getVector(startstate, goalstate);
                        vectorstarttogoal = normalize(vectorstarttogoal);


                        //  for(int i = 0; i< totalForceVecwithStart.size();i++){ 
                        //     std::cout << "_________vectorgoaltostart : " << vectorgoaltostart[i]<< std::endl;
                   
                        // }
                                for(int i = 0; i< totalForceVecwithStart.size();i++){ 
                           std::cout << "totalForceVec from the points : " << totalForceVec[i]<< std::endl;
                   
                        }
                                for(int i = 0; i< totalForceVecwithStart.size();i++){ 
                            std::cout << "befor project totalForceVecwithStart : " << totalForceVecwithStart[i]<< std::endl;
                   
                        }


                        if (!(totalForceVecwithStart.empty() || vectorgoaltostart.empty())){
                            if(iterateForwardSearch){
                                 totalForceVecwithStart = normalize(totalForceVecwithStart + vectorstarttogoal);
                            }
                            else{
                                 totalForceVecwithStart = normalize(totalForceVecwithStart + vectorgoaltostart);

                            }

                        }
                     
                        for(int i = 0; i< totalForceVecwithStart.size();i++){
                  
                            std::cout << "after project totalForceVecwithStart: " << totalForceVecwithStart[i]<< std::endl;

                        }
                // for(int i = 0; i < dimension_; i++){

                //     TotalForce::totalForceVecwithStart_[i] = totalForceVecwithStart[1-i];
                // }
                
                        TotalForce::totalForceVecwithStart_ = totalForceVecwithStart;
                
                }


                 std::vector<double>TotalForce:: getValueofforceDirection(){

                    return TotalForce::totalForceVecwithStart_;
                 }
                   
            

            // 椭圆距离计算函数
            double TotalForce::calculateEllipticalDistance(const std::shared_ptr<State>& state1, 
                                            const std::shared_ptr<State>& state2, 
                                            const std::vector<double>& Vector, std::size_t dimension_) {
                    
                if (!state1)
                {
                    throw std::invalid_argument("Provided state1 is null");
                }

                if (!state2)
                {
                    throw std::invalid_argument("Provided state2 is null");
                }

                auto cstate1 = state1->raw()->as<ompl::base::RealVectorStateSpace::StateType>();
                auto cstate2 = state2->raw()->as<ompl::base::RealVectorStateSpace::StateType>();

                if (!cstate1 || !cstate2)
                {
                    throw std::runtime_error("rstate pointer is null");
                }
                double theta1 = 0., theta2 = 0., dx = 0., dy = 0., dist = 0.;


                for (unsigned int i = 0; i < dimension_; ++i)
                {
                    theta1 += cstate1->values[i];
                    theta2 += cstate2->values[i];
                    dx += cos(theta1) - cos(theta2);
                    dy += sin(theta1) - sin(theta2);
                    //std::cout << "vector" << Vector[i] << std::endl;
                    dist += sqrt((dx * dx + dy * dy)/(Vector[i]*Vector[i]));
                }

                return dist;


            }


            double TotalForce::getNorm(const Vector &vector) const
            {
                // Safety check in case of Segmentation Fault
                if (vector.empty())
                {
                    throw std::invalid_argument("Privided vector is null");
                }

                double sum = 0.0;

                for (auto v : vector)
                {
                    sum += v * v;
                }

                return std::sqrt(sum);
            }



            double TotalForce::dotProduct(const Vector &v1, const Vector &v2) const
            {
                // Safety check in case of Segmentation Fault
                if (v1.empty())
                {
                    throw std::invalid_argument("Provided vector1 is null");
                }

                // Safety check in case of Segmentation Fault
                if (v2.empty())
                {
                    throw std::invalid_argument("Provided vector2 is null");
                }

                double dot_product = 0.0;

                for (size_t i = 0; i < dimension_; ++i)
                {
                    dot_product += v1[i] * v2[i];
                }

                return dot_product;
            }

            std::vector<double> TotalForce::vectorProjection(const std::vector<double>& a, const std::vector<double>& b) {
                            double dotAB = dotProduct(a, b);
                            double dotBB = dotProduct(b, b);
                            double scale = dotAB / dotBB;

                            std::vector<double> projection;
                            projection.reserve(b.size());
                            for (auto& element : b) {
                                projection.push_back(element * scale);
                            }
                            return projection;
                        }

            Vector TotalForce::normalize(const Vector &v) const
            {
                double norm = getNorm(v);

                if (std::fabs(norm) < epsilon_)
                {
                    throw std::runtime_error("Cannot normalize a zero vector");
                }

                return v / norm;
            }

            bool TotalForce::isVectorBetween(const Vector &targetVector, const Vector &goalVector,
                                                  const std::shared_ptr<State> &sourceState,
                                                  const std::shared_ptr<State> &neighborState) const
            {
                // Safety checks
                if (targetVector.empty())
                {
                    throw std::invalid_argument("Provided target vector is null");
                }
                if (!sourceState)
                {
                    throw std::invalid_argument("Provided sourceState is null");
                }
                if (!neighborState)
                {
                    throw std::invalid_argument("Provided neighborState is null");
                }

                Vector vectorTocheck = getVector(sourceState, neighborState);

                if (std::fabs(getNorm(targetVector)) < epsilon_ || std::fabs(getNorm(goalVector)) < epsilon_ ||
                    std::fabs(getNorm(vectorTocheck)) < epsilon_)
                {
                    return false;
                }

                // Normalize the vectors to focus on direction, not magnitude
                Vector normalizedTarget = normalize(targetVector);
                Vector normalizedGoal = normalize(goalVector);
                Vector normalizedToCheck = normalize(vectorTocheck);

                // Compute the dot products
                double dotProductWithTarget = dotProduct(normalizedToCheck, normalizedTarget);
                double dotProductWithGoal = dotProduct(normalizedToCheck, normalizedGoal);

                // If the dot products with both vectors are positive,
                // then the direction of vectorToCheck is between targetVector and goalVector.
                return (dotProductWithTarget >= 0.0 && dotProductWithGoal >= 0.0);
            }

            bool TotalForce::checkAngle(const Vector &targetVector, const std::shared_ptr<State> &sourceState,
                                             const std::shared_ptr<State> &neighborState) const
            {
                // Safety check in case of Segmentation Fault
                if (targetVector.empty())
                {
                    throw std::invalid_argument("Provided target vector is null");
                }

                // Safety check in case of Segmentation Fault
                if (!sourceState)
                {
                    throw std::invalid_argument("Provided sourceState is null");
                }

                // Safety check in case of Segmentation Fault
                if (!neighborState)
                {
                    throw std::invalid_argument("Provided neighborState is null");
                }

                Vector vectorTocheck = getVector(sourceState, neighborState);

                double dotValue = dotProduct(targetVector, vectorTocheck);

                // Check to avoid division by zero
                if (std::fabs(getNorm(targetVector)) < epsilon_)
                {
                    throw std::invalid_argument("Vector1 has zero magnitude.");
                }

                if (std::fabs(getNorm(vectorTocheck)) < epsilon_)
                {
                    return true;
                }

                double cosAngle = dotValue / (getNorm(targetVector) * getNorm(vectorTocheck));
                double safetyBuffer = std::cos(M_PI / 3.0);  // cos(60°)

                return cosAngle <= safetyBuffer;
        
            }

            double TotalForce::getRatioofValidInvalidPoints(std::vector<std::shared_ptr<State>> states){
                std::size_t l1 = 0;
                std::size_t l2 = 0;
                for (auto state : states ){// l1 number of Valid points
                    if (state->isAttractive_){
                    l1++;
                }
                else l2++;
                }

                if (l1 == 0)
                {
                    return INFINITY;
                }

                else {//ratio samller is better
                    std::cout << "VAlid points: " << l1 << std::endl;
                    std::cout << "INVAlid points: " << l2 << std::endl;
                    double ratio = static_cast<double>(l2) / l1;
                return ratio;}
            }



        }  // namespace fitstar

    }  // namespace geometric

}  // namespace ompl
