#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_FITSTAR_TOTALFORCE_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_FITSTAR_TOTALFORCE_

#include <memory>
#include <vector>
#include <mutex>

#include "ompl/base/Cost.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/BinaryHeap.h"

#include "ompl/geometric/planners/informedtrees/fitstar/Direction.h"
#include "ompl/geometric/planners/informedtrees/fitstar/Edge.h"
#include "ompl/geometric/planners/informedtrees/fitstar/Vertex.h"
namespace ompl
{
    namespace geometric
    {
        namespace fitstar
        {
            // Forward declare the ZIT* state class.
            class State;

            /** \brief The vertex class for both the forward and reverse search. */
            class TotalForce
            {
            public:
                /** \brief Constructs the vertex, which must be associated with a state. */
                TotalForce(const std::shared_ptr<State> &state, std::vector<std::shared_ptr<State>> &Allneigbors, size_t dimension_
                           );

                /** \brief Destructs this vertex. */
                ~TotalForce();
                /** \brief Set the state */
                void setState(const std::shared_ptr<State> &state);

                /** \brief Return the state */
                std::shared_ptr<State> getState() const;

                /** \brief Return the states */
                std::vector<std::shared_ptr<State>> getStates() const;

                /** \brief Return the std::vector<double> by two states */
                std::vector<double> getVector(const std::shared_ptr<State> state1,
                                              const std::shared_ptr<State> state2) const;

                double distance(const std::shared_ptr<State> state1, const std::shared_ptr<State> state2) const;

                std::vector<std::shared_ptr<State>> NearestKSamples(const std::shared_ptr<State> state,
                                                                    std::vector<std::shared_ptr<State>> Samples, int k);

                std::vector<std::shared_ptr<State>>
                NearestEllipseticKSamples(const std::shared_ptr<State> state, std::vector<double> &ForceDirection,
                                          std::vector<std::shared_ptr<State>> &Samples, int k, std::size_t dimension_);

                double calculateEllipticalDistance(const std::shared_ptr<State> &state1,
                                                   const std::shared_ptr<State> &state2,
                                                   std::vector<double> &totalForceVec, std::size_t dimension_);

                std::vector<double> force(const std::shared_ptr<State> &state1, const std::shared_ptr<State> &state2);

                void totalForce(const std::shared_ptr<State> &currentState,
                                const std::vector<std::shared_ptr<State>> &Samples);

                void totalForcewithStart(const std::shared_ptr<State> &currentState,
                                         const std::shared_ptr<State> &startstate,
                                         const std::shared_ptr<State> &goalstate, bool iterateForwardSearch);

                /** \brief Return the norm of two states */
                double getNorm(const std::vector<double> &v) const;

                /** \brief Return the dot product of two Vectors */
                double dotProduct(const std::vector<double> &v1, const std::vector<double> &v2) const;

                std::vector<double> vectorProjection(const std::vector<double> &a, const std::vector<double> &b);

                /** \brief normalize the vector */
                std::vector<double> normalize(const std::vector<double> &v) const;

                /** \brief Check whether the std::vector<double> within two guided direction. */
                bool isVectorBetween(const std::vector<double> &targetVector, const std::vector<double> &goalVector,
                                     const std::shared_ptr<State> &sourceState,
                                     const std::shared_ptr<State> &neighborState) const;

                /** \brief Check whether the std::vector<double> meet the required angle. */
                bool checkAngle(const std::vector<double> &targetVector, const std::shared_ptr<State> &sourceState,
                                const std::shared_ptr<State> &neighborState) const;

                /** \brief filter the neighbors vector using direction info*/
                void filterNeighbors(const std::shared_ptr<State> &lastState, const std::shared_ptr<State> &state,
                                     std::vector<std::shared_ptr<State>> &states,
                                     const std::shared_ptr<State> &goalState);

                /** \brief calculate direction Cost of current direction*/

                /** \brief Clear the neighbors vector of current target */
                void clearNeighbors();

                double getRatioofValidInvalidPoints(std::vector<std::shared_ptr<State>> states);

                std::size_t dimension_;

                mutable double totalMagnitude_;

                mutable std::vector<double> totalForceVec_;

                mutable double totalMagnitudewithStart_;

                void settotalForcewithStartValue(std::vector<double> &totalForceVecWithStart);

                mutable std::vector<double> totalForceVecwithStart_;

                /** \brief small parameter */
                const double epsilon_ = 1e-9;

                std::vector<double> getValueofforceDirection();

            private:
                /** \brief target state */
                std::shared_ptr<State> state_;

                /** \brief The neighbor states of target state */
                std::vector<std::shared_ptr<State>> states_;

                /** \brief The dimension of the state */

                /** \brief state lock */
                std::mutex state_mutex_;
            };

        }  // namespace fitstar

    }  // namespace geometric

}  // namespace ompl
inline std::vector<double> operator-(const std::vector<double> &lhs, const std::vector<double> &rhs)
{
    assert(lhs.size() == rhs.size() && "Vectors must be of the same size for subtraction!");

    std::vector<double> result(lhs.size());
    for (size_t i = 0; i < lhs.size(); ++i)
    {
        result[i] = lhs[i] - rhs[i];
    }
    return result;
}

// 向量加法运算符重载
inline std::vector<double> operator+(const std::vector<double> &lhs, const std::vector<double> &rhs)
{
    assert(lhs.size() == rhs.size() && "Vectors must be of the same size for addition!");

    std::vector<double> result(lhs.size());
    for (size_t i = 0; i < lhs.size(); ++i)
    {
        result[i] = lhs[i] + rhs[i];
    }
    return result;
}

// 向量与标量的乘法运算符重载
inline std::vector<double> operator*(const std::vector<double> &lhs, double scalar)
{
    std::vector<double> result(lhs.size());
    for (size_t i = 0; i < lhs.size(); ++i)
    {
        result[i] = lhs[i] * scalar;
    }
    return result;
}

// 向量与标量的除法运算符重载
inline std::vector<double> operator/(const std::vector<double> &lhs, double scalar)
{
    assert(scalar != 0 && "Cannot divide by zero!");

    std::vector<double> result(lhs.size());
    for (size_t i = 0; i < lhs.size(); ++i)
    {
        result[i] = lhs[i] / scalar;
    }
    return result;
}

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_FITSTAR_TOTALFORCE_
