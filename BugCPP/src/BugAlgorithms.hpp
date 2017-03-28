/**
 *@file BugAlgorithms.hpp
 *@brief Prototype for the Bug algorithms required in this assignment
 */

#ifndef BUG_ALGORITHMS_HPP_
#define BUG_ALGORITHMS_HPP_

#include "Simulator.hpp"

/**
 * @brief Bug algorithm computes a small move (m_dx, m_dy) that the robot needs to make
 */
struct Move
{
    double m_dx;
    double m_dy;    
};

/**
 *@brief Prototype for the different Bug algorithms required in this assignment
 *
 *@remark
 *  Feel free to add additional functions/data members as you see fit
 */
class BugAlgorithms
{
public:
    /**
     *@brief Set simulator
     *@param simulator pointer to simulator
     *
     *@remark
     *  Constructor performs only pointer assignment. Therefore,
     *  destructor should not delete the simulator
     */
    BugAlgorithms(Simulator * const simulator);
            
    /**
     *@brief Free memory and delete objects allocated by this instance
     *
     *@remark
     *  Constructor performs only pointer assignment. Therefore,
     *  destructor should not delete the simulator.
     */
    ~BugAlgorithms(void);
     
    
    /**
     *@brief Select the appropriate move so that the robot behaves
     *       as described in the respective bug algorithms.
     *@param sensor provides closest point from obstacle boundary to robot center
     */
    Move Bug0(Sensor sensor);
    Move Bug1(Sensor sensor);
    Move Bug2(Sensor sensor);
    
    /**
     * @brief evaluate whether obstacle is closer than goal.
     * @param sensor current sensor reading
     *@return true if dist from obstacle to goal is shorter than robot to goal
     */
    bool IsObstacleCloserToGoal(Sensor);

    /**
     * @brief check whether an obstacle is closer than the threshhold.
     * @param sensor current sensor reading
     * @return true if an obstacle is close.
     */
    bool IsObstacleNear(Sensor);
    
    /**
     * @brief Check whether an unobstructed line to goal exists from robot.
     * @param Current sensor reading.
     * @return true if we have an unobstructed line toward goal.
     */
    bool IsObstacleOnPath(Sensor);
    
    /**
     *@brief Move around obstacle by moving slightly away and using tangent.
     *@param sensor current sensor reading
     *@return next move
     */
    Move MoveAroundBug0(Sensor sensor);
    
    /**
     *@brief Move around an obstacle using tangents to closest point.
     *@param Current sensor reading.
     *@return next move
     */
    Move MoveAround(Sensor sensor, int direction);
    
    /**
     *@brief Move directly from current position toward goal along the
     *       slope of the most direct line.
     *@return next move
     */
    Move MoveStraight();
    
protected:
    /**
     *@brief Pointer to simulator
     */
    Simulator  *m_simulator;

    enum Mode
	{
	    STRAIGHT,
	    STRAIGHT_AND_AWAY_FROM_LEAVE_POINT,
	    AROUND_AND_AWAY_FROM_HIT_POINT,
	    AROUND_AND_TOWARD_LEAVE_POINT,
	    AROUND
	};
    
    // indicators of turn direction around obstacle
    enum Direction
    {
        TURN_LEFT,
        TURN_RIGHT
    };

    double m_hit[2], m_leave[2], m_distLeaveToGoal, m_distanceHitToGoal;
    int    m_mode;
    
    // false until we are cleare of the initial hit point
    bool isHitPointCleared;
    
    //////////////////////////////////////////////////////
    // step counters are used by Bug1 to choose shortest
    // route back to best leave point.
    
    // step tracker
    int stepCount;
    
    // remember step count to best leave point
    int stepCountUntilBestLeavePoint;
    
    // remember step count after best leave point
    int stepCountAfterBestLeavePoint;
    
    friend class Graphics;
};

#endif
