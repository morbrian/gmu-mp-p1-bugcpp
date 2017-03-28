#include "BugAlgorithms.hpp"

#include <iostream>
using std::cout;

BugAlgorithms::BugAlgorithms(Simulator * const simulator)
{
    m_simulator = simulator;   
    m_mode = STRAIGHT;
    m_hit[0] = m_hit[1] = HUGE_VAL;
    m_leave[0] = m_leave[1] = 0;
    m_distLeaveToGoal = HUGE_VAL;
    m_distanceHitToGoal = HUGE_VAL;
    isHitPointCleared = false;
    stepCount = 0;
    stepCountUntilBestLeavePoint = 0;
    stepCountAfterBestLeavePoint = 0;
}

BugAlgorithms::~BugAlgorithms(void)
{
    //do not delete m_simulator  
}

/**
 * Brian Moriarty
 * HW1 - Feb 24, 2012
 * Bug0 Algorithm
 */
Move BugAlgorithms::Bug0(Sensor sensor)
{
    Move move = {0, 0};

    // if goal reached stop moving 
    if (m_simulator->HasRobotReachedGoal())
        return move;
    
    // if obstacle is close
    if (IsObstacleNear(sensor))
    {
        // then move around the obstacle
        m_mode = AROUND;
        move = MoveAroundBug0(sensor);
    } 
    else 
    {
        // otherwise move straight toward goal.
        m_mode = STRAIGHT;
        move = MoveStraight();
    }
    return move;
}

/**
 * Brian Moriarty
 * HW1 - Feb 24, 2012
 * Bug1 Algorithm
 */
Move BugAlgorithms::Bug1(Sensor sensor)
{
    Move move ={0,0};
    
    // if goal reached stop moving 
    if (m_simulator->HasRobotReachedGoal())
        return move;
    
    // pick operation according to current state
    switch (m_mode)
    {
        case STRAIGHT:
        case STRAIGHT_AND_AWAY_FROM_LEAVE_POINT:
            if (!IsObstacleNear(sensor)) 
            {
                //if nothing near by, go straight toward goal.
                m_mode = STRAIGHT;
                move = MoveStraight();
            } 
            else
            {   
                // othewise record hit-point coords, and change state
                m_mode = AROUND_AND_AWAY_FROM_HIT_POINT;
                
                // reset counter and flags for new hit point
                isHitPointCleared = false;
                stepCount = 1;
                
                // remember hit point
                m_hit[0] = m_simulator->GetRobotCenterX();
                m_hit[1] = m_simulator->GetRobotCenterY();
                
                // make leave point equal hit point until finding closer option
                m_leave[0] = m_hit[0];
                m_leave[1] = m_hit[1];
                m_distLeaveToGoal = m_simulator->GetDistanceFromRobotToGoal();
                
                // make first move around obstacle
                move = MoveAround(sensor, TURN_LEFT);
            }
            break;
        case AROUND_AND_AWAY_FROM_HIT_POINT:
            ++stepCount;
            // if we walked the entire obstacle boundary...
            if (isHitPointCleared &&
                m_simulator->ArePointsNear(m_hit[0], m_hit[1],
                                           m_simulator->GetRobotCenterX(),
                                           m_simulator->GetRobotCenterY()))
            {
                // then change mode and decide quickest way to best leave point
                m_mode = AROUND_AND_TOWARD_LEAVE_POINT;
                // calculate how many steps since the over all best leave point
                stepCountAfterBestLeavePoint 
                    = stepCount - stepCountUntilBestLeavePoint;
            }
            else
            {
                // otherwise continue searching for a better leave point.
                // check if can leave safely; if yes compare distance
                if (!IsObstacleCloserToGoal(sensor)
                    && m_distLeaveToGoal 
                        > m_simulator->GetDistanceFromRobotToGoal())
                {
                    // remember this closer leave point and distance to goal
                    m_leave[0] = m_simulator->GetRobotCenterX();
                    m_leave[1] = m_simulator->GetRobotCenterY();
                    m_distLeaveToGoal 
                        = m_simulator->GetDistanceFromRobotToGoal();
                    
                    // remember steps to this point
                    stepCountUntilBestLeavePoint = stepCount;
                }
                
                // when we have moved far enough from initial hit point
                // on this obstacle, toggle the indicator.
                if (!isHitPointCleared
                    && !m_simulator->ArePointsNear(m_hit[0], m_hit[1],
                                                m_simulator->GetRobotCenterX(),
                                                m_simulator->GetRobotCenterY()))
                    isHitPointCleared = true;
            }
            move = MoveAround(sensor, TURN_LEFT);
            break;
        case AROUND_AND_TOWARD_LEAVE_POINT:
            // if we arrive at leave point, move toward goal
            // otherwise keep moving around obstacle turning toward
            // the shortest route toward leave point.
            if (m_simulator->ArePointsNear(m_leave[0], m_leave[1],
                                           m_simulator->GetRobotCenterX(),
                                           m_simulator->GetRobotCenterY()))
            {
                m_mode = STRAIGHT_AND_AWAY_FROM_LEAVE_POINT;
                move = MoveStraight();
            }
            else if (stepCountAfterBestLeavePoint 
                        < stepCountUntilBestLeavePoint)
                move = MoveAround(sensor, TURN_RIGHT);
            else
                move = MoveAround(sensor, TURN_LEFT);
            break;
    }
    return move;
}

/**
 * Brian Moriarty
 * HW1 - Feb 24, 2012
 * Bug2 Algorithm
 */
Move BugAlgorithms::Bug2(Sensor sensor)
{
    Move move ={0,0};
    
    // if goal reached stop moving 
    if (m_simulator->HasRobotReachedGoal())
        return move;
    
    // pick operation according to current state
    switch (m_mode)
    {
        case STRAIGHT:
            // if nothing is near by, or at least is not in our way
            if (!IsObstacleNear(sensor) || !IsObstacleCloserToGoal(sensor)) 
            {
                // then move straight toward goal.
                m_mode = STRAIGHT;
                move = MoveStraight();
            } 
            else
            {   
                // remember hit point and change mode
                isHitPointCleared = false;
                m_hit[0] = m_simulator->GetRobotCenterX();
                m_hit[1] = m_simulator->GetRobotCenterY();
                m_distanceHitToGoal = m_simulator->GetDistanceFromRobotToGoal();
                m_mode = AROUND_AND_AWAY_FROM_HIT_POINT;
                
                // make first move around obstacle
                move = MoveAround(sensor, TURN_LEFT);
            }
            
            break;
        case AROUND_AND_AWAY_FROM_HIT_POINT:
            if (isHitPointCleared &&
                m_simulator->ArePointsNear(m_hit[0], m_hit[1],
                                           m_simulator->GetRobotCenterX(),
                                           m_simulator->GetRobotCenterY())) {
                // if we walked the entire obstacle boundary, we must be stuck.
                // this state should not be possible unless we are surrounded
                // by a single obstacle with no exit.
                printf("Failed to find path after traversing object.\n");
            }
            else if (!isHitPointCleared
                    && !m_simulator->
                       ArePointsNear(m_hit[0], m_hit[1],
                                     m_simulator->GetRobotCenterX(),
                                     m_simulator->GetRobotCenterY()))
            {
                // when we have moved far enough from initial hit point
                // on this obstacle, toggle the indicator.
                isHitPointCleared = true;
            }
            
            // if we moved away from the hitPoint, and we are on a line
            // toward the goal and are closer than the initial hit point
            // and path is not obstructed by obstacle, then move straight.
            if (isHitPointCleared
                     && m_simulator->
                        IsPointNearLine(m_simulator->GetRobotCenterX(), 
                                        m_simulator->GetRobotCenterY(),
                                        m_simulator->GetRobotInitX(),
                                        m_simulator->GetRobotInitY(),
                                        m_simulator->GetGoalCenterX(), 
                                        m_simulator->GetGoalCenterY())
                     && m_simulator->GetDistanceFromRobotToGoal()
                        < m_distanceHitToGoal
                     && !IsObstacleCloserToGoal(sensor))
            {
                m_mode = STRAIGHT;
                move = MoveStraight();
            } else
                // otherwise, continue moving left around obstacle
                move = MoveAround(sensor, TURN_LEFT);
            break;
    }
    return move;

}

// true if obstacle is closer to the goal than the robot
bool BugAlgorithms::IsObstacleCloserToGoal(Sensor sensor)
{
    // calculate distance from obstacle point to goal
    double xDelta = m_simulator->GetGoalCenterX() - sensor.m_xmin;
    double yDelta = m_simulator->GetGoalCenterY() - sensor.m_ymin;
    double dObsToGoal = sqrt( (xDelta * xDelta) + (yDelta * yDelta) );
    
    return (dObsToGoal <= m_simulator->GetDistanceFromRobotToGoal());
}

// true if the obstacle distance is less than the "when to turn" threshhold.
bool BugAlgorithms::IsObstacleNear(Sensor sensor)
{
    return (sensor.m_dmin <= m_simulator->GetWhenToTurn());
}

// true if the sensed obstacle is on a line with the goal and robot.
bool BugAlgorithms::IsObstacleOnPath(Sensor sensor)
{
    return (m_simulator->IsPointNearLine(sensor.m_xmin, sensor.m_ymin, 
                                         m_simulator->GetRobotCenterX(), 
                                         m_simulator->GetRobotCenterY(), 
                                         m_simulator->GetGoalCenterX(), 
                                         m_simulator->GetGoalCenterY()));
}

// move around obstacle by moving away from closes point and perpendicular
// the "move away" step is unique to Bug0 because the other 2 Bugs have
// other logic describing how to leave the obstacle and can just move
// perpendicular to circumnavigate the obstacle.
// If Bug0 always moved only perpindicular  (like the other MoveAround does)
// then it might never get the clue to change states to move toward obstacle.
Move BugAlgorithms::MoveAroundBug0(Sensor sensor)
{
    double xDistanceToObject = sensor.m_xmin 
    - m_simulator->GetRobotCenterX();
    
    double yDistanceToObject = sensor.m_ymin
    - m_simulator->GetRobotCenterY();
    
    double stepRatio = m_simulator->GetStep() /  sensor.m_dmin;
    
    Move move;
    
    // move step distance along line perpendicular to line encountering object.
    // identify perpendicular distance by swapping x,y values with -y, x
    // this is an "always turn left" strategy.
    // step away
    move.m_dx = - stepRatio * xDistanceToObject;
    move.m_dy = - stepRatio * yDistanceToObject;
    
    // make it perpendicular
    move.m_dx += - yDistanceToObject / 2.0;
    move.m_dy += xDistanceToObject / 2.0;
    
    return move; 
}

// move around by following perpindicular line to nearest obstacle
// turning in the specified direction (right or left).
Move BugAlgorithms::MoveAround(Sensor sensor, int direction) 
{
    double xDistanceToObject = sensor.m_xmin 
                            - m_simulator->GetRobotCenterX();
    
    double yDistanceToObject = sensor.m_ymin
                            - m_simulator->GetRobotCenterY();
    
    double stepRatio = m_simulator->GetStep() /  sensor.m_dmin;
    
    Move move;
    
    // move step distance along line perpendicular to line encountering object.
    // identify perpendicular distance by swapping x,y values with -y, x
    if (TURN_LEFT == direction)
    {
        move.m_dx = - stepRatio * yDistanceToObject;
        move.m_dy = stepRatio * xDistanceToObject;
    }
    else
    {
        move.m_dx = stepRatio * yDistanceToObject;
        move.m_dy = - stepRatio * xDistanceToObject;
    }
    
    return move;
}

// move straight from current position to goal.
Move BugAlgorithms::MoveStraight()
{
    double xDistanceToGoal = m_simulator->GetGoalCenterX() 
                             - m_simulator->GetRobotCenterX();
    
    double yDistanceToGoal = m_simulator->GetGoalCenterY() 
                             - m_simulator->GetRobotCenterY();
    
    double distanceToGoal = m_simulator->GetDistanceFromRobotToGoal();
    
    double stepRatio = m_simulator->GetStep() / distanceToGoal;
    
    Move move;
    
    move.m_dx = stepRatio * xDistanceToGoal;
    move.m_dy = stepRatio * yDistanceToGoal;
    
    return move;
}


