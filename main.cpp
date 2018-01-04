//
//  main.cpp
//  无人机路径规划算法
//
//  Created by 孙浩 on 2018/1/4.
//  Copyright © 2018年 BUAA F632. All rights reserved.
//
#include <iostream>
#include "Situation.hpp"
#include "B.hpp"

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

#ifndef M_PI
const float M_PI = 3.14159265358979323846f;
#endif

///* Store the goals of the Aircrafts. */
//std::vector<B> goals;
//
//void setupA(Situation *situa)
//{
//    /* Seed the random number generator. */
//    std::srand(static_cast<unsigned int>(std::time(0)));
//
//    /* Specify the global time step of the situaulation. */
//    situa->setTimeGap(3.0f);
//
//    /* Specify the default parameters for Aircrafts that are subsequently added. */
//    situa->setAircraftDefaults(15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f);
//
//    /*
//     * Add Aircrafts, specifying their start position, and store their goals on the
//     * opposite side of the environment.
//     */
//    for (size_t i = 0; i < 25 ; ++i) {
//        situa->addAircraft(B(250.0f,  40.0f + i * 40.0f));
//        goals.push_back(B(-110.0f, 40.0f + i * 40.0f));
//
//        situa->addAircraft(B(-250.0f ,  40.0f + i * 40.0f));
//        goals.push_back(B(110.0f, 40.0f + i * 40.0f));
//
//        situa->addAircraft(B(40.0f + i * 40.0f, 250.0f));
//        goals.push_back(B(40.0f + i * 40.0f, -110.0f));
//
//        situa->addAircraft(B(40.0f + i * 40.0f, -250.0f));
//        goals.push_back(B(40.0f + i * 40.0f, 110.0f));
//    }
//
//
//}
//
//void renewVisualization(Situation *situa)
//{
//    /* Output the current global time. */
//    std::cout << situa->AllTime();
//
//    /* Output the current position of all the Aircrafts. */
//    for (size_t i = 0; i < situa->SizeAircrafts(); ++i) {
//        std::cout << " " << situa->AircraftStation(i);
//    }
//
//    std::cout << std::endl;
//}
//
//void setPreferredVelocities(Situation *situa)
//{
//    /*
//     * Set the preferred velocity to be a vector of unit magnitude (speed) in the
//     * direction of the goal.
//     */
//#ifdef _OPENMP
//#pragma omp parallel for
//#endif
//    for (int i = 0; i < static_cast<int>(situa->SizeAircrafts()); ++i) {
//        B goalVector = goals[i] - situa->AircraftStation(i);
//
//        if (findnorm(goalVector) > 1.0f) {
//            goalVector = norm(goalVector);
//        }
//
//        situa->setAircraftPrefSpeed(i, goalVector);
//
//        /*
//         * Perturb a little to avoid deadlocks due to perfect symmetry.
//         */
//        float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
//        float dist = std::rand() * 0.0001f / RAND_MAX;
//
//        situa->setAircraftPrefSpeed(i, situa->AircraftPrefSpeed(i) +
//                                    dist * B(std::cos(angle), std::sin(angle)));
//    }
//}
//
//bool reachedGoal(Situation *situa)
//{
//    int count =0;
//    /* Check if all Aircrafts have reached their goals. */
//    for (size_t i = 0; i < situa->SizeAircrafts(); ++i) {
//        if (findnorm(situa->AircraftStation(i) - goals[i]) > 20.0f * 20.0f) {
//            std::cout<<i<<" ";
//            count ++;
//        }
//    }
//    if (count) {
//        std::cout<<count<<" ";
//        return false;
//    } else {
//        return true;
//    }
//}
//
//#ifndef M_PI
//const float M_PI = 3.14159265358979323846f;
//#endif
//
//
//void setupB(Situation *situa)
//{
//    std::srand(static_cast<unsigned int>(time(0)));
//
//    situa->setTimeGap(3.0f);
//
//    situa->setAircraftDefaults(15.0f, 10, 10.0f, 10.0f, 1.5f, 2.0f);
//
//    for (size_t i = 0; i < 250; ++i) {
//        situa->addAircraft(200.0f *B(cos(i * 2.0f * M_PI / 250.0f),sin(i * 2.0f * M_PI / 250.0f)));
//        goals.push_back(-situa->AircraftStation(i));
//    }
//}
//
////void setPreferredVelocities(RVO::RVOSimulator *sim)
////{
////    /*
////     * Set the preferred velocity to be a vector of unit magnitude (speed) in the
////     * direction of the goal.
////     */
////#ifdef _OPENMP
////#pragma omp parallel for
////#endif
////    for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) {
////        RVO::Vector2 goalVector = goals[i] - sim->getAgentPosition(i);
////
////        if (RVO::absSq(goalVector) > 1.0f) {
////            goalVector = RVO::normalize(goalVector);
////        }
////
////        sim->setAgentPrefVelocity(i, goalVector);
////    }
////}
////
////bool reachedGoal(RVO::RVOSimulator *sim)
////{
////    /* Check if all agents have reached their goals. */
////    for (size_t i = 0; i < sim->getNumAgents(); ++i) {
////        if (RVO::absSq(sim->getAgentPosition(i) - goals[i]) > sim->getAgentRadius(i) * sim->getAgentRadius(i)) {
////            return false;
////        }
////    }
////
////    return true;
////}
////
//
//int main(int argc, const char *argv[])
//{
//    /* Create a new situaulator instance. */
//    Situation *situa = new Situation();
//
//    /* Set up the . */
//    setupA(situa);
//    int Titime=0;
//    /* Perform (and manipulate) the situaulation. */
//    do {
//        renewVisualization(situa);
//        setPreferredVelocities(situa);
//        situa->doStep();
//        Titime++;
//        reachedGoal(situa);
//    }
//    //    while(!reachedGoal(situa));
//    while (Titime <120);
//
//    delete situa;
//
////    /* Create a new situaulator instance. */
////    Situation *situaB = new Situation();
////
////    /* Set up the . */
////    setupB(situaB);
////    int Titime=0;
////    /* Perform (and manipulate) the situaulation. */
////    do {
////        renewVisualization(situa);
////        setPreferredVelocities(situa);
////        situa->doStep();
////        Titime++;
////        reachedGoal(situa);
////    }
////    //    while(!reachedGoal(situa));
////    while (Titime <120);
////
////    delete situa;
//
//    return 0;
//}

#include <iostream>

#include <iostream>
#include "Situation.hpp"
#include "B.hpp"

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

#ifndef M_PI
const float M_PI = 3.14159265358979323846f;
#endif

/* Store the goals of the Aircrafts. */
vector<B> goals;

void setupA(Situation *situa)
{
    /* Seed the random number generator. */
    srand(static_cast<unsigned int>(time(0)));

    /* Specify the global time step of the situaulation. */
    situa->setTimeGap(3.0f);

    /* Specify the default parameters for Aircrafts that are subsequently added. */
    situa->setAircraftDefaults(15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f);

    /*
     * Add Aircrafts, specifying their start position, and store their goals on the
     * opposite side of the environment.
     */
    for (size_t i = 0; i < 3 ; ++i) {
        situa->addAircraft(B(0.0f,  40.0f + i * 40.0f));
        goals.push_back(B(200.0f, 40.0f + i * 40.0f));

        situa->addAircraft(B(200.0f ,  40.0f + i * 40.0f));
        goals.push_back(B(0.0f, 40.0f + i * 40.0f));

        situa->addAircraft(B(40.0f + i * 40.0f, 200.0f));
        goals.push_back(B(40.0f + i * 40.0f, 0.0f));

        situa->addAircraft(B(40.0f + i * 40.0f, 0.0f));
        goals.push_back(B(40.0f + i * 40.0f, 200.0f));
    }


}

void renewVisualization(Situation *situa)
{
    /* Output the current global time. */
    cout << situa->AllTime();

    /* Output the current position of all the Aircrafts. */
    for (size_t i = 0; i < situa->SizeAircrafts(); ++i) {
        cout << " " << situa->AircraftStation(i);
    }

    cout << endl;
}

void setPreferredVelocities(Situation *situa)
{
    /*
     * Set the preferred velocity to be a vector of unit magnitude (speed) in the
     * direction of the goal.
     */
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < static_cast<int>(situa->SizeAircrafts()); ++i) {
        B goalVector = goals[i] - situa->AircraftStation(i);

        if (findnorm(goalVector) > 1.0f) {
            goalVector = norm(goalVector);
        }

        situa->setAircraftPrefSpeed(i, goalVector);

        /*
         * Perturb a little to avoid deadlocks due to perfect symmetry.
         */
        float angle = rand() * 2.0f * M_PI / RAND_MAX;
        float dist = rand() * 0.0001f / RAND_MAX;

        situa->setAircraftPrefSpeed(i, situa->AircraftPrefSpeed(i) +
                                    dist * B(cos(angle), sin(angle)));
    }
}

bool reachedGoal(Situation *situa)
{
    int count =0;
    /* Check if all Aircrafts have reached their goals. */
    for (size_t i = 0; i < situa->SizeAircrafts(); ++i) {
        if (findnorm(situa->AircraftStation(i) - goals[i]) > 2.0f * 2.0f) {
            cout<<i<<" ";
            count ++;
        }
    }
    if (count) {
        cout<<count<<" ";
        return false;
    } else {
        return true;
    }
}

void setupB(Situation *situa)
{
    
    srand(static_cast<unsigned int>(time(0)));
    situa->setTimeGap(3.0f);

    situa->setAircraftDefaults(15.0f, 10, 10.0f, 10.0f, 1.5f, 2.0f);

    for (size_t i = 0; i < 8; ++i) {
        situa->addAircraft(200.0f *B(cos(i * 2.0f * M_PI / 8.0f),sin(i * 2.0f * M_PI / 8.0f)));
        goals.push_back(-situa->AircraftStation(i));
    }
    vector<B> barrier;
    barrier.push_back(B(90.0f, 110.0f));
    barrier.push_back(B(90.0f, 90.0f));
    barrier.push_back(B(110.0f, 90.0f));
    barrier.push_back(B(110.0f, 110.0f));
    situa->addBarrier(barrier);
    
//    Process the obstacles so that they are accounted for in the simulation.
    situa->processBarriers();
}

void setPreferredVelocitiesB(Situation *situaB)
{
    /*
     * Set the preferred velocity to be a vector of unit magnitude (speed) in the
     * direction of the goal.
     */
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < static_cast<int>(situaB->SizeAircrafts()); ++i) {
        B goalVector = goals[i] - situaB->AircraftStation(i);

        if (findnorm(goalVector) > 1.0f) {
            goalVector =2*norm(goalVector);
        }
        else goalVector *= 2;
        situaB->setAircraftPrefSpeed(i, goalVector);
    }
}

bool reachedGoalB(Situation *situaB)
{
    int count = 0;
    /* Check if all agents have reached their goals. */
    for (size_t i = 0; i < situaB->SizeAircrafts(); ++i) {
        if (findnorm(situaB->AircraftStation(i) - goals[i]) > 2.0f * 2.0f) {
            cout<<i<<" ";
            count ++;
        }
    }

    return true;
}


void setupC(Situation *situa)
{
    
    srand(static_cast<unsigned int>(time(0)));
    situa->setTimeGap(3.0f);
    
    situa->setAircraftDefaults(15.0f, 10, 10.0f, 10.0f, 1.5f, 2.0f);
    
    for (size_t i = 0; i < 5 ; ++i) {
        situa->addAircraft(B(100.0f,  0.0f + i * 5.0f));
        goals.push_back(B(100.0f, 200.0f));
        
        situa->addAircraft(B(100.0f , 200.0f - i * 5.0f));
        goals.push_back(B(100.0f, 0.0f));
        
        situa->addAircraft(B(0.0f + i * 5.0f, 100.0f));
        goals.push_back(B(200.0f, 100.0f));
        
        situa->addAircraft(B(200.0f - i * 5.0f, 100.0f));
        goals.push_back(B(0.0f, 100.0f));
    }
    vector<B> barrier;
    barrier.push_back(B(90.0f, 110.0f));
    barrier.push_back(B(90.0f, 90.0f));
    barrier.push_back(B(110.0f, 90.0f));
    barrier.push_back(B(110.0f, 110.0f));
    situa->addBarrier(barrier);
    
    //    Process the obstacles so that they are accounted for in the simulation.
    situa->processBarriers();
}

void setPreferredVelocitiesC(Situation *situaC)
{
    /*
     * Set the preferred velocity to be a vector of unit magnitude (speed) in the
     * direction of the goal.
     */
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < static_cast<int>(situaC->SizeAircrafts()); ++i) {
        B goalVector = goals[i] - situaC->AircraftStation(i);
        
        if (findnorm(goalVector) > 1.0f) {
            goalVector =2*norm(goalVector);
        }
        else goalVector *= 2;
        
        situaC->setAircraftPrefSpeed(i, goalVector);
    }
}

bool reachedGoalC(Situation *situaC)
{
    int count = 0;
    /* Check if all agents have reached their goals. */
    for (size_t i = 0; i < situaC->SizeAircrafts(); ++i) {
        if (findnorm(situaC->AircraftStation(i) - goals[i]) > 2.0f * 2.0f) {
            cout<<i<<" ";
            count ++;
//            return false;
        }
    }
    
    return true;
}
int main(int argc, const char *argv[])
{
    /* Create a new situaulator instance. */
    Situation *situa = new Situation();

    /* Set up the . */
    setupA(situa);
    int Titime=0;
    /* Perform (and manipulate) the situaulation. */
    do {
        renewVisualization(situa);
        setPreferredVelocities(situa);
        situa->doStep();
        Titime++;
        reachedGoal(situa);
    }
//        while(!reachedGoal(situa));
    while (Titime <120);

    delete situa;

    /* Create a new situaulator instance. */
    Situation *situaB = new Situation();

    /* Set up the . */
    setupB(situaB);
    Titime=0;
    /* Perform (and manipulate) the situaulation. */
    do {
        renewVisualization(situaB);
        setPreferredVelocitiesB(situaB);
        situaB->doStep();
        Titime++;
        reachedGoalB(situaB);
    }
//    while(!reachedGoal(situa));
    while (Titime <150);
    delete situaB;
    
    /* Create a new situaulator instance. */
    Situation *situaC = new Situation();
    
    /* Set up the . */
    setupC(situaC);
    Titime=0;
    /* Perform (and manipulate) the situaulation. */
    do {
        renewVisualization(situaC);
        setPreferredVelocitiesC(situaC);
        situaC->doStep();
        Titime++;
        reachedGoalC(situaC);
    }
//    while(reachedGoal(situa));
    while (Titime <1000);
    delete situaC;

    return 0;
}


