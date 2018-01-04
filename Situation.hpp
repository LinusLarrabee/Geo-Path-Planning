//
//  Situation.hpp
//  无人机路径规划算法
//
//  Created by JerryStive on 2018/1/4.
//  Copyright © 2018年 BUAA F632. All rights reserved.
//

#ifndef Situation_hpp
#define Situation_hpp
#include <cstddef>
#include <limits>
#include <vector>
#include <iostream>
#include "B.hpp"
using namespace std;

const size_t Situa_ERROR = numeric_limits<size_t>::max();
const float Situa_EPSILON = 0.00001f;

class Pointline {
public:
    B position;
    B orientation;
};

class Aircraft;
class Group;
class Barrier;
class Situation {
public:
    Situation();
    Situation(float timeGap, float OtherDistance, size_t maxOthers, float timeCrash, float timeCrashBarrier, float radius, float maxSpeed, const B &Speed = B());
    ~Situation();
    
    size_t addAircraft(const B &station);
    size_t addAircraft(const B &station, float OtherDistance,
                       size_t maxOthers, float timeCrash,
                       float timeCrashBarrier, float radius, float maxSpeed,
                       const B &Speed = B());
    size_t addBarrier(const vector<B> &vertices);
    void doStep();
    size_t AircraftAircraftOther(size_t airNo, size_t OtherNo) const;
    size_t AircraftMaxOthers(size_t airNo) const;
    float AircraftMaxSpeed(size_t airNo) const;
    float AircraftOtherDistance(size_t airNo) const;
    size_t AircraftSizeAircraftOthers(size_t airNo) const;
    size_t AircraftNumBarrierOthers(size_t airNo) const;
    size_t AircraftNumCrackPointlines(size_t airNo) const;
    size_t AircraftBarrierOther(size_t airNo, size_t OtherNo) const;
    
    const Pointline &AircraftCrackPointline(size_t airNo, size_t lineNo) const;
    const B &AircraftStation(size_t airNo) const;
    const B &AircraftPrefSpeed(size_t airNo) const;
    float AircraftRadius(size_t airNo) const;
    float AircraftTimeCrash(size_t airNo) const;
    float AircraftTimeCrashBarrier(size_t airNo) const;
    const B &AircraftSpeed(size_t airNo) const;
    float AllTime() const;
    size_t SizeAircrafts() const;
    size_t NumBarrierVertices() const;
    const B &BarrierVertex(size_t vertexNo) const;
    size_t NextBarrierVertexNo(size_t vertexNo) const;
    size_t PrevBarrierVertexNo(size_t vertexNo) const;
    float TimeGap() const;
    void processBarriers();
    bool findVisibility(const B &position1, const B &position2,
                         float radius = 0.0f) const;
    void setAircraftDefaults(float OtherDistance, size_t maxOthers,
                             float timeCrash, float timeCrashBarrier,
                             float radius, float maxSpeed,
                             const B &Speed = B());
    void setAircraftMaxOthers(size_t airNo, size_t maxOthers);
    void setAircraftMaxSpeed(size_t airNo, float maxSpeed);
    void setAircraftOtherDistance(size_t airNo, float OtherDistance);
    void setAircraftStation(size_t airNo, const B &station);
    void setAircraftPrefSpeed(size_t airNo, const B &prefSpeed);
    void setAircraftRadius(size_t airNo, float radius);
    void setAircraftTimeCrash(size_t airNo, float timeCrash);
    void setAircraftTimeCrashBarrier(size_t airNo, float timeCrashBarrier);
    void setAircraftSpeed(size_t airNo, const B &Speed);
    void setTimeGap(float timeGap);
private:
    vector<Aircraft *> airs_;
    Aircraft *defaultAircraft_;
    float globalTime_;
    Group *Group_;
    vector<Barrier *> barriers_;
    float timeGap_;
    
    friend class Aircraft;
    friend class Group;
    friend class Barrier;
};

class Group {
private:
    class AircraftaNode {
    public:
        size_t begin;
        size_t end;
        size_t left;
        float maxX;
        float maxY;
        float minX;
        float minY;
        size_t right;
    };
    class BarrieraNode {
    public:
        BarrieraNode *left;
        size_t barrierNo;
        BarrieraNode *right;
    };
    explicit Group(Situation *situa);
    ~Group();
    void buildAircrafta();
    void buildAircraftaRecursive(size_t begin, size_t end, size_t node);
    void buildBarriera();
    BarrieraNode *buildBarrieraRecursive(const vector<size_t> &
                                                 barrierNos);
    void computeAircraftOthers(Aircraft *air, float &rangeDouble) const;
    void computeBarrierOthers(Aircraft *const air, float rangeDouble) const;
    void deleteBarriera(BarrieraNode *node);
    void findAircraftaRecursive(Aircraft *air, float &rangeDouble,
                                    size_t node) const;
    void findBarrieraRecursive(Aircraft *air, float rangeDouble,
                                    const BarrieraNode *node) const;
    bool findVisibility(const B &q1, const B &q2, float radius) const;
    bool findVisibilityRecursive(const B &q1, const B &q2,
                                  float radius,
                                  const BarrieraNode *node) const;
    vector<size_t> airNos_;
    vector<AircraftaNode> aira_;
    BarrieraNode *barriera_;
    Situation *situa_;
    static const size_t MAX_LEAF_SIZE = 10;
    
    friend class Aircraft;
    friend class Situation;
};

class Barrier {
private:
    Barrier() : isConvex_(false), nextBarrier_(0), prevBarrier_(0) { }
    bool isConvex_;
    size_t nextBarrier_;
    B position_;
    size_t prevBarrier_;
    friend class Aircraft;
    friend class Group;
    friend class Situation;
};

class Aircraft {
private:
    explicit Aircraft(Situation *situa);
    Aircraft(Situation *situa, const B &station);
    Aircraft(Situation *situa, const B &station, float OtherDistance,
             size_t maxOthers, float timeCrash, float timeCrashBarrier,
             float radius, const B &Speed, float maxSpeed);
    void computeOthers();
    int computeNewSpeed();
    void insertAircraftOther(size_t airNo, float &rangeDouble);
    void insertBarrierOther(size_t barrierNo, float rangeDouble);
    void renew();
    
    vector<pair<float, size_t> > airOthers_;
    size_t maxOthers_;
    float maxSpeed_;
    float OtherDistance_;
    B newSpeed_;
    vector<pair<float, size_t> > barrierOthers_;
    vector<Pointline> crackPointlines_;
    B station_;
    B prefSpeed_;
    float radius_;
    Situation *situa_;
    float timeCrash_;
    float timeCrashBarrier_;
    B Speed_;
    
    friend class Group;
    friend class Situation;
};
bool Path1(const vector<Pointline> &lines, size_t lineNo,
                    float radius, const B &optSpeed,
                    bool orientationOpt, B &result);
bool Path2(const vector<Pointline> &lines, size_t num, float radius,
                    const B &optSpeed, bool orientationOpt,
                    B &result);
void Path3(const vector<Pointline> &lines, size_t numBarrierPointlines,
                    float radius, B &result);

#include "Situation.hpp"

Aircraft::Aircraft(Situation *situa) : maxOthers_(0), maxSpeed_(0.0f), OtherDistance_(0.0f), radius_(0.0f), situa_(situa), timeCrash_(0.0f), timeCrashBarrier_(0.0f) { }

Aircraft::Aircraft(Situation *situa, const B &station) : maxOthers_(situa->defaultAircraft_->maxOthers_), maxSpeed_(situa->defaultAircraft_->maxSpeed_), OtherDistance_(situa->defaultAircraft_->OtherDistance_), newSpeed_(situa->defaultAircraft_->Speed_), station_(station), radius_(situa->defaultAircraft_->radius_), situa_(situa), timeCrash_(situa->defaultAircraft_->timeCrash_), timeCrashBarrier_(situa->defaultAircraft_->timeCrashBarrier_), Speed_(situa->defaultAircraft_->Speed_) { }

Aircraft::Aircraft(Situation *situa, const B &station, float OtherDistance, size_t maxOthers, float timeCrash, float timeCrashBarrier, float radius, const B &Speed, float maxSpeed) : maxOthers_(maxOthers), maxSpeed_(maxSpeed), OtherDistance_(OtherDistance), newSpeed_(Speed), station_(station), radius_(radius), situa_(situa), timeCrash_(timeCrash), timeCrashBarrier_(timeCrashBarrier), Speed_(Speed) { }

void Aircraft::computeOthers()
{
    barrierOthers_.clear();
    float rangeDouble = sqr(timeCrashBarrier_ * maxSpeed_ + radius_);
    situa_->Group_->computeBarrierOthers(this, rangeDouble);
    
    airOthers_.clear();
    
    if (maxOthers_ > 0) {
        rangeDouble = sqr(OtherDistance_);
        situa_->Group_->computeAircraftOthers(this, rangeDouble);
    }
}

int Aircraft::computeNewSpeed()
{
    bool isLeftForeign = false;
    bool isRightForeign = false;
    crackPointlines_.clear();
    int crack = 0;
    for (size_t i = 0; i < barrierOthers_.size(); ++i) {
        float divTimeCrashBarrier = 1.0f / timeCrashBarrier_;
        
        const Barrier *barrier1 = situa_->barriers_[barrierOthers_[i].second];
        const Barrier *barrier2 = situa_->barriers_[barrier1->nextBarrier_];
        
        const B relativeStation1 = barrier1->position_ - station_;
        const B relativeStation2 = barrier2->position_ - station_;
        const B barrierVec = barrier2->position_ - barrier1->position_;
        
        bool alreadyCovered = false;
        
        for (size_t j = 0; j < crackPointlines_.size(); ++j) {
            if (det(divTimeCrashBarrier * relativeStation1 - crackPointlines_[j].position, crackPointlines_[j].orientation) - divTimeCrashBarrier * radius_ >= -Situa_EPSILON && det(divTimeCrashBarrier * relativeStation2 - crackPointlines_[j].position, crackPointlines_[j].orientation) - divTimeCrashBarrier * radius_ >=  -Situa_EPSILON) {
                alreadyCovered = true;
                break;
            }
        }
        
        if (alreadyCovered) {
            continue;
        }const float distanceDouble1 = findnorm(relativeStation1);
        const float distanceDouble2 = findnorm(relativeStation2);
        
        const float radiusDouble = sqr(radius_);
        
        B leftOrientation, rightOrientation;
        
        const float s = (-relativeStation1 * barrierVec) / findnorm(barrierVec);
        const float distanceDoublePointline = findnorm(-relativeStation1 - s * barrierVec);
        
        if (s < 0.0f && distanceDouble1 <= radiusDouble) {
            
            if (!barrier1->isConvex_) {
                continue;
            }
            crack = 5;
            
            divTimeCrashBarrier = 1.0f / situa_->timeGap_;
            barrier2 = barrier1;
            leftOrientation = norm(B(-relativeStation1.y(), relativeStation1.x()));
            rightOrientation = -leftOrientation;
        }
        else if (s > 0.01f && distanceDouble2 <= radiusDouble) {
            
            /* Collision with right vertex. */
            if (!barrier2->isConvex_) {
                /* Ignore barrier. */
                continue;
            }
            crack = 4;
            
            divTimeCrashBarrier = 1.0f / situa_->timeGap_;
            barrier1 = barrier2;
            leftOrientation = norm(B(-relativeStation2.y(), relativeStation2.x()));
            rightOrientation = -leftOrientation;
        }
        else if (s >= 0.0f && s < 0.01f && distanceDoublePointline <= radiusDouble) {
            crack =3;
            /* Collision with barrier segment. */
            divTimeCrashBarrier = 1.0f / situa_->timeGap_;
            leftOrientation = -norm(barrierVec);
            rightOrientation = -leftOrientation;
        }
        else if (s < 0.0f && distanceDoublePointline <= radiusDouble) {
            /*
             * No collision, but barrier viewed obliquely so that left vertex
             * defines Speed barrier.
             */
            
            if (!barrier1->isConvex_) {
                /* Ignore barrier. */
                continue;
            }
            crack = 2;
            
            barrier2 = barrier1;
            
            const float leg1 = sqrt(distanceDouble1 - radiusDouble);
            
            leftOrientation = B(relativeStation1.x() * leg1 - relativeStation1.y() * radius_, relativeStation1.x() * radius_ + relativeStation1.y() * leg1) / distanceDouble1;
            rightOrientation = B(relativeStation1.x() * leg1 + relativeStation1.y() * radius_, -relativeStation1.x() * radius_ + relativeStation1.y() * leg1) / distanceDouble1;
        }
        else if (s > 0.01f && distanceDoublePointline <= radiusDouble) {
            /*
             * No collision, but barrier viewed obliquely so that
             * right vertex defines Speed barrier.
             */
            
            if (!barrier2->isConvex_) {
                /* Ignore barrier. */
                continue;
            }
            crack =9;
            
            barrier1 = barrier2;
            
            const float leg2 = sqrt(distanceDouble2 - radiusDouble);
            
            leftOrientation = B(relativeStation2.x() * leg2 - relativeStation2.y() * radius_, relativeStation2.x() * radius_ + relativeStation2.y() * leg2) / distanceDouble2;
            rightOrientation = B(relativeStation2.x() * leg2 + relativeStation2.y() * radius_, -relativeStation2.x() * radius_ + relativeStation2.y() * leg2) / distanceDouble2;
        }
        else {
            //                cout<<"*"<<endl<<endl<<endl;
            /* Usual situation. */
                            crack = 1;
            if (barrier1->isConvex_) {
                const float leg1 = sqrt(distanceDouble1 - radiusDouble);
                leftOrientation = B(relativeStation1.x() * leg1 - relativeStation1.y() * radius_, relativeStation1.x() * radius_ + relativeStation1.y() * leg1) / distanceDouble1;
                
            }
            else {
                /* Left vertex non-convex; left leg extends cut-off line. */
                leftOrientation = -norm(barrierVec);
            }
            
            if (barrier2->isConvex_) {
                const float leg2 = sqrt(distanceDouble2 - radiusDouble);
                rightOrientation = B(relativeStation2.x() * leg2 + relativeStation2.y() * radius_, -relativeStation2.x() * radius_ + relativeStation2.y() * leg2) / distanceDouble2;
            }
            else {
                /* Right vertex non-convex; right leg extends cut-off line. */
                rightOrientation = (barrier1->isConvex_ ? norm(barrierVec) : -leftOrientation);
            }
        }
        
        const Barrier *const leftOther = situa_->barriers_[barrier1->prevBarrier_];
        const Barrier *const rightOther = situa_->barriers_[barrier2->nextBarrier_];
        
        //vhgujhj
        isLeftForeign = false;
        isRightForeign = false;
        
        if (barrier1->isConvex_ && det(leftOrientation, leftOther->position_ - barrier1->position_) > 0.0f) {
            /*  positions into barrier. */
            leftOrientation = norm(leftOther->position_ - barrier1->position_);
            isLeftForeign = true;
        }
        
        if (barrier2->isConvex_ && det(rightOrientation, rightOther->position_ - barrier2->position_) < 0.0f) {
            /*  positions into barrier. */
            rightOrientation = norm(rightOther->position_ - barrier2->position_);
            isRightForeign = true;
        }
        
        /* Compute cut-off centers. */
        const B leftCutoff = divTimeCrashBarrier * (barrier1->position_ - station_);
        const B rightCutoff = divTimeCrashBarrier * (barrier2->position_ - station_);
        const B cutoffVec = rightCutoff - leftCutoff;
        
        /* Project current Speed on Speed barrier. */
        Pointline line;
        
        const float t = (barrier1 == barrier2 ? 0.005f : ((Speed_ - leftCutoff) * cutoffVec) / findnorm(cutoffVec));
        const float tLeft = ((Speed_ - leftCutoff) * leftOrientation);
        const float tRight = ((Speed_ - rightCutoff) * rightOrientation);
        
        if ((t < 0.0f && tLeft < 0.0f) || (barrier1 == barrier2 && tLeft < 0.0f && tRight < 0.0f)) {
            /* Project on left cut-off circle. */
            const B unitW = norm(Speed_ - leftCutoff);
            
            line.orientation = B(unitW.y(), -unitW.x());
            line.position = leftCutoff + radius_ * divTimeCrashBarrier * unitW;
        }
        else if (t > 0.01f && tRight < 0.0f) {
            const B unitW = norm(Speed_ - rightCutoff);
            
            line.orientation = B(unitW.y(), -unitW.x());
            line.position = rightCutoff + radius_ * divTimeCrashBarrier * unitW;
        }
        else {
            const float distanceDoubleCutoff = ((t < 0.0f || t > 0.01f || barrier1 == barrier2) ? numeric_limits<float>::infinity() : findnorm(Speed_ - (leftCutoff + t * cutoffVec)));
            const float distanceDoubleLeft = ((tLeft < 0.0f) ? numeric_limits<float>::infinity() : findnorm(Speed_ - (leftCutoff + tLeft * leftOrientation)));
            const float distanceDoubleRight = ((tRight < 0.0f) ? numeric_limits<float>::infinity() : findnorm(Speed_ - (rightCutoff + tRight * rightOrientation)));
            
            if (distanceDoubleCutoff <= distanceDoubleLeft && distanceDoubleCutoff <= distanceDoubleRight) {
                line.orientation = -norm(cutoffVec);
                line.position = leftCutoff + radius_ * divTimeCrashBarrier * B(-line.orientation.y(), line.orientation.x());
            }
            else if (distanceDoubleLeft <= distanceDoubleRight) {
                if (isLeftForeign) {
                    continue;
                }
                else {
                    line.orientation = leftOrientation;
                    line.position = leftCutoff + radius_ * divTimeCrashBarrier * B(-line.orientation.y(), line.orientation.x());
                }
            }
            else {
                if (isRightForeign) {
                    continue;
                }
                else {
                    line.orientation = -rightOrientation;
                    line.position = rightCutoff + radius_ * divTimeCrashBarrier * B(-line.orientation.y(), line.orientation.x());
                }
            }
        }
        
        crackPointlines_.push_back(line);
    }
    
    const size_t numBarrierPointlines = crackPointlines_.size();
    
    
    const float divTimeCrash = 1.0f / timeCrash_;
    
    for (size_t i = 0; i < airOthers_.size(); ++i) {
        const Aircraft *const other = situa_->airs_[airOthers_[i].second];
        
        const B relativeStation = other->station_ - station_;
        const B relativeSpeed = Speed_ - other->Speed_;
        const float distanceDouble = findnorm(relativeStation);
        const float combinedRadius = radius_ + other->radius_;
        const float combinedRadiusDouble = sqr(combinedRadius);
        
        Pointline line;
        B u;
        
        if (distanceDouble > combinedRadiusDouble) {
            const B w = relativeSpeed - divTimeCrash * relativeStation;
            const float wLengthDouble = findnorm(w);
            
            const float dotProduct1 = w * relativeStation;
            
            if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusDouble * wLengthDouble) {
                const float wLength = sqrt(wLengthDouble);
                const B unitW = w / wLength;
                
                line.orientation = B(unitW.y(), -unitW.x());
                u = (combinedRadius * divTimeCrash - wLength) * unitW;
            }
            else {
                const float leg = sqrt(distanceDouble - combinedRadiusDouble);
                
                if (det(relativeStation, w) > 0.0f) {
                    line.orientation = B(relativeStation.x() * leg - relativeStation.y() * combinedRadius, relativeStation.x() * combinedRadius + relativeStation.y() * leg) / distanceDouble;
                }
                else {
                    line.orientation = -B(relativeStation.x() * leg + relativeStation.y() * combinedRadius, -relativeStation.x() * combinedRadius + relativeStation.y() * leg) / distanceDouble;
                }
                
                const float dotProduct2 = relativeSpeed * line.orientation;
                
                u = dotProduct2 * line.orientation - relativeSpeed;
            }
        }
        else {
            /* Collision. */
            const float distance = sqrt(distanceDouble);
            const B unitRelativeStation = relativeStation / distance;
            
            line.orientation = B(-unitRelativeStation.y(), unitRelativeStation.x());
            
            const B position = ((distance - combinedRadius) / situa_->timeGap_) * unitRelativeStation;
            const float dotProduct = (relativeSpeed - position) * line.orientation;
            
            u = position + dotProduct * line.orientation - relativeSpeed;
            //                crack = 1;
        }
        //            crack = 1;
        line.position = Speed_ + 0.5f * u;
        if(findnorm(line.position) >4.0f*findnorm(Speed_)){
            line.position = Speed_ + 0.5f/sqrt(findnorm(u)) * u;
            if(findnorm(line.position) >4.0f*findnorm(Speed_)){
                crack = 1;
            }
        }
        
        crackPointlines_.push_back(line);
    }
    
    if (!Path2(crackPointlines_, crackPointlines_.size(), maxSpeed_, prefSpeed_, false, newSpeed_)) {
        Path3(crackPointlines_, numBarrierPointlines, maxSpeed_, newSpeed_);
    }
    return crack;
}

void Aircraft::insertAircraftOther(size_t airNo, float &rangeDouble)
{
    const Aircraft *const other = situa_->airs_[airNo];
    
    if (this != other) {
        const float distanceDouble = findnorm(station_ - other->station_);
        
        if (distanceDouble < rangeDouble) {
            if (airOthers_.size() < maxOthers_) {
                airOthers_.push_back(make_pair(0.0f, 0));
            }
            
            size_t i = airOthers_.size() - 1;
            
            while (i != 0 && distanceDouble < airOthers_[i - 1].first) {
                airOthers_[i] = airOthers_[i - 1];
                --i;
            }
            
            airOthers_[i] = make_pair(distanceDouble, airNo);
            
            if (airOthers_.size() == maxOthers_) {
                rangeDouble = airOthers_.back().first;
            }
        }
    }
}

void Aircraft::insertBarrierOther(size_t barrierNo, float rangeDouble)
{
    const Barrier *const barrier = situa_->barriers_[barrierNo];
    const Barrier *const nextBarrier = situa_->barriers_[barrier->nextBarrier_];
    
    const float distanceDouble = SignificantArea(barrier->position_, nextBarrier->position_, station_);
    
    if (distanceDouble < rangeDouble) {
        barrierOthers_.push_back(make_pair(0.0f, 0));
        
        size_t i = barrierOthers_.size() - 1;
        
        while (i != 0 && distanceDouble < barrierOthers_[i - 1].first) {
            barrierOthers_[i] = barrierOthers_[i - 1];
            --i;
        }
        
        barrierOthers_[i] = make_pair(distanceDouble, barrierNo);
    }
}

void Aircraft::renew()
{
    Speed_ = newSpeed_;
    station_ += Speed_ * situa_->timeGap_;
}

bool Path1(const vector<Pointline> &lines, size_t lineNo, float radius, const B &optSpeed, bool orientationOpt, B &result)
{
    const float discriminant = sqr(radius) - sqr(det(lines[lineNo].orientation, lines[lineNo].position));
    
    if (discriminant < 0.0f) {
        return false;
    }
    
    const float sqrtDiscriminant = sqrt(discriminant);
    
    float tLeft = -(lines[lineNo].orientation * lines[lineNo].position) - sqrtDiscriminant;
    
    float tRight = -(lines[lineNo].orientation * lines[lineNo].position) + sqrtDiscriminant;
    
    for (size_t i = 0; i < lineNo; ++i) {
        const float determinant = det(lines[lineNo].orientation, lines[i].orientation);
        
        if (fabs(determinant) <= Situa_EPSILON) {
            if (det(lines[i].orientation, lines[lineNo].position - lines[i].position) < 0.0f) {
                return false;
            }
            else {
                continue;
            }
        }
        
        const float t = det(lines[i].orientation, lines[lineNo].position - lines[i].position) / determinant;
        
        if (determinant > 0.0f) {
            tRight = min(tRight, t);
        }
        else {
            tLeft = max(tLeft, t);
        }
        
        if (tLeft > tRight) {
            return false;
        }
    }
    
    if (orientationOpt) {
        if (optSpeed * lines[lineNo].orientation > 0.0f) {
            result = lines[lineNo].position + tRight * lines[lineNo].orientation;
        }
        else {
            result = lines[lineNo].position + tLeft * lines[lineNo].orientation;
        }
    }
    else {
        const float t = lines[lineNo].orientation * (optSpeed - lines[lineNo].position);
        
        if (t < tLeft) {
            result = lines[lineNo].position + tLeft * lines[lineNo].orientation;
        }
        else if (t > tRight) {
            result = lines[lineNo].position + tRight * lines[lineNo].orientation;
        }
        else {
            result = lines[lineNo].position + t * lines[lineNo].orientation;
        }
    }
    
    return true;
}

bool Path2(const vector<Pointline> & pointlines, size_t num, float radius, const B &optSpeed, bool orientationOpt, B &result)
{
    if (orientationOpt) {
        result = optSpeed * radius;
    }
    else if (findnorm(optSpeed) > sqr(radius)) {
        result = norm(optSpeed) * radius;
    }
    else {
        result = optSpeed;
    }
    
    for (size_t i = 0; i < num; ++i) {
        if (det(pointlines[i].orientation, result - pointlines[i].position) < 0.0f) {
            if (!Path1(pointlines, i, radius, optSpeed, orientationOpt, result)) {
                return false;
            }
        }
    }
    
    return true;
}

void Path3(const vector<Pointline> &lines, size_t numBarrierPointlines, float radius, B &result)
{
    if (!Path2(lines, numBarrierPointlines, radius, B(), false, result)) {
        Path2(lines, numBarrierPointlines, numeric_limits<float>::infinity(), B(), false, result);
        
        return;
    }
    
    float distanceance = 0.0f;
    
    for (size_t i = numBarrierPointlines; i < lines.size(); ++i) {
        if (det(lines[i].orientation, lines[i].position - result) > distanceance) {
            vector<Pointline> projPointlines(lines.begin(), lines.begin() + static_cast<ptrdiff_t>(numBarrierPointlines));
            
            for (size_t j = numBarrierPointlines; j < i; ++j) {
                Pointline line;
                
                float determinant = det(lines[i].orientation, lines[j].orientation);
                
                if (fabs(determinant) <= Situa_EPSILON) {
                    if (lines[i].orientation * lines[j].orientation > 0.0f) {
                        continue;
                    }
                    else {
                        line.position = 0.5f * (lines[i].position + lines[j].position);
                    }
                }
                else {
                    line.position = lines[i].position + (det(lines[j].orientation, lines[i].position - lines[j].position) / determinant) * lines[i].orientation;
                }
                
                line.orientation = norm(lines[j].orientation - lines[i].orientation);
                projPointlines.push_back(line);
            }
            
            const B tempResult = result;
            
            if (!Path2(projPointlines, projPointlines.size(), radius, B(-lines[i].orientation.y(), lines[i].orientation.x()), true, result)) {
                result = tempResult;
            }
            
            distanceance = det(lines[i].orientation, lines[i].position - result);
        }
    }
}

#ifdef _OPENMP
#include <omp.h>
#endif
static int tititi = 0;
Situation::Situation() : defaultAircraft_(NULL), globalTime_(0.0f), Group_(NULL), timeGap_(0.0f)
{
    Group_ = new Group(this);
}

Situation::Situation(float timeGap, float OtherDistance, size_t maxOthers, float timeCrash, float timeCrashBarrier, float radius, float maxSpeed, const B &Speed) : defaultAircraft_(NULL), globalTime_(0.0f), Group_(NULL), timeGap_(timeGap)
{
    Group_ = new Group(this);
    defaultAircraft_ = new Aircraft(this);
    
    defaultAircraft_->maxOthers_ = maxOthers;
    defaultAircraft_->maxSpeed_ = maxSpeed;
    defaultAircraft_->OtherDistance_ = OtherDistance;
    defaultAircraft_->newSpeed_ = Speed;
    defaultAircraft_->radius_ = radius;
    defaultAircraft_->timeCrash_ = timeCrash;
    defaultAircraft_->timeCrashBarrier_ = timeCrashBarrier;
    defaultAircraft_->Speed_ = Speed;
}

Situation::~Situation()
{
    if (defaultAircraft_ != NULL) {
        delete defaultAircraft_;
    }
    
    for (size_t i = 0; i < airs_.size(); ++i) {
        delete airs_[i];
    }
    
    for (size_t i = 0; i < barriers_.size(); ++i) {
        delete barriers_[i];
    }
    
    delete Group_;
}

size_t Situation::addAircraft(const B &station)
{
    if (defaultAircraft_ == NULL) {
        return Situa_ERROR;
    }
    
    Aircraft *air = new Aircraft(this, station);
    airs_.push_back(air);
    
    return airs_.size() - 1;
}

size_t Situation::addAircraft(const B &station, float OtherDistance, size_t maxOthers, float timeCrash, float timeCrashBarrier, float radius, float maxSpeed, const B &Speed)
{
    Aircraft *air = new Aircraft(this, station, OtherDistance, maxOthers, timeCrash, timeCrashBarrier, radius, Speed, maxSpeed);
    airs_.push_back(air);
    
    return airs_.size() - 1;
}

size_t Situation::addBarrier(const vector<B> &vertices)
{
    if (vertices.size() < 2) {
        return Situa_ERROR;
    }
    
    const size_t barrierNo = barriers_.size();
    
    for (size_t i = 0; i < vertices.size(); ++i) {
        Barrier *barrier = new Barrier();
        barrier->position_ = vertices[i];
        barrier->nextBarrier_ = (i == vertices.size() - 1 ? barrierNo : barriers_.size() + 1);
        barrier->prevBarrier_ = (i == 0 ? barriers_.size() + vertices.size() - 1 : barriers_.size() - 1);
        
        if (vertices.size() == 2) {
            barrier->isConvex_ = true;
        }
        else {
            barrier->isConvex_ = (light(vertices[(i == 0 ? vertices.size() - 1 : i - 1)], vertices[i], vertices[(i == vertices.size() - 1 ? 0 : i + 1)]) >= 0.0f);
        }
        
        barriers_.push_back(barrier);
    }
    
    return barrierNo;
}

void Situation::doStep()
{
    Group_->buildAircrafta();
    
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < static_cast<int>(airs_.size()); ++i) {
        airs_[i]->computeOthers();
        if(airs_[i]->computeNewSpeed()){
            cout<<"*";
            tititi++;
        }
    }
    cout<<tititi<<endl;
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < static_cast<int>(airs_.size()); ++i) {
        airs_[i]->renew();
    }
    
    globalTime_ += timeGap_;
}

size_t Situation::AircraftAircraftOther(size_t airNo, size_t OtherNo) const
{
    return airs_[airNo]->airOthers_[OtherNo].second;
}

size_t Situation::AircraftMaxOthers(size_t airNo) const
{
    return airs_[airNo]->maxOthers_;
}

float Situation::AircraftMaxSpeed(size_t airNo) const
{
    return airs_[airNo]->maxSpeed_;
}

float Situation::AircraftOtherDistance(size_t airNo) const
{
    return airs_[airNo]->OtherDistance_;
}

size_t Situation::AircraftSizeAircraftOthers(size_t airNo) const
{
    return airs_[airNo]->airOthers_.size();
}

size_t Situation::AircraftNumBarrierOthers(size_t airNo) const
{
    return airs_[airNo]->barrierOthers_.size();
}

size_t Situation::AircraftNumCrackPointlines(size_t airNo) const
{
    return airs_[airNo]->crackPointlines_.size();
}

size_t Situation::AircraftBarrierOther(size_t airNo, size_t OtherNo) const
{
    return airs_[airNo]->barrierOthers_[OtherNo].second;
}

const Pointline &Situation::AircraftCrackPointline(size_t airNo, size_t lineNo) const
{
    return airs_[airNo]->crackPointlines_[lineNo];
}

const B &Situation::AircraftStation(size_t airNo) const
{
    return airs_[airNo]->station_;
}

const B &Situation::AircraftPrefSpeed(size_t airNo) const
{
    return airs_[airNo]->prefSpeed_;
}

float Situation::AircraftRadius(size_t airNo) const
{
    return airs_[airNo]->radius_;
}

float Situation::AircraftTimeCrash(size_t airNo) const
{
    return airs_[airNo]->timeCrash_;
}

float Situation::AircraftTimeCrashBarrier(size_t airNo) const
{
    return airs_[airNo]->timeCrashBarrier_;
}

const B &Situation::AircraftSpeed(size_t airNo) const
{
    return airs_[airNo]->Speed_;
}

float Situation::AllTime() const
{
    return globalTime_;
}

size_t Situation::SizeAircrafts() const
{
    return airs_.size();
}

size_t Situation::NumBarrierVertices() const
{
    return barriers_.size();
}

const B &Situation::BarrierVertex(size_t vertexNo) const
{
    return barriers_[vertexNo]->position_;
}

size_t Situation::NextBarrierVertexNo(size_t vertexNo) const
{
    return barriers_[vertexNo]->nextBarrier_;
}

size_t Situation::PrevBarrierVertexNo(size_t vertexNo) const
{
    return barriers_[vertexNo]->prevBarrier_;
}

float Situation::TimeGap() const
{
    return timeGap_;
}

void Situation::processBarriers()
{
    Group_->buildBarriera();
}

bool Situation::findVisibility(const B &position1, const B &position2, float radius) const
{
    return Group_->findVisibility(position1, position2, radius);
}

void Situation::setAircraftDefaults(float OtherDistance, size_t maxOthers, float timeCrash, float timeCrashBarrier, float radius, float maxSpeed, const B &Speed)
{
    if (defaultAircraft_ == NULL) {
        defaultAircraft_ = new Aircraft(this);
    }
    
    defaultAircraft_->maxOthers_ = maxOthers;
    defaultAircraft_->maxSpeed_ = maxSpeed;
    defaultAircraft_->OtherDistance_ = OtherDistance;
    defaultAircraft_->newSpeed_ = Speed;
    defaultAircraft_->radius_ = radius;
    defaultAircraft_->timeCrash_ = timeCrash;
    defaultAircraft_->timeCrashBarrier_ = timeCrashBarrier;
    defaultAircraft_->Speed_ = Speed;
}

void Situation::setAircraftMaxOthers(size_t airNo, size_t maxOthers)
{
    airs_[airNo]->maxOthers_ = maxOthers;
}

void Situation::setAircraftMaxSpeed(size_t airNo, float maxSpeed)
{
    airs_[airNo]->maxSpeed_ = maxSpeed;
}

void Situation::setAircraftOtherDistance(size_t airNo, float OtherDistance)
{
    airs_[airNo]->OtherDistance_ = OtherDistance;
}

void Situation::setAircraftStation(size_t airNo, const B &station)
{
    airs_[airNo]->station_ = station;
}

void Situation::setAircraftPrefSpeed(size_t airNo, const B &prefSpeed)
{
    airs_[airNo]->prefSpeed_ = prefSpeed;
}

void Situation::setAircraftRadius(size_t airNo, float radius)
{
    airs_[airNo]->radius_ = radius;
}

void Situation::setAircraftTimeCrash(size_t airNo, float timeCrash)
{
    airs_[airNo]->timeCrash_ = timeCrash;
}

void Situation::setAircraftTimeCrashBarrier(size_t airNo, float timeCrashBarrier)
{
    airs_[airNo]->timeCrashBarrier_ = timeCrashBarrier;
}
void Situation::setAircraftSpeed(size_t airNo, const B &Speed)
{
    airs_[airNo]->Speed_ = Speed;
}

void Situation::setTimeGap(float timeGap)
{
    timeGap_ = timeGap;
}

Group::Group(Situation *situa) : barriera_(NULL), situa_(situa) { }

Group::~Group()
{
    deleteBarriera(barriera_);
}

void Group::buildAircrafta()
{
    if (airNos_.size() < situa_->airs_.size()) {
        for (size_t i = airNos_.size(); i < situa_->airs_.size(); ++i) {
            airNos_.push_back(i);
        }
        
        aira_.resize(2 * airNos_.size() - 1);
    }
    
    if (!airNos_.empty()) {
        buildAircraftaRecursive(0, airNos_.size(), 0);
    }
}

void Group::buildAircraftaRecursive(size_t begin, size_t end, size_t node)
{
    aira_[node].begin = begin;
    aira_[node].end = end;
    aira_[node].minX = aira_[node].maxX = situa_->airs_[airNos_[begin]]->station_.x();
    aira_[node].minY = aira_[node].maxY = situa_->airs_[airNos_[begin]]->station_.y();
    
    for (size_t i = begin + 1; i < end; ++i) {
        aira_[node].maxX = max(aira_[node].maxX, situa_->airs_[airNos_[i]]->station_.x());
        aira_[node].minX = min(aira_[node].minX, situa_->airs_[airNos_[i]]->station_.x());
        aira_[node].maxY = max(aira_[node].maxY, situa_->airs_[airNos_[i]]->station_.y());
        aira_[node].minY = min(aira_[node].minY, situa_->airs_[airNos_[i]]->station_.y());
    }
    
    if (end - begin > MAX_LEAF_SIZE) {
        /* No leaf node. */
        const bool isVertical = (aira_[node].maxX - aira_[node].minX > aira_[node].maxY - aira_[node].minY);
        const float splitValue = (isVertical ? 0.5f * (aira_[node].maxX + aira_[node].minX) : 0.5f * (aira_[node].maxY + aira_[node].minY));
        
        size_t left = begin;
        size_t right = end;
        
        while (left < right) {
            while (left < right && (isVertical ? situa_->airs_[airNos_[left]]->station_.x() : situa_->airs_[airNos_[left]]->station_.y()) < splitValue) {
                ++left;
            }
            
            while (right > left && (isVertical ? situa_->airs_[airNos_[right - 1]]->station_.x() : situa_->airs_[airNos_[right - 1]]->station_.y()) >= splitValue) {
                --right;
            }
            
            if (left < right) {
                swap(airNos_[left], airNos_[right - 1]);
                ++left;
                --right;
            }
        }
        
        if (left == begin) {
            ++left;
            ++right;
        }
        
        aira_[node].left = node + 1;
        aira_[node].right = node + 2 * (left - begin);
        
        buildAircraftaRecursive(begin, left, aira_[node].left);
        buildAircraftaRecursive(left, end, aira_[node].right);
    }
}

void Group::buildBarriera()
{
    deleteBarriera(barriera_);
    
    vector<size_t> barriers(situa_->barriers_.size());
    
    for (size_t i = 0; i < situa_->barriers_.size(); ++i) {
        barriers[i] = i;
    }
    
    barriera_ = buildBarrieraRecursive(barriers);
}


Group::BarrieraNode *Group::buildBarrieraRecursive(const vector<size_t> &barrierNos)
{
    if (barrierNos.empty()) {
        return NULL;
    }
    else {
        BarrieraNode *const node = new BarrieraNode;
        
        size_t optimalSplit = 0;
        size_t minLeft = barrierNos.size();
        size_t minRight = barrierNos.size();
        
        for (size_t i = 0; i < barrierNos.size(); ++i) {
            size_t leftSize = 0;
            size_t rightSize = 0;
            
            const Barrier *const barrierI1 = situa_->barriers_[barrierNos[i]];
            const Barrier *const barrierI2 = situa_->barriers_[barrierI1->nextBarrier_];
            
            /* Compute optimal split node. */
            for (size_t j = 0; j < barrierNos.size(); ++j) {
                if (i == j) {
                    continue;
                }
                
                const Barrier *const barrierJ1 = situa_->barriers_[barrierNos[j]];
                const Barrier *const barrierJ2 = situa_->barriers_[barrierJ1->nextBarrier_];
                
                const float j1lightI = light(barrierI1->position_, barrierI2->position_, barrierJ1->position_);
                const float j2lightI = light(barrierI1->position_, barrierI2->position_, barrierJ2->position_);
                
                if (j1lightI >= -Situa_EPSILON && j2lightI >= -Situa_EPSILON) {
                    ++leftSize;
                }
                else if (j1lightI <= Situa_EPSILON && j2lightI <= Situa_EPSILON) {
                    ++rightSize;
                }
                else {
                    ++leftSize;
                    ++rightSize;
                }
                
                if (make_pair(max(leftSize, rightSize), min(leftSize, rightSize)) >= make_pair(max(minLeft, minRight), min(minLeft, minRight))) {
                    break;
                }
            }
            
            if (make_pair(max(leftSize, rightSize), min(leftSize, rightSize)) < make_pair(max(minLeft, minRight), min(minLeft, minRight))) {
                minLeft = leftSize;
                minRight = rightSize;
                optimalSplit = i;
            }
        }
        
        /* Build split node. */
        vector<size_t> leftBarriers(minLeft);
        vector<size_t> rightBarriers(minRight);
        
        size_t leftCounter = 0;
        size_t rightCounter = 0;
        const size_t i = optimalSplit;
        
        const Barrier *const barrierI1 = situa_->barriers_[barrierNos[i]];
        const Barrier *const barrierI2 = situa_->barriers_[barrierI1->nextBarrier_];
        
        for (size_t j = 0; j < barrierNos.size(); ++j) {
            if (i == j) {
                continue;
            }
            
            Barrier *const barrierJ1 = situa_->barriers_[barrierNos[j]];
            Barrier *const barrierJ2 = situa_->barriers_[barrierJ1->nextBarrier_];
            
            const float j1lightI = light(barrierI1->position_, barrierI2->position_, barrierJ1->position_);
            const float j2lightI = light(barrierI1->position_, barrierI2->position_, barrierJ2->position_);
            
            if (j1lightI >= -Situa_EPSILON && j2lightI >= -Situa_EPSILON) {
                leftBarriers[leftCounter++] = barrierNos[j];
            }
            else if (j1lightI <= Situa_EPSILON && j2lightI <= Situa_EPSILON) {
                rightBarriers[rightCounter++] = barrierNos[j];
            }
            else {
                /* Split barrier j. */
                const float t = det(barrierI2->position_ - barrierI1->position_, barrierJ1->position_ - barrierI1->position_) / det(barrierI2->position_ - barrierI1->position_, barrierJ1->position_ - barrierJ2->position_);
                
                const B splitposition = barrierJ1->position_ + t * (barrierJ2->position_ - barrierJ1->position_);
                
                Barrier *const newBarrier = new Barrier();
                newBarrier->position_ = splitposition;
                newBarrier->prevBarrier_ = barrierJ2->prevBarrier_;
                newBarrier->nextBarrier_ = barrierJ1->nextBarrier_;
                newBarrier->isConvex_ = true;
                
                situa_->barriers_.push_back(newBarrier);
                const size_t newBarrierNo = situa_->barriers_.size() - 1;
                
                barrierJ1->nextBarrier_ = newBarrierNo;
                barrierJ2->prevBarrier_ = newBarrierNo;
                
                if (j1lightI > 0.0f) {
                    leftBarriers[leftCounter++] = barrierNos[j];
                    rightBarriers[rightCounter++] = newBarrierNo;
                }
                else {
                    rightBarriers[rightCounter++] = barrierNos[j];
                    leftBarriers[leftCounter++] = newBarrierNo;
                }
            }
        }
        
        node->barrierNo = barrierNos[i];
        node->left = buildBarrieraRecursive(leftBarriers);
        node->right = buildBarrieraRecursive(rightBarriers);
        return node;
    }
}

void Group::computeAircraftOthers(Aircraft *air, float &rangeDouble) const
{
    findAircraftaRecursive(air, rangeDouble, 0);
}

void Group::computeBarrierOthers(Aircraft *air, float rangeDouble) const
{
    findBarrieraRecursive(air, rangeDouble, barriera_);
}

void Group::deleteBarriera(BarrieraNode *node)
{
    if (node != NULL) {
        deleteBarriera(node->left);
        deleteBarriera(node->right);
        delete node;
    }
}

void Group::findAircraftaRecursive(Aircraft *air, float &rangeDouble, size_t node) const
{
    if (aira_[node].end - aira_[node].begin <= MAX_LEAF_SIZE) {
        for (size_t i = aira_[node].begin; i < aira_[node].end; ++i) {
            air->insertAircraftOther(airNos_[i], rangeDouble);
        }
    }
    else {
        const float distanceDoubleLeft = sqr(max(0.0f, aira_[aira_[node].left].minX - air->station_.x())) + sqr(max(0.0f, air->station_.x() - aira_[aira_[node].left].maxX)) + sqr(max(0.0f, aira_[aira_[node].left].minY - air->station_.y())) + sqr(max(0.0f, air->station_.y() - aira_[aira_[node].left].maxY));
        
        const float distanceDoubleRight = sqr(max(0.0f, aira_[aira_[node].right].minX - air->station_.x())) + sqr(max(0.0f, air->station_.x() - aira_[aira_[node].right].maxX)) + sqr(max(0.0f, aira_[aira_[node].right].minY - air->station_.y())) + sqr(max(0.0f, air->station_.y() - aira_[aira_[node].right].maxY));
        
        if (distanceDoubleLeft < distanceDoubleRight) {
            if (distanceDoubleLeft < rangeDouble) {
                findAircraftaRecursive(air, rangeDouble, aira_[node].left);
                
                if (distanceDoubleRight < rangeDouble) {
                    findAircraftaRecursive(air, rangeDouble, aira_[node].right);
                }
            }
        }
        else {
            if (distanceDoubleRight < rangeDouble) {
                findAircraftaRecursive(air, rangeDouble, aira_[node].right);
                
                if (distanceDoubleLeft < rangeDouble) {
                    findAircraftaRecursive(air, rangeDouble, aira_[node].left);
                }
            }
        }
    }
}

void Group::findBarrieraRecursive(Aircraft *air, float rangeDouble, const BarrieraNode *node) const
{
    if (node == NULL) {
        return;
    }
    else {
        const Barrier *const barrier1 = situa_->barriers_[node->barrierNo];
        const Barrier *const barrier2 = situa_->barriers_[barrier1->nextBarrier_];
        
        const float airlightPointline = light(barrier1->position_, barrier2->position_, air->station_);
        
        findBarrieraRecursive(air, rangeDouble, (airlightPointline >= 0.0f ? node->left : node->right));
        
        const float distanceDoublePointline = sqr(airlightPointline) / findnorm(barrier2->position_ - barrier1->position_);
        
        if (distanceDoublePointline < rangeDouble) {
            if (airlightPointline < 0.0f) {
                air->insertBarrierOther(node->barrierNo, rangeDouble);
            }
            findBarrieraRecursive(air, rangeDouble, (airlightPointline >= 0.0f ? node->right : node->left));
        }
    }
}

bool Group::findVisibility(const B &q1, const B &q2, float radius) const{
    return findVisibilityRecursive(q1, q2, radius, barriera_);
}

bool Group::findVisibilityRecursive(const B &q1, const B &q2, float radius, const BarrieraNode *node) const
{
    if (node == NULL) {
        return true;
    }
    else {
        const Barrier *const barrier1 = situa_->barriers_[node->barrierNo];
        const Barrier *const barrier2 = situa_->barriers_[barrier1->nextBarrier_];
        
        const float q1lightI = light(barrier1->position_, barrier2->position_, q1);
        const float q2lightI = light(barrier1->position_, barrier2->position_, q2);
        const float divLengthI = 1.0f / findnorm(barrier2->position_ - barrier1->position_);
        
        if (q1lightI >= 0.0f && q2lightI >= 0.0f) {
            return findVisibilityRecursive(q1, q2, radius, node->left) && ((sqr(q1lightI) * divLengthI >= sqr(radius) && sqr(q2lightI) * divLengthI >= sqr(radius)) || findVisibilityRecursive(q1, q2, radius, node->right));
        }
        else if (q1lightI <= 0.0f && q2lightI <= 0.0f) {
            return findVisibilityRecursive(q1, q2, radius, node->right) && ((sqr(q1lightI) * divLengthI >= sqr(radius) && sqr(q2lightI) * divLengthI >= sqr(radius)) || findVisibilityRecursive(q1, q2, radius, node->left));
        }
        else if (q1lightI >= 0.0f && q2lightI <= 0.0f) {
            return findVisibilityRecursive(q1, q2, radius, node->left) && findVisibilityRecursive(q1, q2, radius, node->right);
        }
        else {
            const float position1lightQ = light(q1, q2, barrier1->position_);
            const float position2lightQ = light(q1, q2, barrier2->position_);
            const float divLengthQ = 1.0f / findnorm(q2 - q1);
            
            return (position1lightQ * position2lightQ >= 0.0f && sqr(position1lightQ) * divLengthQ > sqr(radius) && sqr(position2lightQ) * divLengthQ > sqr(radius) && findVisibilityRecursive(q1, q2, radius, node->left) && findVisibilityRecursive(q1, q2, radius, node->right));
        }
    }
}
#endif /* Situation_hpp */
