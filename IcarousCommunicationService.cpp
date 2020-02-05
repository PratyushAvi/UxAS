// ===============================================================================
// Authors: AFRL/RQQA & NASA/NIA
// Organizations: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
//                National Aeronautics and Space Administration, National Institute of Aerospace
// 
// Copyright (c) 2017 Government of the United State of America, as represented by
// the Secretary of the Air Force.  No copyright is claimed in the United States under
// Title 17, U.S. Code.  All Other Rights Reserved.
// ===============================================================================

/* 
 * File:   IcarousCommunicationService.cpp
 * Authors: Paul Coen & Winston Smith
 *
 * Created on June 14, 2018, 3:55 PM
 *
 * <Service Type="IcarousCommunicationService" NumberOfUAVs="n" />
 * 
 * This file allows connectivity with the ICAROUS system
 * (CRoss Application Translator of Operational Unmanned Systems) 
 * ICAROUS allows cooperative mission planning between UxAS and ICAROUS
 * 
 *
 *  Things still to do:
 *   TODO - Use UxAS logging function and UxAS print statements
 *
 *   TODO processReceivedLmcpMessage - Find if there is a place to get relative altitude from AMASE for the UAVs position
 *                                     Currently, only one is stored; not sure if this is possible
 *   TODO processReceivedLmcpMessage - Locate a place where horizonal accuracy is defined (hdop)
 *   TODO processReceivedLmcpMessage - Locate a place where vertical accuracy is defined (vdop)
 *   TODO processReceivedLmcpMessage - Find a way to get the yaw from AMASE for a given UAV (not default included in the message)
 *   
 *
 *  Notes:
 *   This code ONLY works on Linux, since it uses Linux function calls (dprintf)
 *   Built and tested on Ubuntu 16.04 32-bit with ICAROUS 2.1
 *   
 *   *************************************************************************************************
 *   * IMPORTANT: There are several known security vulnerabilities in this file. The fixes for these *
 *   * may be closed-source. Do not use this code for real applications without modifications!       *
 *   *************************************************************************************************
 */


// Include header for this service
#include "IcarousCommunicationService.h"
#include "WaypointPlanManagerService.h"

#include <iostream>

#include "afrl/cmasi/AirVehicleState.h"
#include "afrl/cmasi/AirVehicleConfiguration.h"
#include "afrl/cmasi/AirVehicleStateDescendants.h"
#include "afrl/cmasi/Polygon.h"
#include "afrl/cmasi/AbstractGeometry.h"
#include "afrl/cmasi/AutomationResponse.h"
#include "afrl/cmasi/CommandStatusType.h"
#include "afrl/cmasi/FlightDirectorAction.h"
#include "afrl/cmasi/GimbalAngleAction.h"
#include "afrl/cmasi/GoToWaypointAction.h"
#include "afrl/cmasi/Location3D.h"
#include "afrl/cmasi/LoiterAction.h"
#include "afrl/cmasi/MissionCommand.h"
#include "afrl/cmasi/NavigationMode.h"
#include "afrl/cmasi/perceive/TrackEntityAction.h"
#include "afrl/cmasi/VehicleActionCommand.h"

// Unsure if needed
//#include "uxas/messages/route/RoutePlan.h"


#include "uxas/messages/task/TaskPause.h"
#include "uxas/messages/task/TaskResume.h"
#include "uxas/messages/uxnative/IncrementWaypoint.h"

#include "Constants/UxAS_String.h"

// Convenience definitions for the option strings
#define STRING_XML_OPTION_STRING "OptionString"
#define STRING_XML_OPTION_INT "OptionInt"

// Namespace definitions
namespace uxas  // uxas::
{
namespace service   // uxas::service::
{



// This entry registers the service in the service creation registry
IcarousCommunicationService::ServiceBase::CreationRegistrar<IcarousCommunicationService>
IcarousCommunicationService::s_registrar(IcarousCommunicationService::s_registryServiceTypeNames());



// Service constructor
IcarousCommunicationService::IcarousCommunicationService()
: ServiceBase(IcarousCommunicationService::s_typeName(), IcarousCommunicationService::s_directoryName()) { };



// Service destructor
IcarousCommunicationService::~IcarousCommunicationService() { };



// Add subscriptions to other services & grab the xml configurations set for the mission
bool IcarousCommunicationService::configure(const pugi::xml_node& ndComponent)
{
    bool isSuccess(true);

    
    // AirVehicleStates are returned from OpenAMASE to know where a UAV is and what it is doing
    addSubscriptionAddress(afrl::cmasi::AirVehicleState::Subscription);
    
    // Need the aircraft's nominal speed from this
    addSubscriptionAddress(afrl::cmasi::AirVehicleConfiguration::Subscription);
    
    return (isSuccess);
}



bool IcarousCommunicationService::vectorContainsInt(int needle, std::vector<int> haystack){
    bool isFound = false;
    for(int toCheck : haystack){
        if(toCheck == needle){
            isFound = true;
            break;
        }
    }
    return isFound;
}

bool IcarousCommunicationService::isAdjustedThisIteration(int IDtoCheck){
    if(IDtoCheck >= 0){
        return vectorContainsInt(IDtoCheck, adjustedIDs);
    }
    else{
        adjustedIDs.clear();
        return false;
    }
}



// Start up the module, initialize variables, and connect to ICAROUS instance(s)
bool IcarousCommunicationService::initialize()
{
    // Perform any required initialization before the service is started
    
    hasUpdated.resize(NUM_UAVS + NUM_MONITOR);
    vehicleStates.resize(NUM_UAVS + NUM_MONITOR);
    
    hasUpdated.assign(NUM_UAVS + NUM_MONITOR, false);
    vehicleStates.assign(NUM_UAVS + NUM_MONITOR, NULL);

    // Initialization was successful
    return true;
}

bool IcarousCommunicationService::vectorContainsOnlyConstraintTypes(constraintTypes hay, std::vector<constraintTypes> haystack){
    for(constraintTypes currType : haystack){
        if(currType != hay){
            return false;
        }
    }
    return true;
}

std::vector<int> *IcarousCommunicationService::sliceRequirementIDs(int startIndex, 
                                                                std::vector<constraintTypes> haystack,
                                                                std::vector<int> vectorToSlice){
    std::vector<int> *sliceToReturn = new std::vector<int>;
    constraintTypes typeToSlice = haystack[startIndex];
    while(haystack[startIndex] == typeToSlice){
        sliceToReturn->push_back(vectorToSlice[startIndex++]);
    }
    return sliceToReturn;
}

bool IcarousCommunicationService::vectorIntsEqual(std::vector<int> left, std::vector<int> right, 
                                                  bool orderMatters){
    //std::cout << "what\n";
    if(left.size() != right.size()){
        return false;
    }
    if(orderMatters){
        for(int i = 0; i < left.size(); i++){
            bool found = false;
            
            for(int j = 0; j < right.size(); j++){
                if(left[i] == right[j]){
                    found = true;
                    left.erase(left.begin() + i);
                    right.erase(right.begin() + j);
                    i--;
                    j = right.size();
                }
            }
            if(!found){
                //std::cout << "no\n";
                return false;
            }
        }
        if(left.size() != 0 || right.size() != 0){
            //std::cout << "NO\n";
            return false;
        }
    }
    else{
        for(int i = 0; i < left.size(); i++){
            if(left[i] != right[i]){
                //std::cout << "no\n";
                return false;
            }
        }
    }
    //std::cout << "yes\n";
    return true;
}

IcarousCommunicationService::constraintTypes IcarousCommunicationService::findNextConstraintTypeWithPosition
    (std::vector<constraintTypes> haystack, int *position){
    for(*position = 0; *position < haystack.size(); *position++){
        constraintTypes currType = haystack[*position];
        if(currType != invalid){
            return currType;
        }
    }
    return invalid;
}

bool IcarousCommunicationService::nodeIsPresentInGraph(constraintNode *nodeToAdd, 
                                                       constraintNode **otherNode, 
                                                       std::vector<constraintNode *> constraintGraph){
    //std::cout << "nodeIsPresentInGraph called\n";
    for(constraintNode *currNode : constraintGraph){
        if(constraintsEqual(*currNode->data, *nodeToAdd->data, (nodeToAdd->data->type != centroid)) 
           && !(nodeToAdd == currNode)){
            *otherNode = currNode;
            //std::cout << "nodeIsPresentInGraph exited true\n";
            return true;
        }
    }
    //std::cout << "nodeIsPresentInGraph exited false\n";
    return false;
}

bool IcarousCommunicationService::constraintsEqual(constraint left, constraint right, bool orderMatters){
    //this function checks every attribute of left and right to see whether or not the constraints
    //are identical. Returns true if they are, or false otherwise
    //std::cout << "test\n";
    if(left.type != right.type){
        return false;
    }
    else if(!vectorIntsEqual(left.groupIDs, right.groupIDs, orderMatters)){
        return false;
    }
    else if(left.type == monitor){
        if(!vectorIntsEqual(left.monitorIDs, right.monitorIDs, orderMatters)){
            return false;
        }
    }
    //std::cout << "test2\n";
    return true;
}
bool IcarousCommunicationService::nodeCombosEqual(std::vector<constraintNode *> comboToAdd,
                                                  std::vector<constraintNode *> otherCombo){
    
    constraintNode *junk;
    for(constraintNode *currNode : comboToAdd){
        if(!nodeIsPresentInGraph(currNode, &junk, otherCombo)){
            return false;
        }
    }
    return true;
}

bool IcarousCommunicationService::ruleApplies(inferenceRule *ruleToCheck, 
                                              std::vector<constraintNode *> constraintGraph){
    if(ruleToCheck == NULL){
        nodeCombosThisIteration.clear();
    }
    else{
        bool anyNodeFound = false;
        for(constraintNode *currNode : constraintGraph){
            for(int i = 0; i < ruleToCheck->requirementTypes.size(); i++){
                std::vector<int> *requirementSlice;
                requirementSlice = sliceRequirementIDs(i, ruleToCheck->requirementTypes, ruleToCheck->requirementIDs);
                if(requirementSlice->size() == 0){
                    return false;
                }
                if(currNode->data->type == ruleToCheck->requirementTypes[i] && 
                   vectorIntsEqual(currNode->data->groupIDs, *requirementSlice, 
                                  (currNode->data->type != centroid))){
                   
                    std::vector<constraintTypes> remainingRequirements = ruleToCheck->requirementTypes;
                    constraintTypes foundType = remainingRequirements[i];
                    for(int j = i; remainingRequirements[j] == foundType && j < remainingRequirements.size(); j++){
                        foundType = remainingRequirements[j];
                        remainingRequirements[j] = invalid;
                    }
                    std::vector<constraintNode *> applicableNodes;
                    applicableNodes.push_back(currNode);
                    anyNodeFound = true;
                    
                    //find other requirements
                    while(!vectorContainsOnlyConstraintTypes(invalid, remainingRequirements)){
                        
                        bool nodeFound = false;
                        int pos;
                        constraintTypes currType;
                        currType = findNextConstraintTypeWithPosition(remainingRequirements, &pos);
                        requirementSlice->clear();
                        requirementSlice = sliceRequirementIDs(pos, ruleToCheck->requirementTypes, 
                                                               ruleToCheck->requirementIDs);
                                                               
                        for(int j = 0; j < constraintGraph.size(); j++){
                            constraintNode *otherNode = constraintGraph[j];
                            if(otherNode->data->type == currType && 
                               vectorIntsEqual(otherNode->data->groupIDs, *requirementSlice,
                                              (currType != centroid)) && 
                               currNode != otherNode){
                               
                                applicableNodes.push_back(otherNode);
                                nodeFound = true;
                                while(remainingRequirements[pos] == currType && pos < remainingRequirements.size()){
                                    remainingRequirements[pos++] = invalid;
                                }
                            }
                        }
                        if(!nodeFound){
                            return false;
                        }
                        requirementSlice->clear();
                    }/*
                    for(int k = 0; k < applicableNodes.size(); k++){
                        for(int l = 0; l < applicableNodes[k]->data->groupIDs.size(); l++){
                            std::cout << applicableNodes[k]->data->groupIDs[l] << " ";
                        }
                        std::cout << std::endl;
                    }*/
                    bool isFound = false;
                    for(int k = 0; k < nodeCombosThisIteration.size(); k++){
                        if(nodeCombosEqual(applicableNodes, nodeCombosThisIteration[k])){
                            isFound = true;
                        }
                    }
                    if(!isFound){
                        //std::cout << "Pushing combo #" << nodeCombosThisIteration.size() + 1 << std::endl;
                        nodeCombosThisIteration.push_back(applicableNodes);
                        rulesAppliedThisIteration.push_back(*ruleToCheck);
                    }
                }
            }
        }
        if(!anyNodeFound){
            return false;
        }
    }
    return true;
}

void IcarousCommunicationService::gatherDescendents(constraintNode *node, 
                                                    std::vector<constraintNode *> *descendentsFound){
    descendentsFound->push_back(node);
    for(constraintNode *currNode : node->children){
        gatherDescendents(currNode, descendentsFound);
    }
}

bool IcarousCommunicationService::descendentsAreSuperset(constraintNode *nodeToAdd, constraintNode *otherNode){
    //checks if descendents of nodeToAdd are superset of descendents of otherNode
    std::vector<constraintNode *> nodeChildren;
    std::vector<constraintNode *> otherChildren;
    gatherDescendents(nodeToAdd, &nodeChildren);
    gatherDescendents(otherNode, &otherChildren);
    
    for(int i = 0; i < otherChildren.size(); i++){
        constraintNode *nodeToCheck = otherChildren[i];
        bool nodeFound = false;
        for(int j = 0; j < nodeChildren.size(); j++){
            constraintNode *possibleEqualNode = nodeChildren[j];
            if(constraintsEqual(*nodeToCheck->data, *possibleEqualNode->data, (nodeToCheck->data->type != centroid))){
                nodeFound = true;
                nodeChildren.erase(nodeChildren.begin() + j);
                j = nodeChildren.size();
                otherChildren.erase(otherChildren.begin() + i);
                i--;
                break;
            }
        }
        if(!nodeFound){
            return false;
        }
    }
    if(otherChildren.size() == 0){
        return true;
    }
    else{
        return false;
    }
}

bool IcarousCommunicationService::checkCompatibility(std::vector<constraintNode *> constraintGraph){
    bool continueLoop = true;
    //std::cout << "check begun\n";
    while(continueLoop){
        continueLoop = false;
        for(inferenceRule currRule : ruleList){
            if(ruleApplies(&currRule, constraintGraph)){
                continueLoop = true;
            }
        }
        if(continueLoop){
            continueLoop = false;
            for(int j = 0; j < nodeCombosThisIteration.size(); j++){
                std::vector<constraintNode *> currentCombo = nodeCombosThisIteration[j];
                inferenceRule currRule = rulesAppliedThisIteration[j];
                for(int i = 0; i < currRule.resultTypes.size(); i++){
                    constraintNode *otherNode;
                    constraintNode *nodeToAdd = new constraintNode;
                    nodeToAdd->data = new constraint;
                    constraintTypes currType = currRule.resultTypes[i];
                    
                    
                    if(currType == global){
                        nodeToAdd->data->type = global;
                        nodeToAdd->data->groupIDs.push_back(currRule.resultIDs[i]);
                        for(constraintNode *currNode : currentCombo){
                            nodeToAdd->children.push_back(currNode);
                            currNode->parents.push_back(nodeToAdd);
                        }
                        if(!nodeIsPresentInGraph(nodeToAdd, &otherNode, constraintGraph)){
                            constraintGraph.push_back(nodeToAdd);
                            continueLoop = true;
                        }
                        else if(descendentsAreSuperset(nodeToAdd, otherNode)){
                            nodeToAdd->children.clear();
                            for(constraintNode *currNode : currentCombo){
                                currNode->parents.pop_back();
                            }
                            continue;
                        }
                        else{
                            nodeToAdd->children.clear();
                            for(constraintNode *currNode : currentCombo){
                                currNode->parents.pop_back();
                            }
                            nodeCombosThisIteration.clear();
                            rulesAppliedThisIteration.clear();
                            //std::cout << "check ended\n";
                            return false;
                        }
                    }
                    
                    else if(currType == centroid){
                        //std::cout << "centr\n";
                        nodeToAdd->data->type = centroid;
                        while(currType == centroid && i < currRule.resultTypes.size()){
                            currType = currRule.resultTypes[i];
                            nodeToAdd->data->groupIDs.push_back(currRule.resultIDs[i++]);
                        }
                        for(constraintNode *currNode : currentCombo){
                            nodeToAdd->children.push_back(currNode);
                            currNode->parents.push_back(nodeToAdd);
                        }
                        if(!nodeIsPresentInGraph(nodeToAdd, &otherNode, constraintGraph)){
                            //std::cout << "added\n";
                            constraintGraph.push_back(nodeToAdd);
                            continueLoop = true;
                        }
                        else if(descendentsAreSuperset(nodeToAdd, otherNode)){
                            //std::cout << "continued\n";
                            nodeToAdd->children.clear();
                            for(constraintNode *currNode : currentCombo){
                                currNode->parents.pop_back();
                            }
                            continue;
                        }
                        else{
                            //std::cout << "false\n";
                            nodeToAdd->children.clear();
                            for(constraintNode *currNode : currentCombo){
                                currNode->parents.pop_back();
                            }
                            nodeCombosThisIteration.clear();
                            rulesAppliedThisIteration.clear();
                            return false;
                        }
                    }
                    
                    else if(currType == monitor){
                        nodeToAdd->data->type = monitor;
                        nodeToAdd->data->groupIDs.push_back(currRule.resultIDs[i]);
                        i++;
                        while(currType == monitor && i < currRule.resultTypes.size()){
                            currType = currRule.resultTypes[i];
                            nodeToAdd->data->monitorIDs.push_back(currRule.resultIDs[i]);
                            i++;
                        }
                        for(constraintNode *currNode : currentCombo){
                            nodeToAdd->children.push_back(currNode);
                            currNode->parents.push_back(nodeToAdd);
                        }
                        if(!nodeIsPresentInGraph(nodeToAdd, &otherNode, constraintGraph)){
                            constraintGraph.push_back(nodeToAdd);
                            continueLoop = true;
                        }
                        else if(descendentsAreSuperset(nodeToAdd, otherNode)){
                            nodeToAdd->children.clear();
                            for(constraintNode *currNode : currentCombo){
                                currNode->parents.pop_back();
                            }
                            continue;
                        }
                        else{
                            //std::cout << "check ended\n";
                            nodeToAdd->children.clear();
                            for(constraintNode *currNode : currentCombo){
                                currNode->parents.pop_back();
                            }
                            nodeCombosThisIteration.clear();
                            rulesAppliedThisIteration.clear();
                            return false;
                        }
                    }
                    
                    else if(currType == relative){
                        nodeToAdd->data->type = relative;
                        nodeToAdd->data->groupIDs.push_back(currRule.resultIDs[i]);
                        i++;
                        nodeToAdd->data->groupIDs.push_back(currRule.resultIDs[i]);
                        for(constraintNode *currNode : currentCombo){
                            nodeToAdd->children.push_back(currNode);
                            currNode->parents.push_back(nodeToAdd);
                        }
                        if(!nodeIsPresentInGraph(nodeToAdd, &otherNode, constraintGraph)){
                            continueLoop = true;
                            if(currentCombo[0]->data->type != monitor){
                                //std::cout << "got here true??\n";
                            }
                            constraintGraph.push_back(nodeToAdd);
                        }
                        else if(descendentsAreSuperset(nodeToAdd, otherNode)){
                            if(currentCombo[0]->data->type == monitor){
                                //std::cout << "got here continue\n";
                            }
                            nodeToAdd->children.clear();
                            for(constraintNode *currNode : currentCombo){
                                currNode->parents.pop_back();
                            }
                            continue;
                        }
                        else{
                        
                            if(currentCombo[0]->data->type != monitor){
                                //std::cout << "got here false\n";
                            }
                            nodeToAdd->children.clear();
                            for(constraintNode *currNode : currentCombo){
                                currNode->parents.pop_back();
                            }
                            nodeCombosThisIteration.clear();
                            rulesAppliedThisIteration.clear();
                            return false;
                        }
                    }
                    
                    else{
                        std::cout << "CONSTRAINTS: Invalid constraint type given.\n";
                        return false;
                    }
                }
            }
        }
        nodeCombosThisIteration.clear();
        rulesAppliedThisIteration.clear();
    }
    return true;
}



// This function is used to start the service and the ICAROUS listening side of the program
bool IcarousCommunicationService::start()
{
    // perform any actions required at the time the service starts
    //std::cout << "*** STARTING:: Service[" << s_typeName() << "] Service Id[" << m_serviceId << "] with working directory [" << m_workDirectoryName << "] *** " << std::endl;
    
    //dev note: intiial centroid for scenario 1: {-80.7654, 25.3723};
    
    std::cout << "Initializing rule library..." << std::endl;
    
    //TODO read inference rules from an external file (for example, XML)
    //For now, add new rules here.
    
    inferenceRule *newRule = new inferenceRule;
    
    //monitoring-------------------------------------------------------------------------
    newRule->requirementIDs = {1, 2};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {1, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 3};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {1, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 4};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {1, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 5};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {1, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 6};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {1, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 7};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {1, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 1};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {2, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 3};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {2, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 4};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {2, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 5};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {2, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 6};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {2, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 7};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {2, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 1};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {3, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 2};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {3, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 4};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {3, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 5};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {3, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 6};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {3, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 7};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {3, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 1};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {4, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 2};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {4, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 3};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {4, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 5};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {4, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 6};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {4, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 7};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {4, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 1};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {5, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 2};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {5, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 3};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {5, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 4};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {5, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 6};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {5, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 7};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {5, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 1};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {6, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 2};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {6, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 3};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {6, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 4};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {6, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 5};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {6, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 7};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {6, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 1};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {7, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 2};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {7, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 3};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {7, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 4};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {7, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 5};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {7, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 6};
    newRule->requirementTypes = {monitor, monitor};
    newRule->resultIDs = {7, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    //global + relative = global---------------------------------------------------------
    /*
    newRule->requirementIDs = {1, 2, 1};
    newRule->requirementTypes = {global, relative, relative};
    newRule->resultIDs = {2};
    newRule->resultTypes = {global};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 3, 1};
    newRule->requirementTypes = {global, relative, relative};
    newRule->resultIDs = {3};
    newRule->resultTypes = {global};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 4, 1};
    newRule->requirementTypes = {global, relative, relative};
    newRule->resultIDs = {2};
    newRule->resultTypes = {global};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 1, 2};
    newRule->requirementTypes = {global, relative, relative};
    newRule->resultIDs = {1};
    newRule->resultTypes = {global};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 3, 2};
    newRule->requirementTypes = {global, relative, relative};
    newRule->resultIDs = {3};
    newRule->resultTypes = {global};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 4, 2};
    newRule->requirementTypes = {global, relative, relative};
    newRule->resultIDs = {1};
    newRule->resultTypes = {global};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 1, 3};
    newRule->requirementTypes = {global, relative, relative};
    newRule->resultIDs = {1};
    newRule->resultTypes = {global};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 2, 3};
    newRule->requirementTypes = {global, relative, relative};
    newRule->resultIDs = {2};
    newRule->resultTypes = {global};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 4, 3};
    newRule->requirementTypes = {global, relative, relative};
    newRule->resultIDs = {1};
    newRule->resultTypes = {global};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 1, 4};
    newRule->requirementTypes = {global, relative, relative};
    newRule->resultIDs = {1};
    newRule->resultTypes = {global};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 2, 4};
    newRule->requirementTypes = {global, relative, relative};
    newRule->resultIDs = {1};
    newRule->resultTypes = {global};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 3, 4};
    newRule->requirementTypes = {global, relative, relative};
    newRule->resultIDs = {1};
    newRule->resultTypes = {global};
    ruleList.push_back(*newRule);
    */
    //rules for 1-7 relative to each other----------------------------------------------------
    /*
    newRule->requirementIDs = {2, 1};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {1, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 2};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {2, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 3};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {3, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 1};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {1, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 1};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {1, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 4};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {4, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 5};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {5, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 1};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {1, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 6};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {6, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 1};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {1, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 7};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {7, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 1};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {1, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 4};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {4, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 3};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {3, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 2};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {2, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 4};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {4, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 3};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {3, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 2};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {2, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 5};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {5, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 2};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {2, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 6};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {6, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 2};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {2, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 7};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {7, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 2};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {2, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 5};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {5, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 3};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {3, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 3};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {3, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 6};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {6, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 3};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {3, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 7};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {7, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 5};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {5, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 4};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {4, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 6};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {6, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 4};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {4, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 7};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {7, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 4};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {4, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 6};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {6, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 5};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {5, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 7};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {7, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 5};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {5, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 7};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {7, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 6};
    newRule->requirementTypes = {relative, relative};
    newRule->resultIDs = {6, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    */
    //two-vehicle centroid----------------------------------------------------------------
    
    //if all vehicles except one in a centroid are constrained, the last one becomes constrained
    newRule->requirementIDs = {1, 2, 2, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 2, 2, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 2, 2, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 2, 2, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 2, 2, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 2, 2, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 3, 3, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 3, 3, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 3, 3, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 3, 3, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 3, 3, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 3, 3, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 4, 4, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 4, 4, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 4, 4, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 4, 4, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 4, 4, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 4, 4, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 5, 5, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 5, 5, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 5, 5, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 5, 5, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 5, 5, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 5, 5, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 6, 6, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 6, 6, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 6, 6, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 6, 6, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 6, 6, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 6, 6, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 7, 7, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 7, 7, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 7, 7, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 7, 7, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 7, 7, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 7, 7, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {1, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 1, 1, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 1, 1, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 1, 1, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 1, 1, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 1, 1, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 1, 1, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 3, 3, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 3, 3, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 3, 3, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 3, 3, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 3, 3, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 3, 3, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 4, 4, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 4, 4, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 4, 4, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 4, 4, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 4, 4, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 4, 4, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 5, 5, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 5, 5, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 5, 5, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 5, 5, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 5, 5, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 5, 5, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 6, 6, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 6, 6, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 6, 6, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 6, 6, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 6, 6, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 6, 6, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 7, 7, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 7, 7, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 7, 7, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 7, 7, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 7, 7, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 7, 7, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {2, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 1, 1, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 1, 1, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 1, 1, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 1, 1, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 1, 1, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 1, 1, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 2, 2, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 2, 2, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 2, 2, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 2, 2, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 2, 2, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 2, 2, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 4, 4, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 4, 4, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 4, 4, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 4, 4, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 4, 4, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 4, 4, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 5, 5, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 5, 5, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 5, 5, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 5, 5, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 5, 5, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 5, 5, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 6, 6, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 6, 6, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 6, 6, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 6, 6, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 6, 6, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 6, 6, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 7, 7, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 7, 7, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 7, 7, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 7, 7, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 7, 7, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 7, 7, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {3, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 1, 1, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 1, 1, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 1, 1, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 1, 1, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 1, 1, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 1, 1, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 2, 2, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 2, 2, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 2, 2, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 2, 2, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 2, 2, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 2, 2, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 3, 3, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 3, 3, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 3, 3, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 3, 3, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 3, 3, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 3, 3, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 5, 5, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 5, 5, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 5, 5, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 5, 5, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 5, 5, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 5, 5, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 6, 6, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 6, 6, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 6, 6, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 6, 6, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 6, 6, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 6, 6, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 7, 7, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 7, 7, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 7, 7, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 7, 7, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 7, 7, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 7, 7, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {4, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 1, 1, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 1, 1, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 1, 1, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 1, 1, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 1, 1, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 1, 1, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 2, 2, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 2, 2, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 2, 2, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 2, 2, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 2, 2, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 2, 2, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 3, 3, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 3, 3, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 3, 3, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 3, 3, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 3, 3, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 3, 3, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 4, 4, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 4, 4, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 4, 4, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 4, 4, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 4, 4, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 4, 4, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 6, 6, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 6, 6, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 6, 6, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 6, 6, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 6, 6, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 6, 6, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 7, 7, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 7, 7, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 7, 7, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 7, 7, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 7, 7, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 7, 7, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {5, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 1, 1, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 1, 1, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 1, 1, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 1, 1, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 1, 1, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 1, 1, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 2, 2, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 2, 2, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 2, 2, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 2, 2, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 2, 2, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 2, 2, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 3, 3, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 3, 3, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 3, 3, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 3, 3, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 3, 3, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 3, 3, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 4, 4, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 4, 4, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 4, 4, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 4, 4, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 4, 4, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 4, 4, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 5, 5, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 5, 5, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 5, 5, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 5, 5, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 5, 5, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 5, 5, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 7, 7, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 7, 7, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 7, 7, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 7, 7, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 7, 7, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 7, 7, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {6, 7};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 1, 1, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 1, 1, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 1, 1, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 1, 1, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 1, 1, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 1, 1, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 1};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 2, 2, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 2, 2, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 2, 2, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 2, 2, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 2, 2, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 2, 2, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 2};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 3, 3, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 3, 3, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 3, 3, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 3, 3, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 3, 3, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 3, 3, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 3};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 4, 4, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 4, 4, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 4, 4, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 4, 4, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 4, 4, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 4, 4, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 4};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 5, 5, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 5, 5, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 5, 5, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 5, 5, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 5, 5, 6};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 5, 5, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 5};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 6, 6, 1};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 6, 6, 2};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 6, 6, 3};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 6, 6, 4};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 6, 6, 5};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 6, 6, 7};
    newRule->requirementTypes = {centroid, centroid, relative, relative};
    newRule->resultIDs = {7, 6};
    newRule->resultTypes = {relative, relative};
    ruleList.push_back(*newRule);
    
    //centroids are reflexive
    newRule->requirementIDs = {1, 2};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {2, 1};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 3};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {3, 1};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 4};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {4, 1};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 5};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {5, 1};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 6};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {6, 1};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 7};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {7, 1};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 1};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {1, 2};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 3};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {3, 2};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 4};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {4, 2};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 5};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {5, 2};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 6};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {6, 2};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 7};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {7, 2};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 1};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {1, 3};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 2};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {2, 3};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 4};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {4, 3};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 5};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {5, 3};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 6};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {6, 3};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 7};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {7, 3};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 1};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {1, 4};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 2};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {2, 4};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 3};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {3, 4};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 5};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {5, 4};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 6};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {6, 4};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {4, 7};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {7, 4};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 1};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {1, 5};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 2};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {2, 5};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 3};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {3, 5};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 4};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {4, 5};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 6};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {6, 5};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {5, 7};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {7, 5};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 1};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {1, 6};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 2};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {2, 6};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 3};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {3, 6};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 4};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {4, 6};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 5};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {5, 6};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {6, 7};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {7, 6};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 1};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {1, 7};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 2};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {2, 7};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 3};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {3, 7};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 4};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {4, 7};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 5};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {5, 7};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {7, 6};
    newRule->requirementTypes = {centroid, centroid};
    newRule->resultIDs = {6, 7};
    newRule->resultTypes = {centroid, centroid};
    ruleList.push_back(*newRule);
    /*
    //rules for three-vehicle centroids-------------------------------------------------
    newRule->requirementIDs = {1, 2, 3};
    newRule->requirementTypes = {centroid, relative, relative};
    newRule->resultIDs = {1, 2, 2, 3, 1, 3, 2, 1, 3, 2, 3, 1};
    newRule->resultTypes = {relative, relative, relative, relative, relative, relative, relative, relative, relative, relative, relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {1, 3, 2};
    newRule->requirementTypes = {centroid, relative, relative};
    newRule->resultIDs = {1, 2, 2, 3, 1, 3, 2, 1, 3, 2, 3, 1};
    newRule->resultTypes = {relative, relative, relative, relative, relative, relative, relative, relative, relative, relative, relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 1, 3};
    newRule->requirementTypes = {centroid, relative, relative};
    newRule->resultIDs = {1, 2, 2, 3, 1, 3, 2, 1, 3, 2, 3, 1};
    newRule->resultTypes = {relative, relative, relative, relative, relative, relative, relative, relative, relative, relative, relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {2, 3, 1};
    newRule->requirementTypes = {centroid, relative, relative};
    newRule->resultIDs = {1, 2, 2, 3, 1, 3, 2, 1, 3, 2, 3, 1};
    newRule->resultTypes = {relative, relative, relative, relative, relative, relative, relative, relative, relative, relative, relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 2, 1};
    newRule->requirementTypes = {centroid, relative, relative};
    newRule->resultIDs = {1, 2, 2, 3, 1, 3, 2, 1, 3, 2, 3, 1};
    newRule->resultTypes = {relative, relative, relative, relative, relative, relative, relative, relative, relative, relative, relative, relative};
    ruleList.push_back(*newRule);
    
    newRule->requirementIDs = {3, 1, 2};
    newRule->requirementTypes = {centroid, relative, relative};
    newRule->resultIDs = {1, 2, 2, 3, 1, 3, 2, 1, 3, 2, 3, 1};
    newRule->resultTypes = {relative, relative, relative, relative, relative, relative, relative, relative, relative, relative, relative, relative};
    ruleList.push_back(*newRule);
    */
    //end of rules----------------------------------------------------------------------
    std::vector<int> synergyTasksAssigned;
    std::vector<int> baselineTasksAssigned;
    std::vector<int> variableThing;
    
    srand(time(NULL));
    for(int z = 0; z < 100; z++){
        int centroidsAssigned = 0;
        int monitorsAssigned = 0;
        int baselineTasks = 0;
        int baselineVehicles;
        int monitorTasks;
        int centroidTasks;
        int maxToAssign;
        constraintNode *toConstruct;
        constraintNode *junk;
        float latitude;
        float longitude;
        int distance;
        int ID;
        std::vector<constraintNode *> monitorOptions;
        std::vector<constraintNode *> centroidOptions;
        int monitorTaskToTry = 0;
        int centroidTaskToTry = 0;
        std::vector<int> monitorTasksTried;
        std::vector<int> centroidTasksTried;
        ID = 0;
        //std::cout << "How many vehicles would you like to be assigned?\n";
        //std::cin >> NUM_UAVS;
        NUM_UAVS = rand() % 3 + 4; //TODO
        baselineVehicles = NUM_UAVS;
        variableThing.push_back(NUM_UAVS); //TODO
        
        //std::cout << "How many monitoring tasks would you like to be generated?\n";
        //std::cin >> monitorTasks;
        monitorTasks = NUM_UAVS * 2; //TODO
        monitorTasksTried.resize(monitorTasks);
        monitorTasksTried.assign(monitorTasks, 0);
        for(int i = 0; i < monitorTasks; i++){
            toConstruct = new constraintNode;
            toConstruct->data = new constraint;
            int IDtoAssign = rand() % NUM_UAVS + 1;
            int IDtoMonitor;
            while((IDtoMonitor = rand() % NUM_UAVS + 1) == IDtoAssign);
            toConstruct->data->groupIDs.push_back(IDtoAssign);
            toConstruct->data->groupIDs.push_back(IDtoMonitor);
            toConstruct->data->monitorIDs.push_back(IDtoMonitor);
            toConstruct->data->type = monitor;
            while(nodeIsPresentInGraph(toConstruct, &junk, monitorOptions)){
                toConstruct->data->groupIDs.clear();
                toConstruct->data->monitorIDs.clear();
                IDtoAssign = rand() % NUM_UAVS + 1;
                while((IDtoMonitor = rand() % NUM_UAVS + 1) == IDtoAssign);
                toConstruct->data->groupIDs.push_back(IDtoAssign);
                toConstruct->data->groupIDs.push_back(IDtoMonitor);
                toConstruct->data->monitorIDs.push_back(IDtoMonitor);
            }
            monitorOptions.push_back(toConstruct);
        }
        //std::cout << "How many centroid tasks would you like to be generated?\n";
        //std::cin >> centroidTasks;
        centroidTasks = NUM_UAVS * 2; //TODO
        centroidTasksTried.resize(centroidTasks);
        centroidTasksTried.assign(centroidTasks, 0);
        
        //std::cout << "What's the maximum number of vehicles to assign to a centroid task?\n";
        //std::cin >> maxToAssign;
        maxToAssign = 2;
        maxToAssign--;
        for(int i = 0; i < centroidTasks; i++){
            toConstruct = new constraintNode;
            toConstruct->data = new constraint;
            int numToAssign = rand() % maxToAssign + 2;
            toConstruct->data->type = centroid;
            for(int j = 0; j < numToAssign; j++){
                int IDtoAssign = rand() % NUM_UAVS + 1;
                while(vectorContainsInt((IDtoAssign = rand() % NUM_UAVS + 1), toConstruct->data->groupIDs));
                toConstruct->data->groupIDs.push_back(IDtoAssign);
            }
            while(nodeIsPresentInGraph(toConstruct, &junk, centroidOptions)){
                toConstruct->data->groupIDs.clear();
                numToAssign = maxToAssign;
                for(int j = 0; j < numToAssign; j++){
                    int IDtoAssign = rand() % NUM_UAVS + 1;
                    while(vectorContainsInt(IDtoAssign, toConstruct->data->groupIDs)){
                        IDtoAssign = rand() % NUM_UAVS + 1;
                    }
                    toConstruct->data->groupIDs.push_back(IDtoAssign);
                }
            }
            centroidOptions.push_back(toConstruct);
        }
        
        int step = 0;
        int baselineCentroid = 0;
        int baselineMonitor = 0;
        std::vector<int> baselineAssignedVehicles;
        bool continueBaseline1 = true;
        bool continueBaseline2 = true;
        while(baselineVehicles > 0 && (continueBaseline1 | continueBaseline2)){
            if(step == 0){
                continueBaseline1 = false;
                step = 1;
                if(baselineCentroid < centroidOptions.size() && baselineVehicles > centroidOptions[baselineCentroid]->data->groupIDs.size()){
                    bool intersection = false;
                    for(int ID : centroidOptions[baselineCentroid]->data->groupIDs){
                        if(vectorContainsInt(ID, baselineAssignedVehicles)){
                            intersection = true;
                        }
                    }
                    if(!intersection){
                        for(int ID : centroidOptions[baselineCentroid]->data->groupIDs){
                            baselineAssignedVehicles.push_back(ID);
                        }
                        baselineVehicles -= centroidOptions[baselineCentroid++]->data->groupIDs.size();
                        baselineTasks++;
                        continueBaseline1 = true;
                    }
                }
            }
            else{
                continueBaseline2 = false;
                step = 0;
                if(baselineMonitor < monitorOptions.size() && baselineVehicles > 0 && !vectorContainsInt(monitorOptions[baselineMonitor++]->data->groupIDs[0], baselineAssignedVehicles)){
                    baselineAssignedVehicles.push_back(monitorOptions[baselineMonitor++]->data->groupIDs[0]);
                    baselineVehicles--;
                    baselineTasks++;
                    continueBaseline2 = true;
                }
            }
        }
        
        std::vector<constraintNode *> constraintGraph;
        bool continueLoopCentroid = true;
        bool continueLoopMonitor = true;
        bool continueLoop = continueLoopCentroid | continueLoopMonitor;
        //std::cout << "Beginning processing...\n";
        while(continueLoop){
            if(continueLoopCentroid){
                //insert a new centroid task to the graph
                constraintNode *nodeToAdd = new constraintNode;
                nodeToAdd->data = centroidOptions[centroidTaskToTry++]->data;
                constraintGraph.push_back(nodeToAdd);
                if(centroidTaskToTry == centroidOptions.size()){
                    continueLoopCentroid = false;
                }
                if(checkCompatibility(constraintGraph)){
                    centroidsAssigned++;
                    std::cout << "ID size: " << nodeToAdd->data->groupIDs.size() << std::endl;
                    std::cout << "monitoring ID size: " << monitoringIDs.size() << std::endl;
                    if(vectorContainsInt(nodeToAdd->data->groupIDs[0], monitoringIDs) &&
                       !vectorContainsInt(nodeToAdd->data->groupIDs[1], monitoringIDs)){
                        monitoringIDs.push_back(nodeToAdd->data->groupIDs[1]);
                    }
                    else if(vectorContainsInt(nodeToAdd->data->groupIDs[1], monitoringIDs) &&
                       !vectorContainsInt(nodeToAdd->data->groupIDs[0], monitoringIDs)){
                        monitoringIDs.push_back(nodeToAdd->data->groupIDs[0]);
                    }
                    else{
                        centroidsAssigned--;
                        constraintGraph.pop_back();
                        centroidTasksTried[centroidTaskToTry] = 1;
                        if(!vectorContainsInt(0, centroidTasksTried)){
                            continueLoopCentroid = false;
                        }
                    }
                }
                else{
                    constraintGraph.pop_back();
                    centroidTasksTried[centroidTaskToTry] = 1;
                    if(!vectorContainsInt(0, centroidTasksTried)){
                        continueLoopCentroid = false;
                    }
                }
            }
            if(continueLoopMonitor){
                //insert a new monitor task to the graph
                constraintNode *nodeToAdd = new constraintNode;
                nodeToAdd->data = monitorOptions[monitorTaskToTry++]->data;
                constraintGraph.push_back(nodeToAdd);
                if(monitorTaskToTry == monitorOptions.size()){
                    continueLoopMonitor = false;
                }
                if(!vectorContainsInt(nodeToAdd->data->groupIDs[0], monitoringIDs) && checkCompatibility(constraintGraph)){
                    monitorsAssigned++;
                    monitoringIDs.push_back(nodeToAdd->data->groupIDs[0]);
                    if(!vectorContainsInt(nodeToAdd->data->groupIDs[0], vehicleIDs)){
                        vehicleIDs.push_back(nodeToAdd->data->groupIDs[0]);
                    }
                }
                else{
                    constraintGraph.pop_back();
                    monitorTasksTried[monitorTaskToTry] = 1;
                    if(!vectorContainsInt(0, monitorTasksTried)){
                        continueLoopMonitor = false;
                    }
                }
            }
            continueLoop = continueLoopCentroid | continueLoopMonitor;
        }
        
        for(int i = 0; i < constraintGraph.size(); i++){
            for(int j = 0; j < constraintGraph.size(); j++){
                if(constraintsEqual(*constraintGraph[i]->data, *constraintGraph[j]->data,
                                   (constraintGraph[i]->data->type != centroid)) 
                   && i != j){
                    constraintGraph.erase(constraintGraph.begin() + j);
                    i--;
                    j = constraintGraph.size();
                }
            }
        }
        //std::cout << "Task types assigned: " << centroidsAssigned << " | " << monitorsAssigned << std::endl;
        //std::cout << "Tasks assigned: " << constraintGraph.size() << std::endl;
        synergyTasksAssigned.push_back(constraintGraph.size());
        baselineTasksAssigned.push_back(baselineTasks);
        if(constraintGraph.size() > 100){
        for(constraintNode *currNode : constraintGraph){
            std::cout << currNode->data->type;
            for(int ID : currNode->data->groupIDs){
                std::cout << " " << ID;
            }
            std::cout << std::endl;
        }
        std::cout << "---------------------------------------" << std::endl;
        }
        constraintGraph.resize(0);
        monitoringIDs.resize(0);
        centroidOptions.resize(0);
        monitorOptions.resize(0);
    }
    std::cout << "[";
    for(int z = 0; z < 100; z++){
        std::cout << baselineTasksAssigned[z] << ", ";
    }
    std::cout << "]\n[";
    for(int z = 0; z < 100; z++){
        std::cout << synergyTasksAssigned[z] << ", ";
    }
    std::cout << "]\n[";
    for(int z = 0; z < 100; z++){
        std::cout << variableThing[z] << ", ";
    }
    return (true);
};



// Listener for ICAROUS command messages
void IcarousCommunicationService::ICAROUS_listener(int id)
{
    
}



// This function is performed to cleanly terminate the service
bool IcarousCommunicationService::terminate()
{
    // perform any action required during service termination, before destructor is called.
    std::cout << "*** TERMINATING:: Service[" << s_typeName() << "] Service Id[" << m_serviceId << "] with working directory [" << m_workDirectoryName << "] *** " << std::endl;
    
    return (true);
}

// Listen for defined messages and relay them to the needed ICAROUS instance they belong to
bool IcarousCommunicationService::processReceivedLmcpMessage(std::unique_ptr<uxas::communications::data::LmcpMessage> receivedLmcpMessage)
{
    /*
    // Template for added new message parsing
    if(<namespace>::<namespace>::is<type>(receivedLmcpMessage->m_object))
    {
        auto ptr_<type> = std::shared_ptr<<namespace>::<namespace>::<type>((<namespace>::<namespace>::<type>*)receivedLmcpMessage->m_object->clone());
        // Parsing code
        ptr_<type>->getInformation();
        
        // Sending ICAROUS a Dummy Command message
        dprintf(client_sockfd[vehicleID - 1], "COMND,type%s,\n",
            "Dummy Command");
    }// End of Template
    else
    */
    
    // Parse the AirVehicleConfiguration for the UAVs nominal speeds
    if(afrl::cmasi::isAirVehicleConfiguration(receivedLmcpMessage->m_object))
    {
        auto ptr_AirVehicleConfiguration = std::shared_ptr<afrl::cmasi::AirVehicleConfiguration>((afrl::cmasi::AirVehicleConfiguration*)receivedLmcpMessage->m_object->clone());
        auto vehicleID = ptr_AirVehicleConfiguration->getID();
        
    }
    
    // Process an AirVehicleState from OpenAMASE
    else if(afrl::cmasi::isAirVehicleState(receivedLmcpMessage->m_object))
    {
        // Copy the message pointer to shorten access length
        auto ptr_AirVehicleState = std::shared_ptr<afrl::cmasi::AirVehicleState>((afrl::cmasi::AirVehicleState *)receivedLmcpMessage->m_object->clone());
        int vehicleID = ptr_AirVehicleState->getID();
        
        vehicleStates[vehicleID - 1] = ptr_AirVehicleState;
        
        //for this UAV, update your own info and your spot in the hasUpdated array saying that you've done so
        hasUpdated[vehicleID - 1] = true;
        
        //check if this is the last UAV to update in this timestep
        bool allUpdated = true;
        for(int i = 0; i < NUM_UAVS + NUM_MONITOR; i++){
            if(!hasUpdated[i]){
                allUpdated = false;
                break;
            }
        }
        
        if(monitoringTaskActiveGlobal && allUpdated){
            //reset hasUpdated to all false
            hasUpdated.assign(NUM_UAVS + NUM_MONITOR, false);
            
            //foreach UAV on a monitoring task, find or get their new velocity
            for(int currentVehicleID : monitoringIDs){
                auto currentVehicleState = vehicleStates[currentVehicleID - 1];
                
                auto mc = std::shared_ptr<afrl::cmasi::MissionCommand>(new afrl::cmasi::MissionCommand);
                auto la = new afrl::cmasi::LoiterAction();
                auto newLocation = new afrl::cmasi::Waypoint();
                
                double aveX = 0;
                double aveY = 0;
                std::vector<constraint> relevantConstraints;
                std::vector<constraint> relevantCentroidConstraints;
                
                int numTracked = 0;
                for(constraint currConstraint : constraints){
                    if(currConstraint.type == monitor && currConstraint.groupIDs[0] == currentVehicleID){
                        relevantConstraints.push_back(currConstraint);
                        for(int ID : currConstraint.monitorIDs){
                            aveX += vehicleStates[ID - 1]->getLocation()->getLongitude();
                            aveY += vehicleStates[ID - 1]->getLocation()->getLatitude();
                            numTracked++;
                        }
                    }
                    else if(currConstraint.type == centroid){
                        for(int ID : currConstraint.groupIDs){
                            if(ID == currentVehicleID){
                                relevantCentroidConstraints.push_back(currConstraint);
                                break;
                            }
                        }
                    }
                }
                
                aveX /= (double)numTracked;
                aveY /= (double)numTracked;
                
                if(numTracked == 1){
                    //We'll be able to pick a point at the given distance from the target.
                    //This isn't possible at >1 because we'll have a single viable point if we
                    //want to be at the given distance from all targets. TODO find intersection
                    //of arbitrary numbers of circles for the case of >1 target
                    
                    //average the locations of all centroid constraints
                    int numCentroids = 0;
                    float centroidAveX = 0.;
                    float centroidAveY = 0.;
                    for(constraint currConstraint : relevantCentroidConstraints){
                        centroidAveX += currConstraint.centroidX;
                        centroidAveY += currConstraint.centroidY;
                        numCentroids++;
                    }
                    
                    if(numCentroids == 0){
                        centroidAveX = vehicleStates[currentVehicleID - 1]->getLocation()->getLongitude();
                        centroidAveY = vehicleStates[currentVehicleID - 1]->getLocation()->getLatitude();
                    }
                    
                    //We use a parametrization of the line segment between the average of the
                    //centroids and the location of the monitoring target to find the point
                    //to monitor from. The point (a, b) is the monitored target, and the point
                    //(c, d) is the average of the centroids constraining this vehicle.
                    float a, b, c, d;
                    a = aveX;
                    b = aveY;
                    c = centroidAveX;
                    d = centroidAveY;
                    float lineLength = sqrt(((a - c) * (a - c)) + ((b - d) * (b - d)));
                    float monitorDistance = relevantConstraints[0].monitorDistances[0];
                    monitorDistance /= 111111.;
                    
                    float t = monitorDistance / lineLength;
                    //solve equations above for x and y
                    aveX = (1 - t) * a + (t * c);
                    aveY = (1 - t) * b + (t * d);
                }
                
                auto loc = new afrl::cmasi::Location3D();
                loc->setLongitude(aveX);
                loc->setLatitude(aveY);
                loc->setAltitude(vehicleStates[currentVehicleID - 1]->getLocation()->getAltitude());
                
                la->setLocation(loc);
                la->setDuration(-1);
                
                std::vector<afrl::cmasi::VehicleAction *> actionList;
                actionList.push_back(la);
                
                newLocation->setLatitude(loc->getLatitude());
                newLocation->setLongitude(loc->getLongitude());
                newLocation->setAltitude(loc->getAltitude());
                newLocation->getVehicleActionList().push_back(la);
                newLocation->setNextWaypoint(newLocation->getNumber());
                
                std::vector<afrl::cmasi::Waypoint *> waypointList;
                waypointList.push_back(newLocation);
                
                mc->getWaypointList().push_back(newLocation);
                mc->setCommandID(currentVehicleID);
                mc->setVehicleID(currentVehicleID);
                mc->setStatus(afrl::cmasi::CommandStatusType::Approved);
                
                sendSharedLmcpObjectBroadcastMessage(mc);
                
                adjustedIDs.push_back(currentVehicleID);
            }
            
            //foreach UAV not on a monitoring task, from most constraints to least (TODO), adjust their velocity to fit
            for(int currentVehicleID : idleIDs){
                if(!isAdjustedThisIteration(currentVehicleID)){
                    auto currentVehicleState = vehicleStates[currentVehicleID - 1];
                    
                    //find which constraints are on this vehicle and which vehicles are in a group with this one
                    std::vector<constraint> relevantConstraints;
                    std::vector<std::vector<int>> relevantVehicleIDs;
                    int constraintCount = -1;
                    for(int i = 0; i < constraints.size(); i++){
                        bool isFound = false;
                        for(int j = 0; j < constraints[i].groupIDs.size() && !isFound; j++){
                            if(constraints[i].groupIDs[j] == currentVehicleID){
                                isFound = true;
                                constraintCount++;
                                relevantVehicleIDs.resize(relevantVehicleIDs.size() + 1);
                                relevantVehicleIDs[constraintCount].resize(constraints[i].groupIDs.size());
                                for(int k = 0; k < constraints[i].groupIDs.size(); k++){
                                    relevantVehicleIDs[constraintCount][k] = constraints[i].groupIDs[k];
                                }
                                break;
                            }
                        }
                        if(isFound){
                            relevantConstraints.push_back(constraints[i]);
                        }
                    }
                    
                    //extrapolate new position for each monitoring UAV relevant to this UAV
                    std::vector<std::vector<float>> projectedVehicleLocations;
                    projectedVehicleLocations.resize(vehicleIDs.size());
                    for(int i = 0; i < projectedVehicleLocations.size(); i++){
                        projectedVehicleLocations[i].resize(2); //long and lat
                    }
                    
                    
                    for(int vehID : vehicleIDs){
                        //figure out where it'll be in a half second
                        printf("vehID: %d\n", vehID);
                        long double uHeading = fmod((vehicleStates[vehID - 1]->getHeading() + 360), 360.0);
                        long double vHeading = fmod((uHeading + 90), 360.0);
                        long double uNorth;
                        long double vNorth;
                        long double uEast;
                        long double vEast;
                        
                        uNorth = vehicleStates[vehID - 1]->getU() * cos(uHeading*M_PI/180);
                        uEast = vehicleStates[vehID - 1]->getU() * sin(uHeading*M_PI/180);
                        
                        vNorth = vehicleStates[vehID - 1]->getV() * cos(vHeading*M_PI/180);
                        vEast = vehicleStates[vehID - 1]->getV() * sin(vHeading*M_PI/180);
                        
                        long double northTotal = uNorth + vNorth;
                        long double eastTotal = uEast + vEast;
                        
                        auto loc = vehicleStates[vehID - 1]->getLocation();
                        double longitude = loc->getLongitude();
                        double latitude = loc->getLatitude();
                        
                        //0.5 is for half a second, which is roughly equal to AMASE's tick rate
                        longitude += (eastTotal * cos(latitude) * 0.5) / 111111.;
                        latitude += (northTotal * 0.5) / 111111.;
                        
                        projectedVehicleLocations[vehID - 1][0] = longitude;
                        projectedVehicleLocations[vehID - 1][1] = latitude;
                        
                        printf("long: %10f\t projected: %10f\n", vehicleStates[vehID - 1]->getLocation()->getLongitude(), projectedVehicleLocations[vehID - 1][0]);
                        printf("lat:  %10f\t projected: %10f\n", vehicleStates[vehID - 1]->getLocation()->getLatitude(), projectedVehicleLocations[vehID - 1][1]);
                        //Now we have the lat/long for where this UAV will be in half a second if
                        //it doesn't change course
                    }
                    
                    double remLatError = 0.;
                    double remLongError = 0.;
                    std::vector<int> numIdleVehicles;
                    numIdleVehicles.resize(relevantConstraints.size() + 1);
                    
                    //figure out where each idle UAV in each group needs to go
                    for(int i = 0; i < relevantConstraints.size(); i++){
                        printf("\nconstraint number: %d\n", i);
                        int numVeh = 0;
                        int numIdleVeh = 0;
                        double longError = 0.;
                        double latError = 0.;
                        for(int calcID : relevantVehicleIDs[i]){
                            longError += projectedVehicleLocations[calcID - 1][0] - relevantConstraints[i].centroidX;
                            latError += projectedVehicleLocations[calcID - 1][1] - relevantConstraints[i].centroidY;
                            if(!vectorContainsInt(calcID, monitoringIDs) && !isAdjustedThisIteration(calcID)){
                                numIdleVeh++;
                            }
                            numVeh++;
                        }
                        
                        double aveLongError = longError;
                        double aveLatError = latError;
                        
                        printf("longError: %f\n", aveLongError);
                        printf("latError: %f\n\n", aveLatError);
                        
                        remLongError += longError;
                        remLatError += latError;
                        numIdleVehicles[i] = numIdleVeh;
                        //we now have the amount to adjust projected position by
                        
                    }
                    
                    if(!vectorContainsInt(currentVehicleID, monitoringIDs)){
                        adjustedIDs.push_back(currentVehicleID);
                        
                        //assign the new velocities
                        auto mc = std::shared_ptr<afrl::cmasi::MissionCommand>(new afrl::cmasi::MissionCommand);
                        auto la = new afrl::cmasi::LoiterAction();
                        auto newLocation = new afrl::cmasi::Waypoint();
                        
                        auto loc = new afrl::cmasi::Location3D();
                        
                        double longErrorToTake = 0.;
                        double latErrorToTake = 0.;
                        
                        for(int i = 0; i < relevantConstraints.size(); i++){
                            int numIdleVeh = numIdleVehicles[i];
                            int numVeh = relevantConstraints[i].groupIDs.size();
                            int totalNumIdleVeh = 0;
                            for(int j = 0; j < numIdleVehicles.size(); j++){
                                totalNumIdleVeh += numIdleVehicles[i];
                            }
                            longErrorToTake = remLongError * ((double)numIdleVeh / (double)totalNumIdleVeh);
                            remLongError -= longErrorToTake;
                            latErrorToTake = remLatError * ((double)numIdleVeh / (double)totalNumIdleVeh);
                            remLatError -= latErrorToTake;
                        }
                        
                        loc->setLongitude(projectedVehicleLocations[currentVehicleID - 1][0] - longErrorToTake);
                        loc->setLatitude(projectedVehicleLocations[currentVehicleID - 1][1] - latErrorToTake);
                        loc->setAltitude(vehicleStates[currentVehicleID - 1]->getLocation()->getAltitude());
                        
                        projectedVehicleLocations[currentVehicleID - 1][0] -= longErrorToTake;
                        projectedVehicleLocations[currentVehicleID - 1][1] -= latErrorToTake;
                        
                        la->setLocation(loc);
                        la->setDuration(-1);
                        
                        std::vector<afrl::cmasi::VehicleAction *> actionList;
                        actionList.push_back(la);
                        
                        newLocation->setLatitude(loc->getLatitude());
                        newLocation->setLongitude(loc->getLongitude());
                        newLocation->setAltitude(loc->getAltitude());
                        newLocation->getVehicleActionList().push_back(la);
                        newLocation->setNextWaypoint(newLocation->getNumber());
                        
                        std::vector<afrl::cmasi::Waypoint *> waypointList;
                        waypointList.push_back(newLocation);
                        
                        mc->getWaypointList().push_back(newLocation);
                        mc->setCommandID(currentVehicleID);
                        mc->setVehicleID(currentVehicleID);
                        mc->setStatus(afrl::cmasi::CommandStatusType::Approved);
                        
                        sendSharedLmcpObjectBroadcastMessage(mc);
                    }
                }
                if(currentVehicleID == idleIDs.back()){
                    isAdjustedThisIteration(-1); //clear
                }
            }
        }
        else{
            //no need to replan; continue with the previous velocities
        }
    }// End of AirVehicleState



    // False indicates that we are ready to process more messages
    return false;
}

}; //namespace service
}; //namespace uxas
