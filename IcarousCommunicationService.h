// ===============================================================================
// Authors: AFRL/RQQA & NASA/NIA
// Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
// 
// Copyright (c) 2017 Government of the United State of America, as represented by
// the Secretary of the Air Force.  No copyright is claimed in the United States under
// Title 17, U.S. Code.  All Other Rights Reserved.
// ===============================================================================

/* 
 * File:   IcarousCommunicationService.h
 * Authors: Winston Smith & Paul Coen
 *
 * Created on June 14, 2018, 3:55 PM
 * This file allows connectivity with the CRATOUS system
 * (CRoss Application Translator of Operational Unmanned Systems) 
 * CRATOUS allows cooperative mission planning between UxAS and ICAROUS
 * It also allows for UxAS to use ICAROUS route planning algorithms
 *
 */

#ifndef UXAS_ICAROUSCOMMUNICATIONSERVICE_H
#define UXAS_ICAROUSCOMMUNICATIONSERVICE_H



#include "ServiceBase.h"
#include "CallbackTimer.h"
#include "TypeDefs/UxAS_TypeDefs_Timer.h"

#include "ServiceBase.h"
#include "TypeDefs/UxAS_TypeDefs_String.h"
#include "CallbackTimer.h"

#include "uxas/messages/route/RoutePlan.h"
#include "uxas/messages/route/RoutePlanRequest.h"
#include "uxas/messages/route/RoutePlanResponse.h"

#include "afrl/cmasi/Waypoint.h"
#include "afrl/cmasi/TurnType.h"
#include "afrl/cmasi/MissionCommand.h"
#include "afrl/cmasi/KeepInZone.h"
#include "afrl/cmasi/KeepOutZone.h"
#include "afrl/cmasi/AirVehicleState.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <string.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory>
#include <errno.h>
#include <cmath>
#include <math.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <semaphore.h>

#define PORT 5557
#define STRING_XML_ICAROUS_CONNECTIONS "NumberOfUAVs"
#define STRING_XML_ICAROUS_ROUTEPLANNER "RoutePlannerUsed"
#define STRING_XML_LINE_VOLUME "DeviationAllowed"
#define STRING_XML_ICAROUS_DEVIATION_ORIGIN "DeviationOrigin"
#define M_PI 3.14159265358979323846

namespace uxas
{
namespace service
{

/*! \class IcarousCommunicationService
 *  \brief This service handles communication with ICAROUS for integration of the two pieces of software.
 *  
 * </ul> @n
 * 
 * Configuration String: <Service Type="IcarousCommunicationService" NumberOfUAVs="n" />
 * 
 * Options:
 *  - NumberOfUAVs - Used to specify the number of UAVs in a scenario
 *  - RoutePlannerUsed="n" - Inform this service what planner to use
 *                      -1 - UxAS Visibility planner
 *                      0 - GRID
 *                      1 - ASTAR
 *                      2 - RRT
 *                      3 - SPLINE
 *  - DeviationOrigin - origin point for deviations
 *                      line - the line that is being searched
 *                      path - the path the UAV is taking
 * 
 * Subscribed Messages:
 *  - afrl::cmasi::MissionCommand
 *  - afrl::cmasi::KeepInZone
 *  - afrl::cmasi::KeepOutZone
 *  - afrl::cmasi::AirVehicleState
 *  - afrl::cmasi::AirVehicleConfiguration
 *  - uxas::common::MessageGroup::IcarousPathPlanner
 *  - uxas::messages::route::RoutePlanRequest
 * 
 * Sent Messages:
 *  - afrl::cmasi::MissionCommand
 *  - afrl::cmasi::VehicleActionCommand
 *  - uxas::messages::route::RoutePlanResponse
 *  - uxas::messages::task::TaskPause
 *  - uxas::messages::task::TaskResume
 *
 */

class IcarousCommunicationService : public ServiceBase
{
public:
    /** \brief This string is used to identify this service in XML configuration
     * files, i.e. Service Type="IcarousCommunicationService". It is also entered into
     * service registry and used to create new instances of this service. */
    static const std::string&
    s_typeName()
    {
        static std::string s_string("IcarousCommunicationService");
        return (s_string);
    };

    static const std::vector<std::string>
    s_registryServiceTypeNames()
    {
        std::vector<std::string> registryServiceTypeNames = {s_typeName()};
        return (registryServiceTypeNames);
    };

    /** \brief If this string is not empty, it is used to create a data 
     * directory to be used by the service. The path to this directory is
     * accessed through the ServiceBase variable m_workDirectoryPath. */
    static const std::string&
    s_directoryName() { static std::string s_string(""); return (s_string); };

    static ServiceBase*
    create()
    {
        return new IcarousCommunicationService;
    };

    IcarousCommunicationService();

    /** brief Listen to ICAROUS clients for commands*/
    void ICAROUS_listener(int id);

    virtual
    ~IcarousCommunicationService();

private:

    static
    ServiceBase::CreationRegistrar<IcarousCommunicationService> s_registrar;

    /** brief Copy construction not permitted */
    IcarousCommunicationService(IcarousCommunicationService const&) = delete;

    /** brief Copy assignment operation not permitted */
    void operator=(IcarousCommunicationService const&) = delete;

    bool
    configure(const pugi::xml_node& serviceXmlNode) override;

    bool
    initialize() override;

    bool
    start() override;

    bool
    terminate() override;
    
    bool
    vectorContainsInt(int needle, std::vector<int> haystack);
    
    bool
    isAdjustedThisIteration(int IDtoCheck);
    
    bool 
    vectorIntsEqual(std::vector<int> left, std::vector<int> right, bool orderMatters);

    bool
    processReceivedLmcpMessage(std::unique_ptr<uxas::communications::data::LmcpMessage> receivedLmcpMessage) override;

private:
    
    //Whether or not a UAV has updated in this timestep
    std::vector<bool> hasUpdated;
    
    // Number of unique controlled UAVs in the scenario
    int32_t NUM_UAVS{3};
    // Number of UAVs that are being monitored but aren't controlled
    int32_t NUM_MONITOR{1};
    
    // Holds state information for all vehicles
    std::vector<std::shared_ptr<afrl::cmasi::AirVehicleState>> vehicleStates;
    
    //Holds information about a single constraint
    enum constraintTypes{centroid, monitor, global, relative, invalid};
    typedef struct constraint{
        constraintTypes type;
        float centroidX;
        float centroidY;
        std::vector<int> groupIDs;
        std::vector<int> monitorIDs;
        std::vector<int> monitorDistances;
    }constraint;
    
    //This struct holds inference rules (requirements and resultant information)
    typedef struct inferenceRule{
        std::vector<int> requirementIDs;
        std::vector<constraintTypes> requirementTypes;
        std::vector<int> resultIDs;
        std::vector<constraintTypes> resultTypes;
    }inferenceRule;
    
    typedef struct constraintNode{
        constraint *data;
        std::vector<struct constraintNode *> parents;
        std::vector<struct constraintNode *> children;
    }constraintNode;
    
    bool
    checkCompatibility(std::vector<constraintNode *> constraintGraph);
    
    bool
    ruleApplies(inferenceRule *ruleToCheck, std::vector<constraintNode *> constraintGraph);
    
    bool 
    vectorContainsOnlyConstraintTypes(constraintTypes hay, std::vector<constraintTypes> haystack);
    
    constraintTypes
    findNextConstraintTypeWithPosition(std::vector<constraintTypes> haystack, int *position);
    
    std::vector<int> *
    sliceRequirementIDs(int startIndex, std::vector<constraintTypes> haystack, std::vector<int> vectorToSlice);
    
    bool
    constraintsEqual(constraint left, constraint right, bool orderMatters);
    
    bool
    nodeIsPresentInGraph(constraintNode *nodeToAdd, constraintNode **otherNode, 
                         std::vector<constraintNode *> constraintGraph);
                         
    void gatherDescendents(constraintNode *node, std::vector<constraintNode *> *descendentsFound);
    
    bool descendentsAreSuperset(constraintNode *nodeToAdd, constraintNode *otherNode);
    
    bool nodeCombosEqual(std::vector<constraintNode *> comboToAdd, std::vector<constraintNode *> otherCombo);
    
    std::vector<std::vector<constraintNode *>> nodeCombosThisIteration;
    std::vector<inferenceRule> rulesAppliedThisIteration;
    std::vector<inferenceRule> ruleList;
    
    std::vector<int> monitoringIDs;
    std::vector<int> idleIDs;
    std::vector<int> vehicleIDs;
    bool monitoringTaskActiveGlobal{false};
    
    //Holds constraint groups for UAVs
    std::vector<constraint> constraints;
    bool constraintsInitialized{false};
    
    std::vector<int> adjustedIDs;
};

}; //namespace service
}; //namespace uxas

#endif /* UXAS_ICAROUSCOMMUNICATIONSERVICE_H */

