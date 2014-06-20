// Copyright 2009-2014 Sandia Corporation. Under the terms
// of Contract DE-AC04-94AL85000 with Sandia Corporation, the U.S.
// Government retains certain rights in this software.
// 
// Copyright (c) 2009-2014, Sandia Corporation
// All rights reserved.
// 
// This file is part of the SST software package. For license
// information, see the LICENSE file in the top level directory of the
// distribution.

#include "TaskMapInfo.h"

#include "Job.h"
#include "MeshAllocInfo.h"
#include "output.h"
#include "TaskCommInfo.h"

using namespace SST::Scheduler;

TaskMapInfo::TaskMapInfo(AllocInfo* ai)
{
    allocInfo = ai;
    job = ai -> job;
    taskCommInfo = ai->job->taskCommInfo;
}

void TaskMapInfo::insert(int taskInd, int nodeInd)
{
    //check if taskInd is valid
    //check if exists
    if(taskInd >= job->getProcsNeeded() || taskInd < 0) {
    	schedout.fatal(CALL_INFO, 1, "in TaskMapInfo: Could not map inexistent task %d of job %ld to node %d\n", taskInd, job->getJobNum(), nodeInd);
    }
    //check if already mapped
    if(taskMap.left.count(taskInd) != 0) {
    	schedout.fatal(CALL_INFO, 1, "in TaskMapInfo: Task %d of job %ld is already mapped to node %d\n", taskInd, job->getJobNum(), taskMap.left.at(taskInd));
    }

    //check if nodeInd is valid
    //check if exists
    bool found = false;
    for(int i = 0; i < job->getProcsNeeded() ; i++){
        if(allocInfo->nodeIndices[i] == nodeInd){
            found = true;
        }
    }
    if(!found){
    	schedout.fatal(CALL_INFO, 1, "Could not map task %d of job %ld is to the unallocated node %d\n", taskInd, job->getJobNum(), nodeInd);
    }
    //check if already mapped
    if(taskMap.right.count(nodeInd) != 0) {
    	schedout.fatal(CALL_INFO, 1, "Node %d is already allocated by task %d of job %ld", nodeInd, taskMap.right.at(nodeInd), job->getJobNum());
    }

    //add mapping
    taskMap.insert(taskMapType::value_type(taskInd, nodeInd));
}

taskMapType TaskMapInfo::getTaskMap()
{
    //check if every task is mapped
    if((unsigned int) job->getProcsNeeded() != taskMap.size()){
    	schedout.fatal(CALL_INFO, 1, "Task mapping info requested before all tasks are mapped.");
    }

    return taskMap;
}

//Current version only checks if there is communication
unsigned long TaskMapInfo::getTotalHopDist(Machine* machine)
{
    unsigned long totalDist = 0;

    //check if every task is mapped
    if((unsigned int) job->getProcsNeeded() != taskMap.size()){
    	schedout.fatal(CALL_INFO, 1, "Task mapping info requested before all tasks are mapped.");
    }

    //iterate through all tasks
    MeshLocation* curLoc;
    for(int taskIter = 0; taskIter < (job->getProcsNeeded() - 1); taskIter++){
        int currentNode = taskMap.left.at(taskIter);
        curLoc = new MeshLocation(currentNode, (MachineMesh*) machine);
        //iterate through other tasks and add distance for communication
        for(int otherIter = (taskIter + 1) ; otherIter < job->getProcsNeeded(); otherIter++){
            if( (taskCommInfo->commMatrix[taskIter][otherIter] != 0) || (taskCommInfo->commMatrix[otherIter][taskIter] != 0) ){
                totalDist += curLoc->L1DistanceTo(new MeshLocation(otherIter, (MachineMesh*) machine));
            }
        }
    }
    totalDist = totalDist / 2; //remove duplicates

    return totalDist;
}