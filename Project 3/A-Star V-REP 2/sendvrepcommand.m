function [i] = sendvrepcommand(l_tV,r_tV,i,vrep,cID,l_wmh,r_wmh)

    i = i + 1;
    vrep.simxSetJointTargetVelocity(cID,l_wmh,l_tV(i),vrep.simx_opmode_blocking);
    vrep.simxSetJointTargetVelocity(cID,r_wmh,r_tV(i),vrep.simx_opmode_blocking);
    
end