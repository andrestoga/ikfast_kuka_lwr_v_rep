JointHandle={-1,-1,-1,-1,-1,-1,-1}
    JointHandle[1]=simGetObjectHandle("LBR4p_joint1")
    JointHandle[2]=simGetObjectHandle("LBR4p_joint2")
    JointHandle[3]=simGetObjectHandle("LBR4p_joint3")
    JointHandle[4]=simGetObjectHandle("LBR4p_joint4")
    JointHandle[5]=simGetObjectHandle("LBR4p_joint5")
    JointHandle[6]=simGetObjectHandle("LBR4p_joint6")
    JointHandle[7]=simGetObjectHandle("LBR4p_joint7")
    Base=simGetObjectHandle("Base")
    Target=simGetObjectHandle("Target")

    local min = -3.14
    local max = 3.14

    local joints = {}

    -- local num_of_tests = 10
    local num_of_joints = 7

    local M={}

    path = "Pose_Target.txt"

    math.randomseed(os.time())
    math.random()
    math.random()
    math.random()
        
    file = io.open(path,"w")

    for j = 1, num_of_joints do
        --joints[j] = min + math.random() * (max - min)
        joints[j] = math.random(-0.5*math.pi,0.5*math.pi)
        if j == 4 then
            file:write(-joints[j]," ")
        else
            file:write(joints[j]," ")
        end
    end

    file:close()

    simExtIWRFK(path)

    file = io.open(path,"r")

    simSetObjectPosition( Target,-1,{file:read("*number"),file:read("*number"),file:read("*number")} )

    M = simGetObjectMatrix(Target, Base)

    file:close()

    file = io.open(path,"w")

    for j = 1, 3 do
        file:write(M[j*4]," ")
    end

    file:write("\n")

    for i = 1, 3 do
        for j = 1, 3 do
            file:write(M[(i-1)*4+j]," ")
        end
        file:write("\n")
    end

    file:close()

    simExtIWRIKFast(path)
    
    file=io.open(path,"r")

    for i=1,7 do

        local number = file:read("*number")

        if number == -100 then
            break
        end

        if i==4 then
            simSetJointTargetPosition(JointHandle[i], -number)
        else
            simSetJointTargetPosition(JointHandle[i], number)
        end
    end

    file:close()
