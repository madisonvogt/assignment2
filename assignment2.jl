# assignment2.jl
# Author: Madison Vogt
# University of Kentucky
# ME 676, Dr. Poonawala
# March 27, 2023

include("startup.jl")
using LinearAlgebra
using RigidBodyDynamics
using RigidBodySim

# Define Variables
q0 = [0.01;-0.5;-0.0;-2.0;-0.3;1.5;-0.7;0.1;0.1] 
qd = [0.0;0.0;0.0;0.0;0.0;pi;0.01;0.01;0.01]

function update_planar_state(state,mvis,q)
    set_configuration!(state, q)
    set_configuration!(mvis, configuration(state))
end

urdf = joinpath(dirname(pathof(RigidBodySim)), "..", "test", "urdf", "panda.urdf")

mvis, mechanism = display_urdf("panda.urdf",vis)
state = MechanismState(mechanism)
update_planar_state(state,mvis,q0)

set_configuration!(state,q0)
zero_velocity!(state)

# Update mechanism visual
set_configuration!(mvis, configuration(state))

# START OF PART 1 

#Initialize q
q = configuration(state)

function traj(t)
    # Compute the desired joint angle at time t
    qt = q0 + (1/10)*(qd-q0)*t    # Desired joint angle designed to linearly track at 10 seconds
    vt = (1/10)*(qd-q0)           # Derivative of qt
    at = 0                        # Derivative of vt
    return qt, vt, at
end
qt,vt,at=traj(10)
println("Tracking Final Configuration =", qt) 
println("Tracking Final Joint Velocities =", vt)
println("Tracking Final Joint Accelerations =", at)

#COMPLETE PART 1, START OF PART 2

function control_PD!(τ, t, state)
    # Compute a value for τ
    qt,vt,at = traj(t)
    τ .= -50 .* (velocity(state) - vt) - 500*(configuration(state) - qt)    # Implement Kd and Kp
    # Saturate
    act_sat = 50; # Actuator limits
    τ .= map( x -> x > act_sat ? act_sat : x,τ)
    τ .= map( x -> x < -act_sat ? -act_sat : x,τ)
end

#Solve using the function above and built in ode solver

problemPD = ODEProblem(Dynamics(mechanism,control_PD!), state, (0.,10.))
solPD = solve(problemPD, Vern7())

#Animate Solution

setanimation!(mvis, solPD; realtime_rate = 1.0);

# Print results, showing that the final configuration error norm is less than .01 and giving the final joint angles

println("PD Final Joint Angle:" ,solPD[end][1:9])
println("PD Final Error Norm: ",norm(solPD[end][1:9]-qd))


#COMPLETE PART 2, START OF PART 3


function control_CTC!(τ, t, state)
    # Compute a value for τ
    aq=similar(velocity(state))                                             # Get Segmented Vector for aq

    qt,vt,at = traj(t)

    # Add PD controller, using CTC the Kp can be as low as 5 - 100 times smaller than the PD controller, when the same Kd values are used
    aq .= -50 .* (velocity(state) - vt) - 5*(configuration(state) - qt)     

    τ .= inverse_dynamics(state, aq)

    # Saturate
    act_sat = 50; # Actuator limits
    τ .= map( x -> x > act_sat ? act_sat : x,τ)
    τ .= map( x -> x < -act_sat ? -act_sat : x,τ)
end


problemCTC = ODEProblem(Dynamics(mechanism,control_CTC!), state, (0.,10.))
solCTC = solve(problemCTC, Vern7())

# Animate Solution

setanimation!(mvis, solCTC; realtime_rate = 1.0);

# Print results, showing that the final configuration error norm is less than .01 and giving the final joint angles

println("CTC Final Joint Angle:" ,solCTC[end][1:9])
println("CTC Final Error Norm: ",norm(solCTC[end][1:9]-qd))

# End of Assignment