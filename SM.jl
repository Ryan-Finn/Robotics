####################################################
# This is a Julia program to implement the simple motion algorithm
# Jeff McGough
# SDSMT 2022
####################################################

using Plots
using LinearAlgebra

##
# Simple setup function.
# Writing lots of little functions works best in Julia.
# Helps with the funky scoping rules
##
function init()
	lx = zeros(0)
	ly = zeros(0)
	x = 0
	y = 0
	θ = 0
	dt = 0.1
	goal = false
	return x, y, θ, lx, ly, dt, goal
end

##
# This is probable too much on the small function idea
##
function ArrayPush(new_x,new_y,lx,ly)
	append!(lx,new_x)
	append!(ly,new_y)
end

##
# Am I still in the simulation region?
# Return true if so, false otherwise
##
function region(x,y)
	if (x < 0 || x > 10 || y < -5 || y > 5)
		return false
	end
	return true
end

##
# Am inside the obstacle?  Return true
##
function obstacle(new_x, new_y)
	value = (new_x - 5)^2 + (new_y)^2 - 4
	if(value < 0)
		return true
	else
		return false
	end
end

##
# Helper function to graph the obstacle
##
function obsbndry(h,k,r)
	t = range(0, 2*π, length = 120)
	circx = r .* cos.(t) .+ h
	circy = r .* sin.(t) .+ k
	return circx,circy
end

##
# The "main".  This is the path planner.
##

function main()
	# Setup the variables
	x, y, θ, lx, ly, dt, goal = init()
	goalx, goaly = 10,0
	detour_count = 0
	detour = false
	# Start motion
	while (!goal)
		# Take a step to goal
		new_x = x + dt*cos(θ)
		new_y = y + dt*sin(θ)
		# Stop if you step outside the playing field
		if (!region(new_x,new_y))
			break
		end
		# If you step into an obstacle, turn right and try again ...
		while(obstacle(new_x,new_y))
			θ = θ - 0.5
			detour = true
			new_x = x + dt*cos(θ)
			new_y = y + dt*sin(θ)
			# Note - did not check if θ > 2π (infinite loop issue)
		end
		# Now in free space, update location
		x, y = new_x, new_y
		ArrayPush(x,y,lx,ly)
		# You will make N steps in the new direction
		# Then turn back to motion towards goal
		if detour
			detour_count += 1
			if detour_count > 8
				θ = atan(goaly - y, goalx - x)
				detour = false
				detour_count = 0
			end
		end

	end
	# Plot obstacle and path
	circx,circy = obsbndry(5,0,2)
	p = plot(lx,ly, xlim = (0,10), ylim = (-5,5), legend = false, color = :blue)
	plot!(circx,circy)
	display(p)
	readline()
end

main()
