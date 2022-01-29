using Distributions, LinearAlgebra, Plots, Random, Statistics, StatsPlots

N = 1000000
MEAN = 0
#STD = rand(100)'
STD = [0.05 0.25 0.5]
count = 0

invVar = (1 ./ (STD .* STD))'
sumInvVar = 1 / sum(invVar)
x = zeros(N, 1)
x_sum = 0

for j in 1:N
    z = randn() * STD + [MEAN+1 MEAN-1 MEAN-1]
    x[j] = (z * invVar * sumInvVar)[1]
    global x_sum += x[j]

    if (all(abs(x[j] - MEAN) .<= broadcast(abs, z .- MEAN)))
        global count += 1
    end
end

#println("Sensor stds: ", STD, "\n")

println("Times fused sensor was more accurate: ", 100 * count / N, "%")

println("\nFused sensor average: ", x_sum / N)

println("\nMost accurate sensor std:	", minimum(STD))
println("Fused sensor std:		", stdm(x, x_sum / N))
println("\"Expected\" fused std:		", sqrt(sumInvVar))
