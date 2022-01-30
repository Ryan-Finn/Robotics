using Distributions, LinearAlgebra, Plots, Random
using Statistics, StatsPlots, SpecialFunctions

Random.seed!(0);
global n = 100000
global num = 3
global Mean = 0
global STD = broadcast(abs, 200*rand(num)')
sort!(STD')

function init()
    println("Senor STDs:\n", STD, "\n")

    Z = zeros(10000, num)
    MEAN = Mean * (2*rand(num)' .- 1)
    for i in 1:10000
        Z[i, :] = MEAN + STD .* randn(num)'
    end

    M = zeros(1, num)
    S = zeros(1, num)
    for i in 1:num
        M[i] = mean(Z[:, i])
        S[i] = stdm(Z[:, i], M[i])
    end
    M = MEAN - M .+ Mean

    return M, S
end

global root2 = sqrt(2)
function fuseTwo(Z, M, S)
    w1 = 1 / erf(abs(Z[1] - M[2]) / (S[2] * root2))'
    w2 = 1 / erf(abs(Z[2] - M[1]) / (S[1] * root2))'
    invSum = 1 / (w1 + w2)

    z = (Z[1] * w1 + Z[2] * w2) * invSum
    m = (M[1] * w1 + M[2] * w2) * invSum
    s = (S[1] * w1 + S[2] * w2) * invSum

    return z, m, s
end

function fuse(Z, M, S)
    sz = size(Z, 2)
    if sz == 2
        return fuseTwo(Z, M, S)
    end

    z = zeros(1, sz - 1)
    m = zeros(1, sz - 1)
    s = zeros(1, sz - 1)
    for i in 1:sz-1
        z[i], m[i], s[i] = fuseTwo(Z[i:i+1], M[i:i+1], S[i:i+1])
    end

    return fuse(z, m, s)
end

function main()
    M, S = init()

    X = zeros(n, 2)
    x_sum = [0.0 0.0]
    count = zeros(1, 3)

    invVar = (1 ./ (S .* S))'
    sumInvVar = 1 / sum(invVar)
    for i in 1:n
        Z = M + STD .* randn(num)'

        X[i, 1], _, _ = fuse(Z, M, S)
        x_sum[1] += X[i, 1]
        if (all(abs(X[i, 1] - Mean) .<= broadcast(abs, Z .- Mean)))
            count[1] += 1
        end

        X[i, 2] = (Z*invVar*sumInvVar)[1]
        x_sum[2] += X[i, 2]
        if (all(abs(X[i, 2] - Mean) .<= broadcast(abs, Z .- Mean)))
            count[2] += 1
        end

        if (abs(X[i, 1] - Mean) <= abs(X[i, 2] - Mean))
            count[3] += 1
        end
    end

    println("Time Romberg sensor was more accurate:		", 100 * count[1] / n, "%")
    println("Time Fused sensor was more accurate:		", 100 * count[2] / n, "%")
    println("Time Romberg was more accurate than Fused:	", 100 * count[3] / n, "%")

    println("\nRomberg sensor average:	", x_sum[1] / n)
    println("Fused sensor average:	", x_sum[2] / n)

    println("\nRomberg sensor STD:		", stdm(X[:, 1], x_sum[1] / n))
    println("Fused sensor STD:		", stdm(X[:, 2], x_sum[2] / n))
    println("Most accurate sensor STD:	", minimum(STD), "\n")
end

main()
