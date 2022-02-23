using Distributions, LinearAlgebra, Plots, Random
using Statistics, StatsPlots, SpecialFunctions, Printf

#Random.seed!(0);
global n = 100000
global num = 10
global Mean = 1
global STD = broadcast(abs, rand(num)')
sort!(STD')

function init()
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

global c = 1 / sqrt(2*pi)
global e = exp(-0.5)
function normal(Z, M, S)
	p = (Z .- M) ./ S
	return c .* e.^(p .* p) ./ S
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
		return fuseTwo(Z, M, S)[1]
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

	X = zeros(n, 3)
	count = zeros(1, 3)
	metric = zeros(3)

	invVar = (1 ./ (S .* S))'
	sumInvVar = 1 / sum(invVar)
	for i in 1:n
		Z = M + STD .* randn(num)'

		freq = normal(Z, M, S)'
		freq = freq .* freq
		sumFreq = sum(freq)

		X[i, 1] = fuse(Z, M, S)
		X[i, 2] = (Z * freq / sumFreq)[1]
		X[i, 3] = (Z * invVar * sumInvVar)[1]
		metric[1] += abs(X[i, 1] - Mean)
		metric[2] += abs(X[i, 2] - Mean)
		metric[3] += abs(X[i, 3] - Mean)

		if all(abs(X[i, 1] - Mean) .<= broadcast(abs, Z .- Mean))
			count[1] += 1
		end
		if all(abs(X[i, 2] - Mean) .<= broadcast(abs, Z .- Mean))
			count[2] += 1
		end
		if all(abs(X[i, 3] - Mean) .<= broadcast(abs, Z .- Mean))
			count[3] += 1
		end
	end
	#println("Senor STDs:\n", STD, "\n")

	println("Time Romberg sensor was more accurate:		", 100 * count[1] / n, "%")
	println("Time Frequency sensor was more accurate:	", 100 * count[2] / n, "%")
	println("Time IV sensor was more accurate:		", 100 * count[3] / n, "%")

	@printf("\nAverage Romberg sensor error:	%0.3f%%\n", 100 * metric[1] / n / Mean)
	@printf("Average Frequency sensor error:	%0.3f%%\n", 100 * metric[2] / n / Mean)
	@printf("Average IV sensor error:	%0.3f%%\n", 100 * metric[3] / n / Mean)

	println("\nRomberg sensor STD:		", std(X[:, 1]))
	println("Frequency sensor STD:		", std(X[:, 2]))
	println("IV sensor STD:			", std(X[:, 3]))
	println("Most accurate sensor STD:	", minimum(STD), "\n")
end

main()
