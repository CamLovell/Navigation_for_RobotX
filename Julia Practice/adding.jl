using Printf
using Statistics

function add(a,b)
	c = a+b
	return(c)
end

function addarray(array)
	sum = 0
	for i in array
		sum += array[i]
	end
	return(sum)
end
