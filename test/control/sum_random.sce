for n = [2 3 10]
	N = 1000;

	in = rand(N,n);
	out = sum(in,2);

	f = [ in out ];

//	save(['sum-random-' msprintf('%d',n) '.dat'], 'f')
    write_csv(f,"sum-random-" + string(n) + ".csv", ascii(9))
	
end
