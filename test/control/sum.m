close all; clear all; clc; format short eng;
%%

n = 2; % number of inputs
N = 1000; % number of values

in = rand(N,2);
out = sum(in,2);

file = [ in out ];

save('sum-random.csv', 'file', '-ascii')
