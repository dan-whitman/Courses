%%%%% MAE 5810 Homework 2
% Daniel Whitman

%%%%% Bayesian Network
% Inputs acoustic data to generate a Bayesian Network that can infer the
% propagation loss (PL) of the acoustic measurement given the input
% parameters (or can infer the inputs given any number of other evidence
% observations).
% 
% Inputs:
% - Range (R)
% - Depth (Z)
% - Frequency (F)
% - Bottom Density (BD)
% - Bottom Sound Speed (V)
% - Sea Floor (SF)
% 
% Outputs:
% - Propagation Loss (PL)
% 
% Network Generation:
% This code uses the provided database to both generate the network structure
% and determine the given parameters (CPTs) of the model using the Bayes
% Net Toolbox (BNT) for MATLAB.
% 
% Metrics:
% - Match Accuracy (MA)
% - Distance Metric (DM)
% - Distance Expectation Metric (DEM)
% - Normalized Discrete Distance Expectation Metric (NDDEM)
% 

%%%%% configuration
clear, close all;
% clc;

seed = 42;
rng(seed, 'twister'); % setting rng seed for reproducability

addpath('data'); addpath(genpath('utils')); % adding directories
addpath(genpathKPM(pwd)) % adding BNT toolbox

config.discrete = true; % (true) models all nodes as discrete, (false) models some as continuous
config.k2 = true; % (true) uses K2 structure learning, (false) uses MCMC structure learning
config.verbose = 'no'; % ('yes') outputs K2 training info, ('no') does not
config.dataskip = false; % (true) takes only every k-th row of data, (false) takes all
config.CPTs = false; % (true) displays model CPTs for node is, (false) does not
config.log = true; % (true) bins PL data logarithmically, (false) bins it linearly
config.graph = false; % (true) prints figure of graph, (false) does not

%%%%% model parameters
trainpercent = 0.8; % percentage of data to train on
numbinsPL = 10; % number of discrete PL bins to train with
dataskip = 4; % taking every k-th row of data to speed training
CPTnodes = [4,7]; % node CPTs to print for observation

rawdata = load("data/data_updateBN2-1.mat"); % loading the database
names = {'SF', 'F', 'BD', 'V', 'Z', 'R', 'PL'};
SF = 1; F = 2; BD = 3; V = 4; Z = 5; R = 6; PL = 7; % for future referencing

if ~config.dataskip, dataskip = 1; end % don't skip any data

% extracting data: [SF, F, BD, V, Z, R, PL]
rawSF = rawdata.data(dataskip:dataskip:end,SF); % sea floor
rawF = rawdata.data(dataskip:dataskip:end,F); % frequency
rawBD = rawdata.data(dataskip:dataskip:end,BD); % bottom density
rawV = rawdata.data(dataskip:dataskip:end,V); % bottom sound speed
rawZ = rawdata.data(dataskip:dataskip:end,Z); % depth
rawR = rawdata.data(dataskip:dataskip:end,R); % range
rawPL = rawdata.data(dataskip:dataskip:end,PL); % propagation loss

% preprocessing data (integer-izing for bnet)
[numSF, ~, intSF] = unique(rawSF); numSF = length(numSF);
[numF, ~, intF] = unique(rawF); numF = length(numF);
[numBD, ~, intBD] = unique(rawBD); numBD = length(numBD);
[numV, ~, intV] = unique(rawV); numV = length(numV);
[numZ, ~, intZ] = unique(rawZ); numZ = length(numZ);
[numR, ~, intR] = unique(rawR); numR = length(numR);

if config.log
    edgesPL = logspace(log10(min(rawPL)-1), log10(max(rawPL)+1), numbinsPL+1); % log spacing for discretization
else
    edgesPL = linspace(min(rawPL)-1, max(rawPL)+1, numbinsPL+1); % linear spacing for discretization
end
intPL = discretize(rawPL, edgesPL); % need to force discrete for structure findings

predata = [intSF,intF,intBD,intV,intZ,intR,intPL]; % converting to new data
% predata = [intSF,intF,intBD,intV,intZ,intR,rawPL]; % converting to new data

% splitting data into training and testing
[traindata, testdata] = data_split(predata, trainpercent, 1);
[trainnum, nodenum] = size(traindata); [testnum, ~] = size(testdata); % sizes of data

%%%%% finding network structure
if config.k2, order = [1:nodenum]; end % topological node numbering
if config.discrete, nodesize = [numSF,numF,numBD,numV,numZ,numR,numbinsPL]; % all discrete nodes
else, nodesize = [numSF,ones(1,nodenum-1)]; end % all continuous nodes except sea floor

% structure learning
if config.k2 % if using K2
    dag = learn_struct_K2(traindata', nodesize, order, ...
        'max_fan_in', nodenum-1, 'discrete', [1:nodenum], 'verbose', config.verbose);
else % if using MCMC
    [sampledag, acceptance, numedges] = learn_struct_mcmc(traindata', nodesize, ...
        'max_fan_in', nodenum-1, 'discrete', [1:nodenum], 'init_dag', zeros(nodenum,nodenum));
    [maxi, idx] = min(acceptance);
    dag = sampledag{idx};
end

%%%%% creating network
if config.discrete % if using all discrete nodes
    bnetstruct = mk_bnet(dag, nodesize, ...
        'names', names, 'discrete', [1:nodenum], 'observed', [1:nodenum]);
else % if using continuous nodes
    bnetstruct = mk_bnet(dag, nodesize, ...
        'names', names, 'discrete', [1], 'observed', [1:nodenum]);
end
disp(bnetstruct.dag) % displaying adjacency matrix
if config.graph, draw_graph(bnetstruct.dag); end % showing graph

%%%%% learning CPT parameters
bnetstruct.CPD{SF} = tabular_CPD(bnetstruct, SF); % SF CPT
for i = 2:nodenum % creating for remainder of nodes
    if config.discrete, bnetstruct.CPD{i} = tabular_CPD(bnetstruct, i); % discrete CPTs
    else, bnetstruct.CPD{i} = gaussian_CPD(bnetstruct, i); end % continuous CPDs
end

bnettrain = learn_params(bnetstruct, traindata'); % finding the parameters

% displaying generated CPTs
if config.CPTs
    CPTs = cell(1,nodenum);
    for i = CPTnodes
        s = struct(bnettrain.CPD{i});  % violate object privacy
        disp(s.CPT); fprintf('\n');
    end
end

%%%%% inferring testing data
warning('off', 'all');
engine = jtree_inf_engine(bnettrain); % creating inference engine

inferredPL = zeros(testnum,1); % empty inference array
for i = 1:testnum % for every test case
    evidence = cell(1,nodenum); % create empty evidence array
    for j = 1:nodenum-1
        evidence{j} = testdata(i,j); % adding observed data
    end
    [~, mpe_states, ~] = evalc('calc_mpe(engine, evidence);'); % performing inference
    inferredPL(i) = mpe_states{PL};
end
warning('on', 'all');

%%%%% determining model performance
[MA, DM, DEM, NDDEM] = classification_metrics(testdata(:,PL), inferredPL, edgesPL);
fprintf('\nMA = %4.4f', MA);
fprintf('\nDM = %4.4f', DM);
fprintf('\nDEM = %4.4f', DEM);
fprintf('\nNDDEM = %4.4f', NDDEM);
fprintf('\n');

