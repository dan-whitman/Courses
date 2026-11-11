function [MA, DM, DEM, NDDEM] = classification_metrics(testdata, inferreddata, edges)
    %%%%% match accuracy (MA)
    MA = sum(inferreddata == testdata) / length(testdata);

    %%%%% distance metric (DM)
    datarange = 1:(length(edges)-1);
    datavals = (edges(1:end-1)+edges(2:end))/2;
    testdataest = sum((testdata == datarange).*datavals,2);
    inferreddataest = sum((inferreddata == datarange).*datavals,2);

    DM = sum(abs(inferreddataest - testdataest)) / length(testdata); % in dB

    %%%%% distance expectation metric (DEM)    
    probdist = sum(testdata == datarange) / length(testdata);
    expect = sum(datavals.*probdist);

    DEM = sum(abs(inferreddataest - expect)) / length(testdata); % in dB

    %%%%% normalized discrete distance expectation metric
    ND = 2*(0.5*length(datarange)-1)/((length(datarange)-1)*(length(datarange)-2));

    NDDEM = sum(abs((inferreddata-testdata))*ND) / length(testdata);
end