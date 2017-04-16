%This function is to generate random Node in graph 100 * 100

function NodeRand = RANDOM_STATE()

rng shuffle;

NodeRand.x = randi([0,100]);
NodeRand.y = randi([0,100]);


end