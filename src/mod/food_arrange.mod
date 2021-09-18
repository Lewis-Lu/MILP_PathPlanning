var num_corn >= 0 integer;
var num_milk >= 0 integer;
var num_bread >= 0 integer;

minimize Cost: 0.18*num_corn + 0.23*num_milk + 0.05*num_bread; 

# subject to VitaminA_lb: 107*num_corn + 500*num_milk >= 500;
subject to VitaminB_ub: 500 <= 107*num_corn + 500*num_milk <= 50000;

subject to Calories_lb: 72*num_corn + 121*num_milk + 65*num_bread >= 2000;
subject to Calories_ub: 72*num_corn + 121*num_milk + 65*num_bread <= 2250;