function flag = isGoal(p,targetBox)

if p(1) >= targetBox(1) && p(2) >= targetBox(2) && p(1) <= targetBox(3) && p(2) <= targetBox(4)
    flag = true;
else
    flag = false;
end