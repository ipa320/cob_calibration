#!/usr/bin/env python
from itertools import permutations
import random
seed = [0,0,0,-1.57, 1.57, 0]
variance = [-.4, -.3, -.2, -.1, 0, .1, .2, .3, .4]

perms  = permutations(variance,4)
perm_lis = []
for p in perms:
    perm_lis.append(p)
perms = random.sample(perm_lis,60)
perms.sort()
for p in perms:
    pos=seed
    for i in range(4):
        pos[i+2]+=p[i]
    print "- joint_position: ",pos
    print "  torso_position: [0,0]"

    for i in range(4):
        pos[i+2]-=p[i]


