"""hw3 behavior."""

from __future__ import print_function
from __future__ import division
from __future__ import absolute_import
from math import pi

import memory
import mem_objects
import core
import commands
import cfgstiff
from task import Task
from state_machine import Node, S, C, T, LoopingStateMachine
import UTdebug

DELAY = 0.05
BALL_MIN = 100
GOAL_MIN = 500

VX_MIN = 0.1
VX_MAX = 1
VY_MIN = 0.1
VTHETA_MIN = 0.1
VTHETA_MAX = 0.25

FACTOR = 0.01
SCALE = 0.005

P = 1.
I = 0.
D = 0.

vx = 0.
vy = 0.
vtheta = 0.
goal_side = 1


class Controller(object):
    def __init__(self, p, i, d, T=30):
        self.Kp = p
        self.Ki = i
        self.Kd = d

        self.last_error = 0.
        self.pterm = 0.
        self.iterm = 0.
        self.dterm = 0.
        self.i = 0

        self.T = T

    def __call__(self, error):
        self.pterm = error
        self.iterm += error
        self.dterm = (error - self.last_error) / DELAY

        print ("p=%.8f\t"%(self.pterm * self.Kp), "d=%.8f"%(self.dterm * self.Kd))

        self.i += 1
        if self.i >= self.T:
            self.clear()

        self.last_error = error

        return self.pterm * self.Kp + self.iterm * self.Ki + self.dterm * self.Kd

    def clear(self):
        self.pterm = 0.0
        self.iterm = 0.0
        self.dterm = 0.0
        self.last_error = 0.0
        self.i = 0

class Stand(Node):
    def run(self):
        commands.stand()
        # commands.setHeadPanTilt(pan=0, tilt=0, time=0.5, isChange=True)
        if self.getTime() > 3.0:
            memory.speech.say("stand up")
            self.finish()


class FollowBall(Node):
    def run(self):
        commands.setWalkVelocity(vx, vy, vtheta)



class FindBall(Node):
    def __init__(self):
        super(FindBall, self).__init__()
        # self.vtheta_controller = Controller(p=1.0, i=0.0, d=0.0)
        self.controller = Controller(p=P, i=I, d=D)

    def run(self):
        global vx
        global vy
        global vtheta
        global goal_side

        vx0, vy0, vtheta0 = vx, vy, vtheta

        ball = mem_objects.world_objects[core.WO_BALL]
        goal = mem_objects.world_objects[core.WO_UNKNOWN_GOAL]

        ball_theta = ball.visionBearing
        ball_x = ball.visionDistance

        goal_theta = goal.visionBearing
        goal_x = goal.visionDistance

        # Yajie's goal seen earlier code
        if goal.seen:
            goal_side = goal_theta / abs(goal_theta)
            # print ('Goal Side  %d' % goal_side )

        if not ball.seen:
            print ('\n\n\n\nBall not seen.\n\n\n\n')
            vx = 0.0
            # vy = FACTOR*vy
            # vtheta = FACTOR*vtheta
            vy = 0.0
            vtheta = 0.5
        elif ball.fromTopCamera and ball_x > BALL_MIN:
            print ('\n\n\n\nBall seen far away.\n\n\n\n')
            vx = self.controller(min( (ball_x - BALL_MIN) * SCALE, VX_MAX))
            vtheta = min ( (abs(ball_theta) > VTHETA_MIN)*abs(ball_theta), VTHETA_MAX
                ) * (ball_theta/abs(ball_theta))
        else:
            vx = 0.1
            if goal.seen:
                print ('\n\n\n\nGoal visible.\n\n\n\n')
                vy = -ball_theta
                vtheta = (ball_theta-goal_theta)
            else:
                print ('\n\n\n\nGoal NOT visible. goal side: ', goal_side, "\n\n\n\n")
                vy = -0.5 * goal_side
                vtheta = -0.5*vy


        vx += FACTOR*(vx-vx0);
        vy += FACTOR*(vy-vy0);
        vtheta += FACTOR*(vtheta-vtheta0);

        vx *= (abs(vx) > VX_MIN)
        vy *=  (abs(vy) > VY_MIN)
        vtheta *=  (abs(vtheta) > VTHETA_MIN)

        self.finish()

class Playing(LoopingStateMachine):
    def setup(self):
        findball = FindBall()
        followball = FollowBall()
        stand = Stand()
        self.add_transition(stand, C, findball)
        self.add_transition(findball, C, followball, T(DELAY), findball)
