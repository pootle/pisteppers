#!/usr/bin/python3
"""
This defines most of the settings used by stepperdriver.py and is basically just a config file.
"""
stepsperrev = 144*120*48*16 #144 is ratio of worm drive, 120 is gear ratio in canned stepper motor, 
                #48 is motor's steps per rev
mountParams = {
    'connectTo':('192.168.1.82',56677)
  , 'name':'stepper control'
  , 'printloglevel':63
  , 'sendloglevel':15
  , 'wavepollinterval':.05
  , 'motors':(
        {'name':'RA',  'loglevel':15, 'maxval':stepsperrev, 'flipdir':True
          , 'pins':{'enable':21, 'direction': 12, 'step': 13, 'steplevels': (20,19,16)}
          , 'fastdefaults': {'startsr':3500, 'maxsr':13500, 'overreach':1.1, 'ramp':1, 'pulseontime':2, 'warp':2}}
      , {'name':'DEC', 'loglevel':15, 'maxval':stepsperrev, 'flipdir':True
          , 'pins':{'enable':25, 'direction': 18, 'step': 27, 'steplevels': (24,23,22)}
          , 'fastdefaults': {'startsr':3500, 'maxsr':13500, 'overreach':1.15, 'ramp':1,  'pulseontime':2, 'warp':2}}
      )}