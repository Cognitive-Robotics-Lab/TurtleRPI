#!/usr/bin/env python

import roslib
roslib.load_manifest('TurtleMaster')
import rospy
import time
import json
import sys
from random import *
import threading


class Command:

    def __init__(self, name, params, func):
        self.name = name
        # generate parameter types
        self.params = [type(x) for x in params]
        self.func = func

    #
    def __call__(self, params):
        init = "%s(%s)" % (self.name,  ", ".join(str(x) for x in params))
        print init

        if(len(params) != len(self.params)):  # parameter length check
            error = "Incorrect number of parameters\n Require:" + \
                str(len(self.params))
            error += "\n Received:" + str(len(params))
            print error
            return [init, "Error", error]

        try:  # typify all parameters
            full_params = map(lambda x: x[0](x[1]), zip(self.params, params))
        except:
            # if(all(map(lambda x: x[0]==x[1],zip([type(x) for x in
            # params],self.params)))):
            error = "Incorrect parameter types\n Require:" + str(self.params)
            error += "\n Received:", str([type(x) for x in params])
            print error
            return [init, "Error", error]

        response = self.run_command(full_params)
        print "Status: %s \n\twith: %s" % (response[0], response[1])
        return [init] + response

    # run main action through callback
    def run_command(self, params):
        return self.func(params)


class MetaCommand(Command):

    def __init__(self, name, params, command_block, available_commands):
        super(Command, self).__init__(name, params, self.chained_func)
        self.command_block = self.generateBlock(
            command_block, available_commands)
        self.command_index = 0

    def generateBlock(block_json, command_list):
        commands = []
        # parse json into block

    def chained_func(self, params):
        status_chain = []
        self.command_index = 0
        # execute the meta block keeping track of results and index


class TurtleMaster():
#    pub = rospy.Publisher('turtle_commands', String)

    def __init__(self):
        self.commands = {}
        self.commands["goto"] = Command("goto", [
                                        "id", 1.0, 1.0, 1.0, 1.0, 1.0, 1.0], self.goto_pose)
        self.commands["follow"] = Command(
            "follow", ["id", 1.0], self.follow_agent)

        self.state = {}
        self.setup_state()

        self.input_thread = threading.Thread(target=self.input_run)
        self.input_thread.start()

        self.blocks = []
        self.curr_block = None
        self.curr_block_index = 0

    #
    def goto_pose(self, params):
        print "goto", params

    #
    def follow_agent(self, params):
        print "follow", params
        return ["Success", "Followed"]

    #
    def input_run(self):
        digest = ""
        while True:
            try:
                time.sleep(3)
                with open("input.json", "r") as f:
                    json_in = json.load(f)
                    print json_in
            except IOError:
                print "="*20
                print "\n", "Can't open input.json", "\n"
                print "="*20

# msg = msg.split()
#
# if msg[0] in self.commands:
# self.commands[msg[0]](msg[1:])
# else:
# print 'Not a valid command'
    #
    def setBlock(self, block_json):
        pass

    # setup initial state
    def setup_state(self):
        self.state["agents"] = dict(zip(
            (str(x) for x in range(1, 5)),
            ([uniform(0, 10), uniform(0, 10), uniform(0, 10)]
             for x in range(1, 5)))
        )
        self.state["executed_block"] = {}
        self.state["current_block"] = {}

    # ignore simulation
    def set_new_state(self):
        self.state["executed_block"] = {"index": len(self.blocks),
                                        "block": self.blocks[-1] if (len(self.blocks) > 0) else []}
        if self.curr_block:
            self.state["current_block"] = {"index": self.curr_block_index+1,
                                           "metaindex": self.curr_block.command_index+1
                                           if (isinstance(self.curr_block, MetaCommand)) else 1,
                                           "executed_block": self.curr_block_results}
        for id, pose in self.state["agents"].iteritems():
            if randint(0, 1):
                self.state["agents"][id] = (pose[0]+uniform(-3, 3),
                                            pose[1]+uniform(-3, 3),
                                            pose[2]+uniform(-3, 3))

    # state to json
    def getState(self):
        self.set_new_state()
        return repr(json.dumps(self.state, sort_keys=True, indent=2))

if __name__ == "__main__":
    tm = TurtleMaster()

    # ignore icky threaded input code
    def _exitCheckfunc():
        print "ok"
        try:
            while 1:
                alive = False
                if tm.input_thread.isAlive():
                    alive = True
                if not alive:
                    break
                time.sleep(1)
        except KeyboardInterrupt, e:
            pass
            # traceback.print_exc()
        # print "consume time :",time.time()-start

    threading._shutdown = _exitCheckfunc
