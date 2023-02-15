#! /usr/bin/env python3

import os
import rospy

trigger_1 = "What are you doing?"
trigger_2 = "Why are you doing this?"
trigger_3 = "How do you achieve your goal?"
trigger_4 = "What is your subgoal?"
trigger_5 = "What are the steps for your subgoal?"
trigger_6 = "What went wrong?"
trigger_7 = "What are you doing?"
trigger_8 = "What are you doing?"




if __name__ == "__main__":
    rospy.init_node('explain_bt_simple_interface')

    print(
        f"Test Explainable BT interface:\n"
        f"Enter \"1\" for {trigger_1}\n"
        f"Enter \"2\" for {trigger_2}\n"
        f"Enter \"3\" for {trigger_3}\n"
        f"Enter \"4\" for {trigger_4}\n"
        f"Enter \"5\" for {trigger_5}\n"
        f"Enter \"6\" for {trigger_6}\n"
        f"Enter \"7\" for {trigger_7}\n"
        f"Enter \"8\" for {trigger_8}"
    )

    while True:
        q = int(input("Question type: "))
        if q == 1:
            what = trigger_1
        elif q == 2:
            what = trigger_2
        elif q == 3:
            what = trigger_3
        elif q == 4:
            what = trigger_4
        elif q == 5:
            what = trigger_5
        elif q == 6:
            what = trigger_6
        elif q == 7:
            what = trigger_7
        elif q == 8:
            what = trigger_8
        print(what)
        os.system("rosservice call /explainable_bt \"what: '" + what + "'\"")

