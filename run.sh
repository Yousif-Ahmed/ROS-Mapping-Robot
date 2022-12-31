#!/bin/bash

function sim() {
    roslaunch summit_xl_sim_bringup summit_xls_complete.launch
}

function ira() {
    roslaunch ira_laser_tools laserscan_multi_merger.launch
}

function sens() {
    roslaunch cr_pkg multi.launch
}