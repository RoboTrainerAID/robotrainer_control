from common import *

def print_usage():
    print('Usage: python3 ack.py minimum_turning_radius wheel1name wheel2name\
\nValid names:', end='')
    for k in WHEELS.keys():
        print(' ' + k, end='')
    print('')

def callback(argv):
    expect_argv_len(argv, 4, print_usage)
    check_for_invalid_wheel_names(argv[2:], print_usage)
    x, y, a = pos2ax(WHEELS[argv[2]], WHEELS[argv[3]])
    call_ros_srv(mode=4, axle_x=x, axle_y=y, axle_a=a, r_min=float(argv[1]))

if __name__ == '__main__':
    process_argv_with(callback, print_usage)
