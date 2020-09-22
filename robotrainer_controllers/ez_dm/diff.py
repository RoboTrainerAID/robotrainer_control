from common import *

def print_usage():
    print('Usage: python3 diff.py wheel1name wheel2name\nValid names:', end='')
    for k in WHEELS.keys():
        print(' ' + k, end='')
    print('')

def callback(argv):
    expect_argv_len(argv, 3, print_usage)
    check_for_invalid_wheel_names(argv[1:], print_usage)

    x, y, a = pos2ax(WHEELS[argv[1]], WHEELS[argv[2]])
    call_ros_srv(mode=2, axle_x=x, axle_y=y, axle_a=a)

if __name__ == '__main__':
    process_argv_with(callback, print_usage)
