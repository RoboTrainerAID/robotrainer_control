from common import *

def print_usage():
    print('Usage: python3 pivot.py wheel_name\nValid names:', end='')
    for k in WHEELS.keys():
        print(' ' + k, end='')
    print('')

def callback(argv):
    expect_argv_len(argv, 2, print_usage)
    check_for_invalid_wheel_names(argv[1:], print_usage)

    x, y = WHEELS[argv[1]]
    call_ros_srv(mode=5, icr_x=x, icr_y=y)

if __name__ == '__main__':
    process_argv_with(callback, print_usage)
