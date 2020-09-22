from common import *

def print_usage():
    print('Usage: python3 omni.py', end='')

def callback(argv):
    expect_argv_len(argv, 1, print_usage)
    call_ros_srv(mode=1)

if __name__ == '__main__':
    process_argv_with(callback, print_usage)
