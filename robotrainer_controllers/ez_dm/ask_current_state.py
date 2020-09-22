from common import *

def print_usage():
    print('Usage: python3 ask_current_state.py', end='')

def callback(argv):
    expect_argv_len(argv, 1, print_usage)
    call_ros_srv(mode=0)

if __name__ == '__main__':
    process_argv_with(callback, print_usage)
