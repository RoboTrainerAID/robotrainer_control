WHEELS = {
    # sr2
    'fl': (0.228, 0.184),
    'fr': (0.228, -0.134),
    'bl': (-0.228, 0.134),
    'br': (-0.228, -0.184),
 
    # sr3 
    'f': (0.367032, 0),
    'l': (-0.219122, 0.247322),
    'r': (-0.219122, -0.247322),
}

def is_invalid_wheel_key(k):
    return k not in WHEELS.keys()

# -- CORE OF THE SCRIPT --
# Gets two points on the axle, returns middle of those points and an angle in
# which the axle wants to move.
def pos2ax(wheel_pos_1, wheel_pos_2):
    from math import atan, pi

    x1, y1 = wheel_pos_1
    x2, y2 = wheel_pos_2

    dx = x1 - x2
    dy = y1 - y2

    a = -atan(dx/dy)

    return (x1+x2)*0.5, (y1+y2)*0.5, a


# Exits the program if invalid names are found.
def check_for_invalid_wheel_names(names, print_usage):
    invalid_keys = list(filter(is_invalid_wheel_key, names))
    if len(invalid_keys) > 0:
        print('Invalid wheel names:', end='')
        for k in invalid_keys:
            print(' ' + k, end='')
        print('')
        print_usage()
        exit(1)


def print_all_combinations():
    print('All possible combinations:')
    from itertools import combinations
    for k1, k2 in combinations(WHEELS.keys(), 2):
        v1, v2 = WHEELS[k1], WHEELS[k2]
        axle_x, axle_y, axle_angle = pos2ax(v1, v2)
        print('%s\t%s\t%f\t%f\t%f' % (k1, k2, axle_x, axle_y, axle_angle))



def expect_argv_len(argv, n, print_usage):
    if len(argv) != n:
        print('Incorrect number of arguments. Correct number: %d' % (n-1))
        print_usage()
        exit(2)


def process_argv_with(callback, print_usage):
    from sys import argv
    callback(argv)


def call_ros_srv(mode, icr_x=0, icr_y=0, axle_x=0, axle_y=0, axle_a=0, r_min=0):
    params = map(str, [mode, icr_x, icr_y, axle_x, axle_y, axle_a, r_min])
    cmd = ['bash', 'ros_srv_caller.sh'] + list(params)
    print('RUNNING FOLLOWING COMMAND:\n' + ' '.join(cmd) + '\n')
    from subprocess import check_output, CalledProcessError
    try:
        output = check_output(cmd)
        print('OUTPUT:\n' + output.decode('utf-8'))
    except CalledProcessError as err:
        print('COMMAND FAILED WITH EXIT STATUS %d' % err.args[0])


if __name__ == '__main__':
    print_all_combinations()
