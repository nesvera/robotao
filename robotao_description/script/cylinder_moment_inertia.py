import argparse
import math

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Convert videos in a sequence of images')
    parser.add_argument('-radius',
                        dest='radius',
                        required=True,
                        help='Radius')
    parser.add_argument('-height',
                        dest='height',
                        required=True,
                        help='Height')
    parser.add_argument('-mass',
                        dest='mass',
                        required=True,
                        help='Mass')

    args = parser.parse_args()

    radius = float(args.radius)
    height = float(args.height)
    mass = float(args.mass)

    i_xx = mass*(3*math.pow(radius,2) + math.pow(height,2))/12.
    i_yy = i_xx
    i_zz = mass*math.pow(radius,2)/2.
    i_xy = 0
    i_xz = 0
    i_yz = 0

    print()
    print("Moment of inertia")
    print("Ixx: ", i_xx)
    print("Iyy: ", i_yy)
    print("Izz: ", i_zz)
    print("Ixy: ", i_xy)
    print("Ixz: ", i_xz)
    print("Iyz: ", i_yz)