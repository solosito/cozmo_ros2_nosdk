#!/usr/bin/env python
import rclpy
import sys, select, termios, tty
from geometry_msgs.msg import Twist

settings = termios.tcgetattr(sys.stdin)

msg = '''
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

o : lift up
p : lift down
l : head up
; : head down

anything else : stop

t/y : increase/decrease movement speed by 10%
g/h : increase/decrease lift speed by 10%
b/n : increase/decrease head speed by 10%

CTRL-C to quit
'''

moveBindings = {
        'w' : ( 1.0, 0.0, 0.0, 0.0),
        'e' : ( 1.0, 0.0, 0.0,-1.0),
        'a' : ( 0.0, 0.0, 0.0, 1.0),
        'd' : ( 0.0, 0.0, 0.0,-1.0),
        'q' : ( 1.0, 0.0, 0.0, 1.0),
        'x' : (-1.0, 0.0, 0.0, 0.0),
        'c' : (-1.0, 0.0, 0.0, 1.0),
        'z' : (-1.0, 0.0, 0.0,-1.0),
        'o' : ( 0.0, 0.0, 1.0, 0.0),
        'p' : ( 0.0, 0.0,-1.0, 0.0),
        'l' : ( 0.0, 1.0, 0.0, 0.0),
        ';' : ( 0.0,-1.0, 0.0, 0.0),
           }

speedBindings={
        't' : (1.1, 1.0, 1.0), # Movement speed
        'y' : (0.9, 1.0, 1.0), # Movement speed
        'g' : (1.0, 1.1, 1.0), # Lift speed
        'h' : (1.0, 0.9, 1.0), # Lift speed
        'b' : (1.0, 1.0, 1.1), # Head speed
        'n' : (1.0, 1.0, 0.9), # Head speed
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(movement_speed, lift_speed, head_speed):
    return 'current speeds:\tmovement {:.3f}\tlift {:.3f}\thead {:.3f} '.format(movement_speed, lift_speed, head_speed)

def main(args=None):

    rclpy.init(args=args)
    node = rclpy.create_node(node_name='teleop_twist_keyboard', namespace='cozmo')
    pub  = node.create_publisher(Twist, 'cmd_vel', 1)

    movement_speed = 0.2
    head_speed     = 2.0
    lift_speed     = 2.0
    x              = 0.0
    y              = 0.0
    z              = 0.0
    th             = 0.0
    status         = 0

    try:
        print(msg)
        print(vels(movement_speed, lift_speed, head_speed))
        while(True):
            key = getKey()
            if key in moveBindings.keys():
                x  = moveBindings[key][0]
                y  = moveBindings[key][1]
                z  = moveBindings[key][2]
                th = moveBindings[key][3]

            elif key in speedBindings.keys():
                movement_speed = movement_speed * speedBindings[key][0]
                head_speed     = head_speed     * speedBindings[key][1]
                lift_speed     = lift_speed     * speedBindings[key][2]

                print(vels(movement_speed, lift_speed, head_speed))
                if (status == 5): # 14
                    print(msg)
                status = (status + 1) % 6

            else:
                x  = 0.0
                y  = 0.0
                z  = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            twist = Twist()

            twist.linear.x  = x  * movement_speed;
            twist.linear.y  = y  * head_speed;
            twist.linear.z  = z  * lift_speed;
            twist.angular.z = th * movement_speed

            twist.angular.x = 0.0;
            twist.angular.y = 0.0;

            pub.publish(twist)

    except:
        print(sys.exc_info())

    finally:
        pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()