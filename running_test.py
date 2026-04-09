import threading

from mks_motor import MKSMotor


motor_a = MKSMotor.open(port=0)
motor_b = MKSMotor.open(port=1)

motor_a.setup()
motor_b.setup()

motor_a.home()
motor_b.home()

# Move both motors at the same time
t1 = threading.Thread(target=motor_a.move_to, args=(50,))
t2 = threading.Thread(target=motor_b.move_to, args=(50,))

t1.start()
t2.start()

t1.join()
t2.join()

motor_a.close()
motor_b.close()
