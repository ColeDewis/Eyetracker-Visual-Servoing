import numpy as np
import rospy
import scipy.interpolate as interp

class LinearNDInterpolatorExt(object):
    def __init__(self, points, values):
        self.in_d = points.shape[1]
        self.out = values.shape[1]
        self.npairs = points.shape[0]
        self.funcinterp = interp.LinearNDInterpolator(points,values)
        self.funcnearest = interp.NearestNDInterpolator(points,values)
    def __call__(self,*args):
        t = self.funcinterp(*args)

        # TODO: not sure if this should be .all or .any
        if not np.isnan(t).all():
            return t.flatten()
        else:
            return self.funcnearest(*args)
    
    def __str__(self):
        return f"LinearNDInterpolator(in: {self.in_d}, out: {self.out}, npairs: {self.npairs})"

class MockRobot:
   def __init__(self):
      pass
   
   def move_joint_vel(self, vels):
      return
   
class MockError:
   def __init__(self, dim):
      self.dim = dim
   def read_error(self):
      return np.random.randint(0, 10, size=self.dim)

# TODO robot interface
# TODO error interface
class PiecewiseLinearVS:
    def __init__(self, robot, error_interface, error_dim: int, dof: int):
        self.robot = robot
        self.error_dim = error_dim
        self.error_interface = error_interface
        self.dof = dof
        self.interpolator = None
        self.error_cache = None
        self.joint_cache = None
    
    def initialize(self):
        # need error_dim + 2 test movements generally it seems.
        MOVEMENTS_NEEDED = self.error_dim + 2

        # https://www.mathworks.com/matlabcentral/answers/1653560-how-to-generate-m-almost-mutually-orthogonal-vectors-of-dimensionality-n-where-m-n
        # apparently am relaxing a bit with normalizing but should be fine
        
        # generate error_dim+2 mostly orthogonal movements to ideally have a well conditioned initialization
        # TODO: try this method w secant constriants
        U, _, Vt = np.linalg.svd(np.random.rand(MOVEMENTS_NEEDED, self.dof), full_matrices=False)
        movements = U @ Vt
        norms = np.apply_along_axis(np.linalg.norm, 1, movements)
        movements = movements / norms.reshape(-1, 1)
        movements = movements * 0.1 # scale down a to norm of 0.1

        WAIT_TIME = 0.5
        errors = []
        for i in range(MOVEMENTS_NEEDED):
            # send joint command
            error_init = self.error_interface.read_error()
            move = movements[i, :]
            self.robot.move_joint_vel(move) # TODO

            rospy.sleep(WAIT_TIME)
            self.robot.move_joint_vel(np.zeros(self.dof))

            # read delta error
            error_final = self.error_interface.read_error()
            delta_e = error_final - error_init
            errors.append(delta_e)

        self.joint_cache = movements
        self.error_cache = np.array(errors)
        self.interpolator = LinearNDInterpolatorExt(self.error_cache, self.joint_cache)
        print(self.error_cache.shape, self.joint_cache.shape)

    def move_and_update(self, error):
        move = self.interpolator(error)
        self.robot.move_joint_vel(move)
        error_init = self.error_interface.read_error()
        rospy.loginfo(f"Move: {move}")

        rospy.sleep(0.5) # TODO
        error_final = self.error_interface.read_error()
        delta_e = error_final - error_init

        self.error_cache = np.vstack([self.error_cache, delta_e])
        self.joint_cache = np.vstack([self.joint_cache, move])
        self.interpolator = LinearNDInterpolatorExt(self.error_cache, self.joint_cache) # have to recreate, which sucks, but for proof of concept is fine.
       


if __name__ == "__main__":
    err = MockError(4)
    rbt = MockRobot()
    vs = PiecewiseLinearVS(rbt, err, 4, 7)
    vs.initialize()
    vs.move_and_update(np.array([0, 0, 1, 0]))
    print(vs.interpolator)

