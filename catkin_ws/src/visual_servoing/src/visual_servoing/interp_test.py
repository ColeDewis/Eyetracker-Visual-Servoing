import numpy as np
import scipy.interpolate as interp

# LinearNDInterpolator: The interpolant is constructed by triangulating the input data with Qhull [1], 
# and on each triangle performing linear barycentric interpolation.
# This will require a fairly large number of points

class LinearNDInterpolatorExt(object):
  def __init__(self, points, values):
    self.funcinterp = interp.LinearNDInterpolator(points,values)
    self.funcnearest = interp.NearestNDInterpolator(points,values)
  def __call__(self,*args):
    t = self.funcinterp(*args)
    print(t)
    if not np.isnan(t).all():
      return t.flatten()
    else:
      return self.funcnearest(*args)

if __name__ == "__main__":
    q = np.random.rand(4, 7)
    # q = np.array([
    #     [5, 5, 3,3], # e.g. delta q 1
    #     [5, 7, 3,3],
    #     [7, 5, 3,3],
    #     [10, 3, 4,3],
    # ])
    # e = np.array([
    #     [1, 2, 2, 4], # e.g. delta e 1
    #     [4, 3, 1, 1],
    #     [6, 3, 2, 1],
    #     [4, 4, 4, 4],
    # ])
    e = np.random.rand(4, 4)
    # print(q)
    # print(e)
    # print(e[:, 0])
    func = LinearNDInterpolatorExt(e, q)
    
    print(func([0.5, 0.5, 0.5]))

    # Some notes:
    # will need enough data to initialize interpolation, don't know if that is reasonable for us
    #   - it seems we need N_dim + 2 points (so 7DOF -> 9 points)
    #   - don't really know what to do prior to getting this
    # extrapolation is nearest neighbor (not great)
    # dont think there's any way to "add" another point w/o recomputing (even if i hack something into source code), which is unfortunate

    # however, i do think this prob would work. could start with 2DOF and low error dimension to visualize the function learned.

    # how to compare? interaction matrix seems to use 6dof
    #   - in past experiments, one way to compare is to the basis jacobian at reference points
    #   - could somehow use analytical (interaction matrix) and then translate it using robot jacobian into the space, need to try that.

    # TODOs:
    #   1. try and implement servoing with this just to see if it can work (2DOF)
    #   2. setup RBT ?
    # https://github.com/petercorke/robotics-toolbox-python/blob/master/roboticstoolbox/models/DH/TwoLink.py, can extend to 3 easily