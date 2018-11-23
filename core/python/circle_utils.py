"""
Here is an example of my code on Python
"""
import numpy as np


def minidisk(coords):
    """
    Computes the smallest enclosing sphere [or circle] of a finite set of points in 3D [or 2D]
    Implement the Welzl's algorithm for d = 3 [or d = 2] dimentions. Original paper's designations are used

    Parameters
    ----------
    coords : N x 3 [or N x 2] array of cartesian coordinates of N points in 3D [or 2D], N - positive integer

    Returns
    -------
    tuple (z, r), where
        z : 1 x 3 [or 1 x 2] array of the coordinates of the center of the smallest enclosing sphere [or circle]
        r : non-negative float, the radius of the smallest enclosing sphere [or circle]
    """

    def b_md(R):
        """
           Computes the smallest sphere [or circle] with points R on its boundary

           Parameters
           ----------
           R : set of elements, each element is a tuple of the coordinates of the points on the smallest sphere's [or circle's] boundary

           Returns
           -------
           tuple (z, r), where
               z : 1 x 3 [or 1 x 2] array of the coordinates of the center of the smallest sphere [or circle]
               r : non-negative float, the radius of the smallest sphere [or circle]
           or None, if R is None
        """
        J = len(R)
        R = list(R)
        if J == 0:  # if the input is an empty set return None
            return None

        elif J == 1:  # if there is only one point, choose it as a center of the disk with zero radius
            z = np.array(R[0])
            r = 0
            return (z, r)

        # if there are two points, choose their mean as a center of the disk,
        # and a half distance between them as a radius
        elif J == 2:
            r1 = np.array(list(R[0]))
            r2 = np.array(list(R[1]))
            z = (r1 + r2) / 2
            r = np.linalg.norm(r1 - r2) / 2
            return (z, r)

        # if there are three points, calculate the center and the radius of the circumscribed circle
        elif J == 3:
            r1 = list(R[0])
            r2 = list(R[1])
            r3 = list(R[2])
            ndims = len(r1)  # 2D or 3D case
            if ndims == 2:
                r1.append(0)
                r2.append(0)
                r3.append(0)
            # 3D-arrays
            r1 = np.array(r1)
            r2 = np.array(r2)
            r3 = np.array(r3)
            r12 = r2 - r1
            r13 = r3 - r1
            n = np.cross(r12, r13)
            norm = np.linalg.norm(n)
            z = r1 + (np.linalg.norm(r13) ** 2 * np.cross(n, r12) + np.linalg.norm(r12) ** 2 * np.cross(r13, n)) / (
                2 * norm ** 2)
            r = np.linalg.norm(r1 - z)
            z = z[0:ndims]
            return (z, r)

        # if there are four points (only in the 3D case), calculate the center and the radius of the minimal sphere, enclosing the tetrahedron
        else:
            r1 = np.array(R[0])
            r2 = np.array(R[1])
            r3 = np.array(R[2])
            r4 = np.array(R[3])
            a0 = np.vstack((r1, r2, r3, r4))
            a = np.hstack((a0, np.ones((4, 1))))
            norm = np.linalg.norm(a0, axis=1) ** 2
            D = np.hstack((norm.reshape(4, 1), a))
            Dx = np.delete(D, 1, axis=1)
            Dy = np.delete(D, 2, axis=1)
            Dz = np.delete(D, 3, axis=1)
            c = np.delete(D, 4, axis=1)
            Dx = np.linalg.det(Dx)
            Dy = -np.linalg.det(Dy)
            Dz = np.linalg.det(Dz)
            a = np.linalg.det(a)
            c = np.linalg.det(c)
            z = np.array([Dx / (2 * a), Dy / (2 * a), Dz / (2 * a)])
            r = np.sqrt(Dx ** 2 + Dy ** 2 + Dz ** 2 - 4 * a * c) / (2 * abs(a))
            return (z, r)

    def b_minidisk(P, R):
        """
           Computes the smallest sphere [or circle] enclosing points P with points R on its boundary

           Parameters
           ----------
           P : set of elements, each element is a tuple of the coordinates of the points into the smallest sphere [or circle]
           R : set of elements, each element is a tuple of the coordinates of the points on the smallest sphere's [or circle's] boundary

           Returns
           -------
           tuple (z, r), where
               z : 1 x 3 array of the coordinates of the center of the smallest enclosing sphere [or circle]
               r : non-negative float, the radius of the smallest enclosing sphere [or circle]
           """
        if P == set():
            D = b_md(R.copy())
        else:
            p = P.pop()
            D = b_minidisk(P.copy(), R.copy())
            chk = False
            if D == None:
                chk = True
            else:
                p_array = np.array(p)
                r = np.linalg.norm(p_array - D[0])
                if r > D[1]:
                    chk = True
            if chk:
                R.add(p)
                D = b_minidisk(P.copy(), R.copy())
        return D

    P = set(tuple(row) for row in coords)
    R = set()
    return b_minidisk(P.copy(), R.copy())
