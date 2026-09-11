import unittest
import numpy as np
from wrs.assembly.geometry._triangle_batch import triangle_pairs
from wrs.assembly.geometry.mesh_bvh import triangle_pair,closest_on_triangle


class TriangleBatchTests(unittest.TestCase):
    def test_scalar_agreement_random_degenerate_coplanar_and_piercing(self):
        rng=np.random.default_rng(20260911)
        a=rng.normal(size=(160,3,3)); b=rng.normal(size=(160,3,3))
        a[:15,1]=a[:15,0]; b[5:20,1]=b[5:20,0]
        a[30:40,:,2]=0; b[30:40,:,2]=0
        a[40:50,1:]=a[40:50,:1]
        a[50]=[[-1,-1,0],[1,-1,0],[0,1,0]]
        b[50]=[[0,0,-1],[0,0,1],[2,0,1]]
        for scale in (1e-4,1,1e4):
            aa,bb=a*scale,b*scale; tol=scale*1e-10
            pa,pb=triangle_pairs(aa,bb,tol)
            reference=[triangle_pair(x,y,tol) for x,y in zip(aa,bb)]
            expected=np.array([np.linalg.norm(x-y) for x,y,_ in reference])
            np.testing.assert_allclose(np.linalg.norm(pa-pb,axis=1),expected,atol=scale*1e-10,rtol=1e-9)
            self.assertLess(np.linalg.norm(pa[50]-pb[50]),tol)
            for p,q,ta,tb in zip(pa,pb,aa,bb):
                self.assertLessEqual(np.linalg.norm(p-closest_on_triangle(p,ta)),scale*1e-9)
                self.assertLessEqual(np.linalg.norm(q-closest_on_triangle(q,tb)),scale*1e-9)


if __name__=='__main__': unittest.main()
