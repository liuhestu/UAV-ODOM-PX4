import copy
import math
import unittest
import numpy as np
import yaml
from test_support import ROOT
from uav_core.geometry import Adapter, rotation, quaternion, skew


def config():
    return yaml.safe_load((ROOT/'config/sources/mock.yaml').read_text())


class AdapterTests(unittest.TestCase):
    def test_reference_point_and_lever_velocity(self):
        c=config(); c['extrinsic']['translation_xyz']=[1,0,0]
        a=Adapter(c)
        result=a.adapt([2,3,4],[0,0,math.sqrt(.5),math.sqrt(.5)],[0,0,0],[0,0,2],np.eye(6),np.eye(6))
        np.testing.assert_allclose(result[0],[2,4,4],atol=1e-12)
        np.testing.assert_allclose(result[2],[0,2,0],atol=1e-12)
        np.testing.assert_allclose(rotation(result[1]),rotation([0,0,math.sqrt(.5),math.sqrt(.5)]),atol=1e-12)

    def test_rotations_and_world_velocity(self):
        c=config(); c['input']['linear_velocity_frame']='world'
        c['extrinsic']['rotation_xyzw']=[math.sqrt(.5),0,0,math.sqrt(.5)]
        a=Adapter(c); q=[0,0,math.sqrt(.5),math.sqrt(.5)]
        p,qout,v,w,pc,tc=a.adapt([0,0,0],q,[0,1,0],[0,0,1],np.eye(6),np.eye(6))
        np.testing.assert_allclose(v,[1,0,0],atol=1e-12)
        np.testing.assert_allclose(w,[0,1,0],atol=1e-12)
        np.testing.assert_allclose(rotation(qout),rotation(q)@a.R_sb,atol=1e-12)

    def test_body_orientation_covariance_lever_jacobian(self):
        c=config();c['input']['orientation_covariance_frame']='body';c['extrinsic']['translation_xyz']=[.3,-.4,.2]
        c['adapter']['pose_variance_floor']=[0]*6
        a=Adapter(c);R=rotation([.2,.3,.4,.8]);q=quaternion(R)
        pc=np.diag([.1,.2,.3,.01,.02,.03]); t=np.array(c['extrinsic']['translation_xyz'])
        result=a.adapt([0,0,0],q,[0,0,0],[0,0,0],pc,np.eye(6))
        J=np.zeros((6,6));J[:3,:3]=np.eye(3);J[:3,3:]=-R@skew(t);J[3:,3:]=R
        # Independent finite difference of the translated reference point.
        eps=1e-6
        for k in range(3):
            axis=np.eye(3)[k]; dR=rotation([*(axis*math.sin(eps/2)),math.cos(eps/2)])
            numeric=(R@dR@t-R@t)/eps
            np.testing.assert_allclose(numeric,J[:3,k+3],atol=1e-6)
        np.testing.assert_allclose(result[4],J@pc@J.T,atol=1e-12)

    def test_reject_invalid_input_and_covariance(self):
        a=Adapter(config())
        for q in ([0]*4,[float('nan'),0,0,1]):
            with self.assertRaises(ValueError): a.adapt([0]*3,q,[0]*3,[0]*3,np.eye(6),np.eye(6))
        pc=np.eye(6);pc[0,0]=-1
        with self.assertRaises(ValueError):a.adapt([0]*3,[0,0,0,1],[0]*3,[0]*3,pc,np.eye(6))
        pc=np.eye(6);pc[0,1]=1
        with self.assertRaises(ValueError):a.adapt([0]*3,[0,0,0,1],[0]*3,[0]*3,pc,np.eye(6))

    def test_quaternion_roundtrip_near_pi(self):
        for q in ([1,0,0,0],[0,1,0,0],[0,0,1,0],[.5,.5,.5,.5],[.001,.9999,.0001,.0002]):
            np.testing.assert_allclose(rotation(quaternion(rotation(q))),rotation(q),atol=1e-12)

    def test_world_alignment_and_covariance_psd(self):
        c=config();c['world_alignment']['rotation_xyzw']=[0,0,math.sqrt(.5),math.sqrt(.5)]
        c['world_alignment']['translation_xyz']=[3,4,5]
        c['extrinsic']['translation_xyz']=[.2,.1,.4]
        a=Adapter(c); rng=np.random.default_rng(4); M=rng.normal(size=(6,6));pc=M@M.T
        p,q,v,w,pc,tc=a.adapt([1,0,0],[0,0,0,1],[1,2,3],[.1,.2,.3],pc,np.eye(6))
        np.testing.assert_allclose(p,[2.9,5.2,5.4],atol=1e-12)
        self.assertGreaterEqual(np.linalg.eigvalsh(pc).min(),-1e-10)
        self.assertGreaterEqual(np.linalg.eigvalsh(tc).min(),-1e-10)
