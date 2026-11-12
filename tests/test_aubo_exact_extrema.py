"""Independent polynomial-root and dense-grid verification of exact planning."""
import math,random,time,unittest
import numpy as np
from numpy.polynomial import Polynomial
from teleop.aubo.trajectory import JointTrajectory

class ExactExtremaTests(unittest.TestCase):
    def test_unit_roots_include_repeated_boundary_and_degenerate_roots(self):
        for roots in ((0.,.2,.7,1.),(.3,.3,.8,.8),(.4,.4,.4),(.5,)):
            p=Polynomial.fromroots(roots)
            found=JointTrajectory._roots_unit(tuple(p.coef)+(0.,0.))
            for x in roots:self.assertLess(min(abs(x-y) for y in found),2e-6)
        self.assertEqual(JointTrajectory._roots_unit((1.,0.,0.)),())

    def test_extrema_match_independent_power_basis_roots(self):
        rng=random.Random(932)
        for degree in range(2,6):
            for _ in range(80):
                points=[rng.uniform(-4,4) for _ in range(degree+1)]
                # Independent Bernstein expansion using Polynomial arithmetic.
                u=Polynomial((0.,1.));p=sum(x*math.comb(degree,i)*u**i*(1-u)**(degree-i) for i,x in enumerate(points))
                candidates=[0.,1.]+[float(x.real) for x in p.deriv().roots() if abs(x.imag)<1e-8 and 0.<x.real<1.]
                expected=[p(x) for x in candidates]
                actual=[value for _,value in JointTrajectory.extrema(points)]
                self.assertAlmostEqual(min(actual),min(expected),places=9)
                self.assertAlmostEqual(max(actual),max(expected),places=9)

    def test_rest_steps_match_analytic_limits_without_convex_hull_penalty(self):
        for profile,limits in [('balanced',(25,75,600)),('practical',(45,180,1800))]:
            for delta in (5,10,20):
                t=JointTrajectory([0.]*6,[[-3,3]]*6,profile=profile)
                t.advance([math.radians(delta)]*6,.005)
                v,a,j=limits
                expected=max(1.875*delta/v,math.sqrt(10*math.sqrt(3)*delta/(3*a)),(60*delta/j)**(1/3))*1.01
                self.assertAlmostEqual(t.duration,expected,places=10)
                self.assertTrue(t.curve_within(t.controls[0],t.duration,(-3,3),(t.velocity_rad_s,t.acceleration_rad_s2,t.jerk_rad_s3)))
        self.assertLess(t.duration,.9)

    def test_position_range_uses_quartic_roots_not_control_point_rejection(self):
        p=(0.,0.,1.5,-1.5,0.,0.)
        self.assertTrue(JointTrajectory.curve_within(p,10.,(-1,1),(100,100,100)))
        self.assertFalse(JointTrajectory.curve_within(p,10.,(-.1,.1),(100,100,100)))

    def test_moving_replans_and_brakes_pass_independent_dense_derivative_checks(self):
        t=JointTrajectory([0.]*6,[[-3,3]]*6,profile='practical')
        u=np.linspace(0,1,2001)
        for step in range(120):
            qva=t.q,t.v,t.a
            goal=[math.radians(40)*math.sin(step*.13+i) for i in range(6)]
            t._plan(tuple(goal))
            self.assertEqual(qva,(t.q,t.v,t.a))
            if step%10==0:
                for points in t.controls:
                    n=len(points)-1
                    p=sum(x*math.comb(n,i)*Polynomial((0,1))**i*Polynomial((1,-1))**(n-i) for i,x in enumerate(points))
                    for limit in (t.velocity_rad_s,t.acceleration_rad_s2,t.jerk_rad_s3):
                        p=p.deriv()/t.duration
                        self.assertLessEqual(float(np.max(np.abs(p(u)))),limit+1e-8)
            t.advance(goal,.005)
        qva=t.q,t.v,t.a;target,duration=t.brake()
        self.assertEqual(qva,(t.q,t.v,t.a))
        for points in t.controls:
            n=len(points)-1
            p=sum(x*math.comb(n,i)*Polynomial((0,1))**i*Polynomial((1,-1))**(n-i) for i,x in enumerate(points))
            for limit in (t.velocity_rad_s,t.acceleration_rad_s2,t.jerk_rad_s3):
                p=p.deriv()/duration
                self.assertLessEqual(float(np.max(np.abs(p(u)))),limit+1e-8)
