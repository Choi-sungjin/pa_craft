"""Independent robust far-edge candidate; not the selected constrained_square."""
import math
import numpy as np
import uppercam_box_range as br
from uppercam_estimator import UppercamEstimator

METHOD = 'robust_far_edge_binned_median_v1'
class RobustUppercamEstimator(UppercamEstimator):
    method = METHOD
    top_detector = staticmethod(br.box_top)
    def geometry(self, depth_m, K):
        depth_m=np.asarray(depth_m,dtype=float);K=np.asarray(K,dtype=float)
        if depth_m.ndim!=2 or K.shape!=(3,3) or not np.isfinite(K).all() or K[0,0]<=0 or K[1,1]<=0:
            raise ValueError('invalid image or intrinsics')
        top,cen,n,area,rms=self.top_detector(depth_m,K,self.up,.6229,np.random.default_rng(0),.5,None)
        C,px=br.rectangle(top,K,cen,n)
        fwd=np.array([0.,0.,1.])-self.up[2]*self.up;fwd/=np.linalg.norm(fwd)
        a,b=C[np.argsort(C@fwd)[-2:]];along=(b-a)/np.linalg.norm(b-a)
        toward=np.cross(self.up,along)
        if toward@fwd>0:toward=-toward
        rows,cols=np.nonzero(top)
        rays=np.column_stack(((cols-K[0,2])/K[0,0],(rows-K[1,2])/K[1,1],np.ones(len(rows))))
        P=rays*((cen@n)/(rays@n))[:,None]
        midpoint=(a+b)/2;u=(P-midpoint)@along;v=(P-midpoint)@toward
        # Far edge is the median across 17 lateral strips: a small local mask
        # protrusion cannot move the entire edge. Ignore side-border strips.
        edge=[]
        for left in np.linspace(-.08,.08,18)[:-1]:
            vals=v[(u>=left)&(u<left+.16/17)]
            if len(vals)>=8:edge.append(float(np.percentile(vals,2)))
        if len(edge)<12:raise ValueError('insufficient far-edge coverage')
        far_offset=float(np.median(edge)); spread=float(np.percentile(edge,90)-np.percentile(edge,10))
        far_mid=midpoint+toward*far_offset
        square=np.array([far_mid-along*br.SIDE/2,far_mid+along*br.SIDE/2,
                         far_mid+along*br.SIDE/2+toward*br.SIDE,far_mid-along*br.SIDE/2+toward*br.SIDE])
        pts=square@self.R.T+self.t; near=pts[np.argsort(pts[:,0])[:2]].mean(axis=0)
        tangent=pts[1,:2]-pts[0,:2]
        if tangent[1]<0:tangent=-tangent
        vals=[float((near[0]-.19067)*1000.-19),float(near[1]),math.atan2(-tangent[0],tangent[1])]
        if not np.isfinite(vals).all():raise ValueError('nonfinite geometry')
        return dict(raw_gap_mm=vals[0],lateral_m=vals[1],box_yaw_rad=vals[2],flags=[],
                    far_edge_spread_mm=spread*1000,far_edge_strips=len(edge),
                    plane_rmse_mm=float(rms*1000),top_area_m2=float(area))
