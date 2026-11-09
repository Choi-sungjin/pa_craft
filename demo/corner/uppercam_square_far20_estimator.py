"""Independent square_far20 specification implementation; not original source.

Uses the supplied height-plane detector with a 20 mm depth-drop boundary,
then jointly fits centre x/y and yaw to the far and lateral square edges.
The supplied selected calibration and temporal filter are unchanged, but this
implementation remains diagnostic-only until independently validated.
"""
import math
from pathlib import Path
import numpy as np
from scipy import ndimage
from scipy.optimize import least_squares
import uppercam_box_range as br
import uppercam_fast5_box_range as fast
from uppercam_estimator import UppercamEstimator
from uppercam_selected_adapter import SelectedPostprocessor

SIDE=.205
METHOD='square_far20_spec_reimplementation_v1'

def fit_square(points_xy, initial):
    """Robustly fit far edge and sides of a 205 mm square to visible contour.

    Initial pose is centre x/y and forward-facing normal yaw in base_link.
    Ignore the near-edge interior, because an occluding robot can cut it off.
    """
    pts=np.asarray(points_xy,float);initial=np.asarray(initial,float)
    if pts.ndim!=2 or pts.shape[1]!=2 or len(pts)<30 or initial.shape!=(3,) or not np.isfinite(pts).all() or not np.isfinite(initial).all():
        raise ValueError('invalid or insufficient square boundary points')
    half=SIDE/2
    def coordinates(p):
        dx=pts[:,0]-p[0];dy=pts[:,1]-p[1];c=np.cos(p[2]);s=np.sin(p[2])
        return c*dx+s*dy,-s*dx+c*dy
    # Reject the artificial near boundary of a clipped top; retain visible side
    # segments and the far edge. This selection is fixed for the optimization.
    x,y=coordinates(initial)
    keep=(x>-.075)|(np.abs(np.abs(y)-half)<.018)
    pts=pts[keep]
    if len(pts)<30:raise ValueError('insufficient visible far and side edges')
    def residual(p):
        x,y=coordinates(p)
        far=x-half;side=np.abs(y)-half
        far_abs=np.abs(far)+np.maximum(np.abs(y)-half,0)
        side_abs=np.abs(side)+np.maximum(np.abs(x)-half,0)
        return np.where(far_abs<=side_abs,far,side)
    lo=initial-np.array([.08,.08,math.radians(20)])
    hi=initial+np.array([.08,.08,math.radians(20)])
    fit=least_squares(residual,initial,bounds=(lo,hi),loss='soft_l1',f_scale=.005,max_nfev=80)
    if not fit.success or not np.isfinite(fit.x).all():raise ValueError('square fit failed')
    x,y=coordinates(fit.x);far=np.abs(x-half)<.012;side=np.abs(np.abs(y)-half)<.012
    if far.sum()<8 or side.sum()<8:raise ValueError('insufficient supported square edges')
    c=np.cos(fit.x[2]);s=np.sin(fit.x[2]);near=fit.x[:2]-half*np.array([c,s])
    return dict(center_x=float(fit.x[0]),center_y=float(fit.x[1]),box_yaw_rad=float(fit.x[2]),
                near_x=float(near[0]),lateral_m=float(near[1]),far_inliers=int(far.sum()),
                side_inliers=int(side.sum()),fit_cost=float(fit.cost))

class SquareFar20Estimator(UppercamEstimator):
    method=METHOD
    def __init__(self,urdf_path,selected_model_path=None):
        super().__init__(urdf_path)
        self.selected=SelectedPostprocessor(selected_model_path or Path(__file__).with_name('uppercam_selected_calibration.json'))
        self.geometry_history={}
    def reset(self):
        super().reset();self.selected.reset();self.geometry_history.clear()
    def geometry(self,depth_m,K):
        d=np.asarray(depth_m,float);K=np.asarray(K,float)
        if d.ndim!=2 or K.shape!=(3,3) or not np.isfinite(K).all() or K[0,0]<=0 or K[1,1]<=0:
            raise ValueError('invalid depth image or intrinsics')
        # The independently evaluated candidate refines the entire contour at 20 mm.
        top,cen,n,area,rms=fast.box_top(d,K,self.up,.6229,np.random.default_rng(0),.5,None,dict(br.RANSAC,edge_drop=.020))
        side_top=top
        corners,_=br.rectangle(top,K,cen,n)
        seen=br.far_corner_visibility(d,K,cen,n,corners,self.up)
        square,flags=br.complete_square_far(corners,self.up,seen)
        Sb=square@self.R.T+self.t
        tangent=Sb[1,:2]-Sb[0,:2]
        if tangent[1]<0:tangent=-tangent
        yaw=math.atan2(-tangent[0],tangent[1]); initial=np.array([Sb[:,0].mean(),Sb[:,1].mean(),yaw])
        def project_boundary(mask):
            edge=mask & ~ndimage.binary_erosion(mask);rows,cols=np.nonzero(edge)
            visible=(rows>2)&(rows<d.shape[0]-3)&(cols>2)&(cols<d.shape[1]-3);rows,cols=rows[visible],cols[visible]
            if len(rows)>800:
                take=np.linspace(0,len(rows)-1,800,dtype=int);rows,cols=rows[take],cols[take]
            ray=np.column_stack(((cols-K[0,2])/K[0,0],(rows-K[1,2])/K[1,1],np.ones(len(rows))))
            den=ray@n
            if np.any(np.abs(den)<1e-9):raise ValueError('degenerate projection')
            return (ray*((cen@n)/den)[:,None]@self.R.T+self.t)[:,:2]
        far_points=project_boundary(top);side_points=project_boundary(side_top)
        c,s=math.cos(yaw),math.sin(yaw)
        delta=far_points-initial[:2];far_x=c*delta[:,0]+s*delta[:,1]
        delta=side_points-initial[:2];side_x=c*delta[:,0]+s*delta[:,1];side_y=-s*delta[:,0]+c*delta[:,1]
        boundary=np.vstack((far_points[far_x>.04],side_points[np.abs(side_y)>.075]))
        out=fit_square(boundary,initial)
        out.update(raw_gap_mm=(out['near_x']-.19067)*1000-19,flags=flags,plane_rmse_mm=float(rms*1000),
                   top_area_m2=float(area),provenance='spec_reimplementation',method=METHOD)
        return out
    def estimate(self,depth_m,K,frame_index,timestamp_s,frame_id):
        try:
            geo=self.geometry(depth_m,K)
            return self.estimate_from_geometry(geo,frame_index,timestamp_s,frame_id)
        except (ValueError,TypeError,RuntimeError,ArithmeticError,IndexError,KeyError,np.linalg.LinAlgError) as exc:
            return self.estimate_from_geometry(None,frame_index,timestamp_s,frame_id,error=str(exc))

    def estimate_from_geometry(self,geo,frame_index,timestamp_s,frame_id,error=None):
        """Apply the single causal filter to a geometry result in capture order."""
        result=dict(valid=False,state='invalid',gap_mm=None,raw_gap_mm=None,stamp_s=timestamp_s,
                    capture_stamp_s=timestamp_s,source_stamp_s=None,frame_index=frame_index,
                    source_frame_index=None,sample_age_s=None,frame_id=frame_id,
                    geometry_frame_id='base_link',method=METHOD,provenance='spec_reimplementation',extrapolated=False)
        try:
            if not frame_id or not math.isfinite(float(timestamp_s)):raise ValueError('invalid frame or stamp')
            if error is not None or geo is None:raise ValueError(error or 'missing geometry')
            f=self.selected.update(geo['raw_gap_mm'],frame_index,timestamp_s)
            if f['state']=='invalid':
                self.reset();return result
            self.geometry_history[int(frame_index)]=(float(timestamp_s),geo)
            retained={item['index'] for item in self.selected.filter.history}
            self.geometry_history={index:value for index,value in self.geometry_history.items() if index in retained}
            result.update(geo,current_raw_gap_mm=geo['raw_gap_mm'],state=f['state'],
                          gap_mm=f['diagnostic_gap_mm'],extrapolated=f['extrapolated'],
                          current_extrapolated=f.get('current_extrapolated',False))
            if f.get('source_frame_index') is not None:
                selected=f['source_frame_index'];source_stamp,source_geo=self.geometry_history[selected]
                result.update(source_geo,source_stamp_s=source_stamp,source_frame_index=selected,
                              sample_age_s=f['sample_age_s'])
            return result
        except (ValueError,TypeError,RuntimeError,ArithmeticError,IndexError,KeyError,np.linalg.LinAlgError) as exc:
            self.reset();result['error']=str(exc);return result
