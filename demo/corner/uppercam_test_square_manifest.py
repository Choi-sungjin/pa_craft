#!/usr/bin/env python3
import json,shutil,tempfile,unittest
from pathlib import Path
from uppercam_bridge_utils import load_validated_square_manifest,square_observation_policy,SQUARE_FILES
HERE=Path(__file__).resolve().parent;URDF=HERE.parents[1]/'src/craft_description/urdf/v307_full_robot.urdf';MANIFEST=HERE/'uppercam_square_far20_validation_manifest.json'
class ManifestTest(unittest.TestCase):
 def test_verified_and_tampered_artifacts(self):
  with tempfile.TemporaryDirectory() as d:
   dest=Path(d)
   for name in SQUARE_FILES:
    if name!='@urdf':shutil.copy(HERE/name,dest/name)
   urdf=dest/'v307.urdf';shutil.copy(URDF,urdf);model=dest/'uppercam_selected_calibration.json'
   load_validated_square_manifest(MANIFEST,dest,model,urdf)
   model.write_text(model.read_text()+' ')
   with self.assertRaisesRegex(ValueError,'mismatch'):load_validated_square_manifest(MANIFEST,dest,model,urdf)
   shutil.copy(HERE/'uppercam_selected_calibration.json',model)
   geom=dest/'uppercam_square_far20_estimator.py';geom.write_text(geom.read_text()+'\n# altered')
   with self.assertRaisesRegex(ValueError,'mismatch'):load_validated_square_manifest(MANIFEST,dest,model,urdf)
   shutil.copy(HERE/'uppercam_square_far20_estimator.py',geom)
   altered_manifest=dest/'manifest.json';altered_manifest.write_text(MANIFEST.read_text()+' ')
   with self.assertRaisesRegex(ValueError,'manifest mismatch'):load_validated_square_manifest(altered_manifest,dest,model,urdf)
 def test_policy_boundaries_and_disabled(self):
  r=dict(method='square_far20_spec_reimplementation_v1',provenance='spec_reimplementation',state='geometry_unverified',
         source_stamp_s=2.,gap_mm=10.,lateral_m=0.,box_yaw_rad=0.,extrapolated=False)
  self.assertEqual(square_observation_policy(r,True),(True,'square_far20 one-bag validated',.005))
  r['gap_mm']=80.;self.assertEqual(square_observation_policy(r,True)[2],.010)
  for gap in (0.,9.999,80.001,100.,200.):
   r['gap_mm']=gap;self.assertTrue(square_observation_policy(r,True)[0])
   self.assertIn('experiment',square_observation_policy(r,True)[1])
  for gap in (-.001,200.001,float('nan'),float('inf')):
   r['gap_mm']=gap;self.assertFalse(square_observation_policy(r,True)[0])
  r['gap_mm']=20.;r['extrapolated']=True;self.assertTrue(square_observation_policy(r,True)[0]);r['extrapolated']=False
  self.assertFalse(square_observation_policy(r,False)[0]);r['source_stamp_s']=None;self.assertFalse(square_observation_policy(r,True)[0])
if __name__=='__main__':unittest.main(verbosity=2)
