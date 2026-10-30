"""Bounds and channel-invariance checks for file-only photometry diagnostics."""
import unittest
import hashlib
import json
import tempfile
import cv2
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from tools.analyze_lightbar_photometry import analyze,inspect,sample


class PhotometryTest(unittest.TestCase):
    def setUp(self):
        self.image=np.full((100,120,3),12,np.uint8)
        self.image[25:76,25:36]=[255,95,30]
        self.image[25:76,85:96]=[255,95,30]
        self.corners=[[30,25],[90,25],[90,75],[30,75]]

    def test_channel_order_invariance(self):
        self.assertEqual(inspect(self.image,self.corners,{}),inspect(self.image[:,:,::-1],self.corners,{}))

    def test_bilinear_sample(self):
        self.assertAlmostEqual(sample(np.array([[0,10],[20,30]],np.uint8),[.25,.5]),12.5)
        with self.assertRaises(ValueError):sample(np.zeros((2,2)),[1,1])

    def test_uniform_bar_profile(self):
        bars=inspect(self.image,self.corners,{})
        self.assertEqual(len(bars),2)
        for b in bars:
            self.assertEqual(b['length_px'],50)
            self.assertGreater(b['axis_peak'],200)
            self.assertFalse(b['legacy_axis_below_threshold'])

    def test_invalid_points(self):
        for p in ([],np.full((4,2),np.nan),np.zeros((4,2))):
            with self.assertRaises(ValueError):inspect(self.image,p,{})

    def test_blank_image_is_low_signal(self):
        bars=inspect(np.zeros_like(self.image),self.corners,{})
        self.assertTrue(all(b['legacy_axis_below_threshold'] and b['candidate_axis_below_threshold'] for b in bars))

    def recording(self,folder,corrupt_hash=False):
        folder=Path(folder);image=folder/'image.png';assert cv2.imwrite(str(image),self.image)
        sha=hashlib.sha256(image.read_bytes()).hexdigest()
        events=[dict(kind='manifest',schema='sp-vision-events-1',configs=[dict(content_utf8='{}')]),
            dict(kind='image',data=dict(image_id='frame1',path='image.png',sha256='bad' if corrupt_hash else sha,height=100,width=120)),
            dict(kind='detections',data=dict(image_id='frame1',objects=[dict(label='four',confidence=.9,corners_px=self.corners)])),
            dict(kind='end',complete=True)]
        output=folder/'events.jsonl';output.write_text('\n'.join(json.dumps(r) for r in events)+'\n');return output

    def test_recording_schema_and_images(self):
        with tempfile.TemporaryDirectory() as folder:
            result=analyze(self.recording(folder))
            self.assertEqual(result['inspected_bars'],2)
            self.assertFalse(result['accuracy_measured'])
            self.assertFalse(result['physical_device_opened'])

    def test_changed_image_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(AssertionError):analyze(self.recording(folder,True))


if __name__=='__main__':unittest.main()
