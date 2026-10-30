#!/usr/bin/env python3
"""Rectify one existing example plate; manual anchors are approximate, not labels from YOLO."""
from pathlib import Path
import hashlib
import json
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def main():
    source=ROOT/'assets/test.png';image=cv2.imread(str(source))
    if image is None:raise RuntimeError('missing existing assets/test.png')
    # TL/TR/BR/BL light-bar centre endpoints, manually inspected in the existing
    # 639x720 photograph. A few original pixels of annotation uncertainty remain.
    anchors=np.float32([[491,492],[589,469],[591,518],[492,548]])
    target=np.float32([[64,112],[448,112],[448,272],[64,272]])
    texture=cv2.warpPerspective(image,cv2.getPerspectiveTransform(anchors,target),(512,384))
    values=texture.astype(np.int16);blue,green,red=cv2.split(values)
    mask=(red-blue>25)&(red-green>4)&(red>70)
    recolored=texture.copy();recolored[mask,0]=texture[mask,2];recolored[mask,2]=texture[mask,0]
    folder=ROOT/'data/fixed-target-input';folder.mkdir(parents=True,exist_ok=True)
    path=folder/'photo-armor-three.png'
    if not cv2.imwrite(str(path),recolored):raise RuntimeError('cannot write texture')
    manifest={'source':'assets/test.png','source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'source_anchor_tl_tr_br_bl_px':anchors.tolist(),'target_anchor_tl_tr_br_bl_px':target.tolist(),
        'texture':'data/fixed-target-input/photo-armor-three.png','texture_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
        'scope':'existing example photograph rectified/recolored red->blue; not new robot capture or photorealistic validation',
        'anchor_uncertainty':'manual, uncalibrated photo, approximately a few original pixels; not detector-generated ground truth'}
    (folder/'texture-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest))


if __name__=='__main__':main()
