"""Fit orthogonal course lines; manual motion establishes topology, not exact shape."""
import math
import statistics
from mission import simplify, length


def weighted_median(values):
    values=sorted(values);total=sum(w for v,w in values);seen=0.
    for v,w in values:
        seen+=w
        if seen>=total/2:return v
    raise ValueError('No weighted samples')


def point_line_distance(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1];d=dx*dx+dy*dy
    u=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/d))
    return math.hypot(p[0]-a[0]-u*dx,p[1]-a[1]-u*dy)


def fit(source):
    # Coarse RDP determines only the succession of straight corridors and turns.
    # It is never used as evidence that a shortcut is collision free.
    major=simplify(source,.16)
    if len(major)!=11:raise ValueError('Expected ten major legs from this captured course; review topology')
    indices=[0];previous=0
    for p in major[1:]:
        i=min(range(previous+1,len(source)),key=lambda i:math.dist(source[i],p))
        indices.append(i);previous=i
    angles=[]
    for a,b in zip(major,major[1:]):
        d=math.dist(a,b);angle=(math.atan2(b[1]-a[1],b[0]-a[0])+math.pi/4)%(math.pi/2)-math.pi/4
        angles.append((angle,d))
    angle=weighted_median(angles)
    if abs(angle)>math.radians(12):raise ValueError('Course dominant axes need independent review')
    co,si=math.cos(angle),math.sin(angle)
    def local(p):return (co*p[0]+si*p[1],-si*p[0]+co*p[1])
    def world(p):return [co*p[0]-si*p[1],si*p[0]+co*p[1]]
    axes=[local(p) for p in source];directions=[];offsets=[];fits=[]
    for i,(a,b) in enumerate(zip(major,major[1:])):
        aa,bb=local(a),local(b);horizontal=abs(bb[0]-aa[0])>abs(bb[1]-aa[1])
        directions.append('horizontal' if horizontal else 'vertical')
        if i and directions[i]==directions[i-1]:raise ValueError('Major corner did not change corridor direction')
        lo,hi=indices[i:i+2];count=hi-lo+1
        trim=max(1,int(count*.15)) if count>8 else 0
        support=axes[lo+trim:hi+1-trim]
        offset=statistics.median(p[1 if horizontal else 0] for p in support)
        # The field registration and confirmed task centre are endpoint anchors.
        if i==0:
            if not horizontal:raise ValueError('First corridor must follow the first field axis')
            offset=0.
        if i==9:offset=local(source[-1])[1 if horizontal else 0]
        offsets.append(offset)
        residual=[abs(p[1 if horizontal else 0]-offset) for p in support]
        fits.append(dict(segment=i+1,axis=directions[-1],support_count=len(support),
                         residual_median_m=statistics.median(residual),residual_max_m=max(residual)))
    corners=[[0.,0.]]
    for i in range(1,10):
        if directions[i-1]=='horizontal':uv=(offsets[i],offsets[i-1])
        else:uv=(offsets[i-1],offsets[i])
        corners.append(world(uv))
    corners.append(list(source[-1]))
    for i,(a,b) in enumerate(zip(corners,corners[1:])):
        if math.dist(a,b)<.4:raise ValueError('A fitted course leg collapsed')
        old=(major[i+1][0]-major[i][0],major[i+1][1]-major[i][1])
        new=(b[0]-a[0],b[1]-a[1])
        if old[0]*new[0]+old[1]*new[1]<=0:raise ValueError('Fitting reversed a corridor')
    displacement=max(min(point_line_distance(p,a,b) for a,b in zip(corners,corners[1:])) for p in source)
    return corners,dict(method='Weighted dominant orthogonal axes; per-corridor trimmed median offsets; intersect adjacent lines',
        field_axis_angle_deg=math.degrees(angle),major_segments=10,manual_wobble_retained=False,
        straight_lines= fits,maximum_recorded_to_planned_offset_m=displacement,
        measured_wall_centrelines=False,collision_certified=False,
        source_use='Recorded path determines corridor order and approximate location; not a surveyed free-space boundary')
