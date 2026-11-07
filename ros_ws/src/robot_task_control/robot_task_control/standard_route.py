"""标准路线录制归档、质量报告和可移植数据导出；米、弧度、秒。"""
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import tempfile
from datetime import datetime, timezone
from uuid import uuid4


def inspect_standard(document):
    """输入完成的路线文档；返回质量报告，错误通过issues列出，不修改原始点列。"""
    issues = []
    if document.get('version') != 1 or document.get('valid') is not True or document.get('finished') is not True:
        issues.append('路线未完成或记录器已判无效')
    points = document.get('points', [])
    if not isinstance(points, list) or len(points) < 10:
        issues.append('标准路线至少需要10个采样点')
    if not isinstance(points, list) or not points or any(not isinstance(p, (list, tuple)) or len(p) != 3 or any(isinstance(x, bool) or not isinstance(x, (int, float))
            or not math.isfinite(x) for x in p) for p in points):
        return dict(accepted=False, issues=issues+['点位格式或数值无效'])
    if not document.get('frame_id') or not document.get('child_frame_id'):
        issues.append('缺少里程计坐标系')
    gaps = [math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:])]
    headings = [abs(math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))) for a,b in zip(points,points[1:])]
    length = sum(gaps)
    if length < .1 or max(gaps, default=0.) > .08:
        issues.append('长度不足10cm或采样间距超过8cm')
    if max(headings, default=0.) > .35:
        issues.append('相邻采样朝向跳变超过约20度')
    return dict(accepted=not issues, issues=issues, points=len(points), length_m=length,
                max_spacing_m=max(gaps,default=0.), median_spacing_m=statistics.median(gaps) if gaps else 0.,
                max_heading_step_rad=max(headings,default=0.), station_events=len(document.get('station_events', [])),
                note='合格仅说明记录结构和连续性通过检查，不代表物理定位精度或已验证可重复行驶')


def _atomic_json(path, value, compact=False):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(value, stream, ensure_ascii=False, allow_nan=False,
                      indent=None if compact else 2, separators=(',', ':') if compact else None)
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def publish_standard(snapshot_file, target_file, run_id):
    """停稳后调用：输入原始快照/标准文件/任务ID，返回版本路径；不合格记录保留但不更新标准。"""
    snapshot, target = Path(snapshot_file).expanduser(), Path(target_file).expanduser()
    if snapshot.resolve() == target.resolve():
        raise ValueError('标准文件不能与当次原始路线使用同一路径')
    raw = snapshot.read_bytes()
    document = json.loads(raw)
    report = inspect_standard(document)
    version = target.parent/'versions'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid4().hex[:8])
    version.mkdir(parents=True, exist_ok=False)
    (version/'raw_route.json').write_bytes(raw)
    _atomic_json(version/'quality_report.json', report)
    if not report['accepted']:
        raise ValueError('标准路线未通过质量检查：'+'；'.join(report['issues'])+'；原始记录保留在 '+str(version))
    standard = dict(document, standard_route=dict(schema='robot_standard_route_v1', run_id=run_id,
        version_dir=str(version), raw_sha256=hashlib.sha256(raw).hexdigest(), quality=report,
        start_pose=document['points'][0], end_pose=document['points'][-1],
        units=dict(position='m', angle='rad', timestamp='s'),
        references=['https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/recordreplay-planner-howto.html',
                    'https://github.com/ros-navigation/navigation2/tree/main/nav2_regulated_pure_pursuit_controller']))
    if len(json.dumps(standard,ensure_ascii=False,separators=(',',':')).encode()) > 4_000_000:
        report['accepted']=False
        report['issues'].append('标准路线超过4MB，需缩短或分段录制')
        _atomic_json(version/'quality_report.json', report)
        raise ValueError('标准路线文件过大，原始记录保留在 '+str(version))
    _atomic_json(version/'standard_route.json', standard, compact=True)
    with (version/'points.csv').open('w', newline='', encoding='utf-8') as stream:
        writer=csv.writer(stream)
        writer.writerow(['x_m','y_m','yaw_rad','arc_m','stamp_sec','vx_body_mps','vy_body_mps','w_radps'])
        arc=0.
        for i, point in enumerate(document['points']):
            if i: arc+=math.dist(point[:2],document['points'][i-1][:2])
            samples=document.get('point_samples', [])
            sample=samples[i] if i < len(samples) and isinstance(samples[i],dict) else {}
            velocity=sample.get('velocity') or [None,None,None]
            writer.writerow([*point, arc, sample.get('stamp_sec'), *velocity])
    xs,ys=zip(*[(p[0],p[1]) for p in document['points']])
    scale=700/max(max(xs)-min(xs), max(ys)-min(ys), .1)
    poly=' '.join(f'{40+(x-min(xs))*scale:.2f},{760-(y-min(ys))*scale:.2f}' for x,y in zip(xs,ys))
    (version/'preview.svg').write_text('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 800 800">'
        '<rect width="800" height="800" fill="white"/><text x="20" y="25">Recorded route (metres, odometry frame)</text>'
        f'<polyline points="{poly}" fill="none" stroke="blue" stroke-width="3"/></svg>')
    _atomic_json(target, standard, compact=True)  # 最后更新命名标准，失败时上一版仍可用。
    return dict(standard_file=str(target), version_dir=str(version), quality=report)
