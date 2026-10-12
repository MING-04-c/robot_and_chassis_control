// Run with: gjs camer_diver/visualization/test_line_scan.js
// Exercises the actual HTML scanner without requiring a browser or a camera.
const Gio = imports.gi.Gio;
const GLib = imports.gi.GLib;
const scriptPath = GLib.path_get_dirname(imports.system.programPath);
const [, contents] = Gio.File.new_for_path(scriptPath + '/line_scan.html').load_contents(null);
const html = new TextDecoder().decode(contents);
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1]
  .replace('reset();requestAnimationFrame(animate);', '');
const elements = new Map();
const context = new Proxy({}, {get: (target, key) => target[key] || (() => {})});
globalThis.document = {
  getElementById(id) {
    if (!elements.has(id)) elements.set(id, {
      value: ({stride:'6',width:'48',speed:'6',scene:'sharp',floor:'white'})[id] || '',
      checked:false, classList:{toggle() {}}, addEventListener() {}, getContext:() => context,
    });
    return elements.get(id);
  },
  createElement() {return {getContext:() => context};},
};
globalThis.performance = {now:() => 0};
globalThis.requestAnimationFrame = () => {};

const tests = `
function assert(condition,message) {if(!condition)throw new Error(message);}
function prepare(ranges) {
  results=[];points=[];seed=W/2;nextY=H-24;
  mask=new Uint8Array(W*H);
  for(const [left,right] of ranges)mask.fill(1,nextY*W+left,nextY*W+right+1);
  beginRow();advancePixels(W);
}
prepare([[400,560]]);
assert(current.L===399.5 && current.R===560.5 && current.mid===480 && current.valid,
  'Normal stripe boundaries/midpoint wrong');
beginRow();assert(current.seed===480 && current.y===610,'Previous-row seed not propagated');
prepare([[400,959]]);
assert(current.R===959 && current.rightEdge && !current.valid,'Edge fallback not marked lost');
prepare([]);
assert(current.L===0 && current.R===959 && current.mid===479.5 && !current.valid,
  'Empty row fallback wrong');
prepare([[100,140],[800,840]]);
assert(current.L===140.5 && current.R===799.5 && !current.valid,
  'Background seed / unrelated jumps should be invalid');
assert(isBlue(23,107,218) && !isBlue(241,240,233) && !isBlue(221,199,160),
  'Blue HSV segmentation wrong');
assert(isBlue(52,116,175),'Blue-floor interference missing');

// Independently rasterize the sharp-turn path by distance to its segments.
points=generatePath('sharp');mask=new Uint8Array(W*H);
const halfWidth=24;
for(let y=H-24;y>=24;y-=6) {
  for(let x=0;x<W;x++) {
    for(let i=1;i<points.length;i++) {
      const a=points[i-1],b=points[i],dx=b.x-a.x,dy=b.y-a.y;
      if(x<Math.min(a.x,b.x)-halfWidth || x>Math.max(a.x,b.x)+halfWidth ||
        y<Math.min(a.y,b.y)-halfWidth || y>Math.max(a.y,b.y)+halfWidth)continue;
      const t=Math.max(0,Math.min(1,((x-a.x)*dx+(y-a.y)*dy)/(dx*dx+dy*dy)));
      if((x-a.x-t*dx)**2+(y-a.y-t*dy)**2<=halfWidth*halfWidth) {
        mask[y*W+x]=1;break;
      }
    }
  }
}
results=[];seed=W/2;nextY=H-24;beginRow();
while(current) {
  advancePixels(W);
  if(nextY<24)break;
  beginRow();
}
const bend=results.filter(r=>r.valid&&r.ref.x!==null);
const worst=bend.reduce((a,b)=>Math.abs(b.mid-b.ref.x)>Math.abs(a.mid-a.ref.x)?b:a);
assert(Math.abs(worst.mid-worst.ref.x)>40,'Sharp turn did not demonstrate geometric bias');
assert(results.some(r=>!r.valid),'End-of-track loss not demonstrated');
print('PASS: stripe / propagated seed / frame edge / empty row / outside seed / HSV / sharp turn');
print('Synthetic 48px-wide sharp turn: worst valid horizontal-midpoint error = '+
  (worst.mid-worst.ref.x).toFixed(1)+' px at row y='+worst.y);
`;
new Function(script + '\n' + tests)();
