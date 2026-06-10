// Extract the REAL webui functions from interactive.html (balanced-brace scan)
// and assemble legs from the planner output exactly as the browser would.
const fs = require('fs');
const html = fs.readFileSync(process.argv[2], 'utf8');
function extract(name) {
  const sig = 'function ' + name + '(';
  const i = html.indexOf(sig);
  if (i < 0) throw new Error('not found: ' + name);
  let j = html.indexOf('{', i), depth = 0, k = j;
  for (; k < html.length; k++) {
    if (html[k] === '{') depth++;
    else if (html[k] === '}') { depth--; if (depth === 0) break; }
  }
  return html.slice(i, k + 1);
}
const fns = ['lbLatlonToEN','lbPolylineLenM','lbDeCast','lbBezierSample',
  'lbStepRecipeLocal','lbSlalomRecipeLocal','lbRecipeLocal',
  'lbPlaceAtStartPose','lbExpLatLon','lbExpEnd','lbRepoCtrl','lbRepoSamples',
  'lbBuiltLegs'];
let src = fns.map(extract).join('\n');
// Stubs for the DOM fallbacks lbBuiltLegs touches when a repo entry is absent.
const $ = (id) => ({ value: id === 'lb-repo-v' ? '0.4' : '0.15' });
let lbExps = [], lbRepos = [];
eval(src);
const data = JSON.parse(fs.readFileSync(process.argv[3], 'utf8'));
const out = [];
for (const st of data.plan.stages) {
  lbExps = st.experiments.map(e => ({ id: e.id, recipe: e.recipe,
    start: { lat: e.start.lat, lon: e.start.lon, heading_deg: e.start.heading_deg } }));
  lbRepos = st.glues.map(g => ({ mids: g.mids || [], waypoints: g.waypoints || null,
    v_const: g.v_const, pos_tol_m: g.pos_tol_m }));
  out.push({ name: st.name, legs: lbBuiltLegs() });
}
fs.writeFileSync(process.argv[4], JSON.stringify(out));
console.log('assembled', out.length, 'stages via real webui JS');
